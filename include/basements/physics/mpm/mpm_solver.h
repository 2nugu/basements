#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include "basements/physics/mpm/particle.h"
#include "basements/physics/mpm/spgrid_cpu.h"
#include "basements/physics/mpm/mpm_autodiff_tape.h"
#include "basements/core/math/common.h"
#include "basements/core/math/svd.h"

namespace basements {
namespace mpm {

    class MPMSolver {
    public:
        MPMSolver() : grid(0.1f) {} 

        void initialize(float dx) {
            grid = SPGridCPU(dx);
        }

        void add_particle(const basements::math::Vec3& pos, const basements::math::Vec3& vel, float mass = 1.0f) {
            Particle p;
            p.position = pos;
            p.velocity = vel;
            p.mass = mass;
            // Reference volume V_p^0 from current bulk density. Required by the
            // MLS-MPM stress-force formula  f = -(4/dx^2) V_p^0 τ (x_i - x_p) w.
            p.volume = (density > 1e-12f) ? (mass / density) : 1.0f;
            p.F = basements::math::Matrix3::identity();
            p.C = basements::math::Matrix3::zero();
            particles.push_back(p);
        }

        // ── Material + integration parameters ─────────────────────────────
        // Young's modulus / Poisson ratio / bulk density. Defaults are a soft
        // sand (low E for CFL headroom at dt = 1/240, dx = 0.1).
        // Order-independent: if particles were added under the previous
        // density, their V_p^0 is rescaled (m/ρ_old → m/ρ_new) so the
        // call order with add_particle() does not matter.
        void set_material(float young_modulus, float poisson_ratio, float bulk_density) {
            E = young_modulus;
            nu = poisson_ratio;
            if (bulk_density > 1e-12f && density > 1e-12f
                && bulk_density != density && !particles.empty()) {
                const float rescale = density / bulk_density;
                for (auto& p : particles) p.volume *= rescale;
            }
            density = bulk_density;
        }
        void set_gravity(const basements::math::Vec3& g) { gravity = g; }
        void set_plastic_alpha(float a) { alpha = a; }
        void set_floor_friction(float mu_floor) { floor_friction = mu_floor; }

        // Test/coupling-hook access to the particle buffer. External code may
        // overwrite F, position, velocity between step()s (e.g. to seed a
        // known strain state in a constitutive unit test).
        std::vector<Particle>& get_particles_mutable() { return particles; }

        // Forward-only step. Hot path unchanged from pre-Pivot-6 behaviour.
        void step(float dt) {
            step(dt, nullptr);
        }

        // Opt-in differentiable step. Pivot 6 in-place autodiff design.
        //   tape == nullptr → identical to forward-only step (one branch).
        //   tape != nullptr → p2g / update / g2p record intermediate state
        //                      for downstream backward pass.
        // M1 milestone: API surface only; tape recording wires up in M2.
        void step(float dt, AutodiffTape* tape) {
            grid.clear();

            for (int i = 0; i < (int)particles.size(); ++i) {
                p2g_single(particles[i], dt);
                // M2 hook: if (tape) tape->push(... P2G_PARTICLE entry ...);
                (void)tape;
            }

            update_grid(dt);
            // M2 hook: if (tape) for-each-active-node tape->push(...);

            for (int i = 0; i < (int)particles.size(); ++i) {
                g2p_single(particles[i], dt);
                // M2 hook: if (tape) tape->push(... G2P_PARTICLE entry ...);
            }
        }
        
        const std::vector<Particle>& get_particles() const { return particles; }
        const SPGridCPU& get_grid() const { return grid; }

        // Mutable grid access — used by external coupling code (e.g. the
        // MPM ↔ rigid Dirichlet BC) that needs to mutate node velocities
        // between p2g/g2p phases of the solver.
        SPGridCPU& get_grid_mutable() { return grid; }

    private:
        std::vector<Particle> particles;
        SPGridCPU grid;
        basements::math::Vec3 gravity = basements::math::Vec3(0, -9.81f, 0);

        // Constitutive Model Parameters
        //   α = √(2/3) · 2 sinφ / (3 - sinφ)    [Klar 2016 eq. 3.2]
        //   φ = 35°  →  α ≈ 0.398               [dry sand, canonical]
        float alpha   = 0.40f;
        float E       = 1.0e5f;     // Young's modulus  [Pa] — soft sand default
        float nu      = 0.3f;       // Poisson ratio
        float density = 1500.0f;    // Bulk density     [kg/m^3]
        float floor_friction = 0.4f; // Coulomb μ for separable floor BC

        float mu_lame()     const { return E / (2.0f * (1.0f + nu)); }
        float lambda_lame() const { return E * nu / ((1.0f + nu) * (1.0f - 2.0f * nu)); }

        void apply_plasticity(Particle& p, float dt) {
            basements::math::Matrix3 I = basements::math::Matrix3::identity();
            basements::math::Matrix3 grad_v = p.C;
            
            basements::math::Matrix3 F_trial = (I + grad_v * dt) * p.F;

            // SVD
            basements::math::Matrix3 U, V;
            basements::math::Vec3 sigma;
            basements::math::SVD::compute(F_trial, U, sigma, V);
            
            // Safety clamp
            sigma.x = std::max(sigma.x, 1e-6f);
            sigma.y = std::max(sigma.y, 1e-6f);
            sigma.z = std::max(sigma.z, 1e-6f);

            basements::math::Vec3 epsilon(std::log(sigma.x), std::log(sigma.y), std::log(sigma.z));
            float trace_epsilon = epsilon.x + epsilon.y + epsilon.z;
            
            basements::math::Vec3 epsilon_hat = epsilon - basements::math::Vec3(trace_epsilon / 3.0f);
            float epsilon_hat_norm = epsilon_hat.length();
            
            // Drucker-Prager Yield Condition
            // trace > 0 means Tension (expand)
            if (trace_epsilon > 0.0f) {
                // Project to zero stress (tip of cone)
                epsilon = basements::math::Vec3(0,0,0); 
                sigma = basements::math::Vec3(1,1,1);
            } else {
                // Drucker-Prager yield in log-strain space (Klar 2016 eq. 3.1):
                //   ‖ε̂‖_max  =  -α · (3λ + 2μ)/(2μ) · tr(ε)
                // The (3λ+2μ)/(2μ) factor is the bulk-to-shear stiffness ratio
                // — without it, the cone is ~3× tighter than canonical and
                // sand yields far too easily.
                float H = epsilon_hat_norm;
                float kohesion = (3.0f * lambda_lame() + 2.0f * mu_lame())
                                 / (2.0f * mu_lame());
                float limit = -alpha * kohesion * trace_epsilon;

                if (H > limit) {
                    float scale = limit / H;
                    epsilon_hat = epsilon_hat * scale;
                    epsilon = epsilon_hat + basements::math::Vec3(trace_epsilon / 3.0f);
                    
                    sigma.x = std::exp(epsilon.x);
                    sigma.y = std::exp(epsilon.y);
                    sigma.z = std::exp(epsilon.z);
                }
            }

            basements::math::Matrix3 Sigma_mat = basements::math::Matrix3::scale(sigma);
            p.F = U * Sigma_mat * V.transposed();
        }

        void p2g_single(Particle& p, float dt) {
            float inv_dx = grid.get_inv_dx();
            float fx = p.position.x * inv_dx - 0.5f;
            float fy = p.position.y * inv_dx - 0.5f;
            float fz = p.position.z * inv_dx - 0.5f;

            int base_x = (int)std::floor(fx);
            int base_y = (int)std::floor(fy);
            int base_z = (int)std::floor(fz);

            float fx_fract = fx - base_x;
            float fy_fract = fy - base_y;
            float fz_fract = fz - base_z;

            float w_x[3] = { 0.5f * (1.0f - fx_fract)*(1.0f - fx_fract), 0.75f - (fx_fract - 0.5f)*(fx_fract - 0.5f), 0.5f * fx_fract*fx_fract };
            float w_y[3] = { 0.5f * (1.0f - fy_fract)*(1.0f - fy_fract), 0.75f - (fy_fract - 0.5f)*(fy_fract - 0.5f), 0.5f * fy_fract*fy_fract };
            float w_z[3] = { 0.5f * (1.0f - fz_fract)*(1.0f - fz_fract), 0.75f - (fz_fract - 0.5f)*(fz_fract - 0.5f), 0.5f * fz_fract*fz_fract };

            // ── Hencky-Kirchhoff stress τ_p from current deformation F ──────
            //   F = U Σ V^T,  ε = log Σ
            //   τ_diag = 2μ ε + λ tr(ε) I       (in U-eigenbasis)
            //   τ     = U diag(τ_diag) U^T      (world frame, Kirchhoff)
            // Klar 2016 eq. 26 / Hu 2018 §4.2.
            basements::math::Matrix3 U_F, V_F;
            basements::math::Vec3 sigma_F;
            basements::math::SVD::compute(p.F, U_F, sigma_F, V_F);

            sigma_F.x = std::max(sigma_F.x, 1e-6f);
            sigma_F.y = std::max(sigma_F.y, 1e-6f);
            sigma_F.z = std::max(sigma_F.z, 1e-6f);

            const float ln_sx = std::log(sigma_F.x);
            const float ln_sy = std::log(sigma_F.y);
            const float ln_sz = std::log(sigma_F.z);
            const float tr_ln = ln_sx + ln_sy + ln_sz;

            const float mu_  = mu_lame();
            const float lam_ = lambda_lame();
            basements::math::Vec3 tau_diag(
                2.0f * mu_ * ln_sx + lam_ * tr_ln,
                2.0f * mu_ * ln_sy + lam_ * tr_ln,
                2.0f * mu_ * ln_sz + lam_ * tr_ln
            );
            basements::math::Matrix3 Tau_diag_mat = basements::math::Matrix3::scale(tau_diag);
            basements::math::Matrix3 tau = U_F * Tau_diag_mat * U_F.transposed();

            // MLS-MPM force prefactor (Hu 2018 eq. 17):
            //   f_i = -Σ_p (4/dx^2) V_p^0 τ_p (x_i - x_p) w_ip
            const float force_factor = -4.0f * inv_dx * inv_dx * p.volume;
            basements::math::Matrix3 stress_kernel = tau * force_factor;

            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        float weight = w_x[i] * w_y[j] * w_z[k];
                        if (weight == 0.0f) continue;

                        int ni = base_x + i;
                        int nj = base_y + j;
                        int nk = base_z + k;

                        basements::math::Vec3 dist_vec(
                             ((float)(ni) - fx - 0.5f) * grid.get_dx(),
                             ((float)(nj) - fy - 0.5f) * grid.get_dx(),
                             ((float)(nk) - fz - 0.5f) * grid.get_dx()
                        );

                        basements::math::Vec3 affine = p.C * dist_vec;

                        GridNode* node = grid.get_node(ni, nj, nk);
                        float mass_contrib = weight * p.mass;

                        node->mass     += mass_contrib;
                        node->velocity += (p.velocity + affine) * mass_contrib;
                        // Internal stress reaction onto neighbouring nodes.
                        node->force    += (stress_kernel * dist_vec) * weight;
                        node->active = true;
                    }
                }
            }
        }

        void update_grid(float dt) {
            const float dx_local = grid.get_dx();
            auto& block_map = grid.get_blocks_mutable();
            for (auto& pair : block_map) {
                GridBlock& block = pair.second;
                for (int i = 0; i < BLOCK_SIZE*BLOCK_SIZE*BLOCK_SIZE; ++i) {
                    GridNode& node = block.nodes[i];
                    if (!node.active || node.mass <= 1e-6f) continue;

                    // v_i = (m·v + f·dt) / m   then   += g·dt
                    node.velocity = (node.velocity + node.force * dt) / node.mass;
                    node.velocity += gravity * dt;

                    // ── Separable floor BC at y = 0 with Coulomb friction ─
                    // Local-index decode: i = (li·BLOCK_SIZE + lj)·BLOCK_SIZE + lk.
                    int local_i = (i >> (2 * BLOCK_SIZE_LOG2));
                    int local_j = (i >> BLOCK_SIZE_LOG2) & BLOCK_MASK;
                    int local_k = i & BLOCK_MASK;
                    (void)local_i; (void)local_k; // X/Z indices not needed yet
                    int ny_idx = block.origin_y + local_j;
                    float node_y = (ny_idx + 0.5f) * dx_local;

                    if (node_y < dx_local && node.velocity.y < 0.0f) {
                        // Separable normal: prevent penetration only.
                        const float v_n_into_floor = -node.velocity.y; // > 0
                        node.velocity.y = 0.0f;

                        // Coulomb friction on tangent (μ_floor in [0,1]).
                        const float v_tan_x = node.velocity.x;
                        const float v_tan_z = node.velocity.z;
                        const float v_tan_mag = std::sqrt(v_tan_x*v_tan_x + v_tan_z*v_tan_z);
                        const float friction_budget = floor_friction * v_n_into_floor;
                        if (v_tan_mag > 1e-12f) {
                            const float scale = std::max(0.0f,
                                (v_tan_mag - friction_budget) / v_tan_mag);
                            node.velocity.x = v_tan_x * scale;
                            node.velocity.z = v_tan_z * scale;
                        }
                    }
                }
            }
        }
        
        void g2p_single(Particle& p, float dt) {
            float inv_dx = grid.get_inv_dx();
            float fx = p.position.x * inv_dx - 0.5f;
            float fy = p.position.y * inv_dx - 0.5f;
            float fz = p.position.z * inv_dx - 0.5f;
            
            int base_x = (int)std::floor(fx);
            int base_y = (int)std::floor(fy);
            int base_z = (int)std::floor(fz);
             
            float fx_fract = fx - base_x;
            float fy_fract = fy - base_y;
            float fz_fract = fz - base_z;

            float w_x[3] = { 0.5f * (1.0f - fx_fract)*(1.0f - fx_fract), 0.75f - (fx_fract - 0.5f)*(fx_fract - 0.5f), 0.5f * fx_fract*fx_fract };
            float w_y[3] = { 0.5f * (1.0f - fy_fract)*(1.0f - fy_fract), 0.75f - (fy_fract - 0.5f)*(fy_fract - 0.5f), 0.5f * fy_fract*fy_fract };
            float w_z[3] = { 0.5f * (1.0f - fz_fract)*(1.0f - fz_fract), 0.75f - (fz_fract - 0.5f)*(fz_fract - 0.5f), 0.5f * fz_fract*fz_fract };
            
            p.velocity = basements::math::Vec3(0,0,0);
            p.C = basements::math::Matrix3::zero();
            
             for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    for (int k = 0; k < 3; ++k) {
                        float weight = w_x[i] * w_y[j] * w_z[k];
                        if (weight == 0.0f) continue;
                        
                        int ni = base_x + i;
                        int nj = base_y + j;
                        int nk = base_z + k;
                        
                        GridNode* node = grid.get_node(ni, nj, nk);
                        
                        basements::math::Vec3 dist_vec(
                             ((float)(ni) - fx - 0.5f) * grid.get_dx(),
                             ((float)(nj) - fy - 0.5f) * grid.get_dx(),
                             ((float)(nk) - fz - 0.5f) * grid.get_dx()
                        );
                        
                        basements::math::Vec3 weighted_vel = node->velocity * weight;
                        
                        p.velocity += weighted_vel;
                        p.C = p.C + basements::math::Matrix3::outer_product(weighted_vel, dist_vec) * (4.0f * inv_dx * inv_dx);
                    }
                }
             }
             
             p.position += p.velocity * dt;
             
             apply_plasticity(p, dt);
             
             if (p.position.y < 0) {
                 p.position.y = 0;
                 p.velocity.y = 0;
             }
        }
    };

} // namespace mpm
} // namespace basements
