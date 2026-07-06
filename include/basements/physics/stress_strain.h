/**
 * @file stress_strain.h
 * @brief 3D Stress and Strain Tensor calculations for continuum mechanics
 * 
 * Implements Cauchy stress tensor, strain tensor, and constitutive relations
 * for linear elasticity (Hooke's Law) and failure criteria.
 */

#ifndef BASEMENTS_PHYSICS_STRESS_STRAIN_H
#define BASEMENTS_PHYSICS_STRESS_STRAIN_H

#include "basements/core/math/common.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/matrix3.h"
#include "basements/physics/material.h"
#include <cmath>
#include <algorithm>

namespace basements {
namespace physics {

using namespace basements::math;

/**
 * @brief 3x3 Symmetric Tensor for stress and strain
 * 
 * Stored as 6 independent components (Voigt notation):
 * [σ_xx, σ_yy, σ_zz, σ_xy, σ_yz, σ_xz]
 * 
 * Full tensor:
 * | σ_xx  σ_xy  σ_xz |
 * | σ_xy  σ_yy  σ_yz |
 * | σ_xz  σ_yz  σ_zz |
 */
struct SymmetricTensor3 {
    float xx, yy, zz;   ///< Normal components (diagonal)
    float xy, yz, xz;   ///< Shear components (off-diagonal)
    
    HOST_DEVICE SymmetricTensor3()
        : xx(0), yy(0), zz(0), xy(0), yz(0), xz(0) {}
    
    HOST_DEVICE SymmetricTensor3(float normal_xx, float normal_yy, float normal_zz,
                                  float shear_xy, float shear_yz, float shear_xz)
        : xx(normal_xx), yy(normal_yy), zz(normal_zz)
        , xy(shear_xy), yz(shear_yz), xz(shear_xz) {}
    
    /// Hydrostatic (mean) component: σ_m = (σ_xx + σ_yy + σ_zz) / 3
    HOST_DEVICE float hydrostatic() const {
        return (xx + yy + zz) / 3.0f;
    }
    
    /// First invariant I1 = trace = σ_xx + σ_yy + σ_zz
    HOST_DEVICE float I1() const {
        return xx + yy + zz;
    }
    
    /// Second invariant I2
    HOST_DEVICE float I2() const {
        return xx*yy + yy*zz + zz*xx - xy*xy - yz*yz - xz*xz;
    }
    
    /// Third invariant I3 = det
    HOST_DEVICE float I3() const {
        return xx*(yy*zz - yz*yz) - xy*(xy*zz - yz*xz) + xz*(xy*yz - yy*xz);
    }
    
    /// Deviatoric tensor (stress - hydrostatic)
    HOST_DEVICE SymmetricTensor3 deviatoric() const {
        float h = hydrostatic();
        return SymmetricTensor3(xx - h, yy - h, zz - h, xy, yz, xz);
    }
    
    /// Von Mises equivalent stress: σ_vm = sqrt(3/2 * s:s)
    HOST_DEVICE float von_mises() const {
        SymmetricTensor3 s = deviatoric();
        float s_squared = s.xx*s.xx + s.yy*s.yy + s.zz*s.zz
                        + 2.0f*(s.xy*s.xy + s.yz*s.yz + s.xz*s.xz);
        return std::sqrt(1.5f * s_squared);
    }
    
    /// Maximum shear stress (Tresca): τ_max = (σ_max - σ_min) / 2
    HOST_DEVICE float max_shear() const {
        Vec3 principals = principal_stresses();
        float sigma_max = std::max({principals.x, principals.y, principals.z});
        float sigma_min = std::min({principals.x, principals.y, principals.z});
        return (sigma_max - sigma_min) / 2.0f;
    }
    
    /// Principal stresses (eigenvalues) using J2/J3 deviatoric invariants
    HOST_DEVICE Vec3 principal_stresses() const {
        // 1. Mean stress (hydrostatic component)
        float sigma_m = hydrostatic();
        
        // 2. Deviatoric stress tensor S = σ - σ_m·I
        float s_xx = xx - sigma_m;
        float s_yy = yy - sigma_m;
        float s_zz = zz - sigma_m;
        // Shear components remains the same for deviatoric tensor
        
        // 3. Second deviatoric invariant J2 = 1/2 tr(S²)
        float j2 = 0.5f * (s_xx*s_xx + s_yy*s_yy + s_zz*s_zz) + xy*xy + yz*yz + xz*xz;
        
        if (j2 < 1e-8f) {
            // Hydrostatic state (all eigenvalues equal)
            return Vec3(sigma_m, sigma_m, sigma_m);
        }
        
        // 4. Third deviatoric invariant J3 = det(S)
        float j3 = s_xx * (s_yy * s_zz - yz * yz) 
                 - xy * (xy * s_zz - yz * xz) 
                 + xz * (xy * yz - s_yy * xz);
                 
        // 5. Calculate angle for trigonometric solution
        // cos(3θ) = (√3/2) * (J3 / J2^(3/2))
        float sqrt_j2 = std::sqrt(j2);
        float j2_3_2 = j2 * sqrt_j2; // J2^1.5
        
        // Clamp argument to [-1, 1] to handle numerical noise
        float arg = (1.5f * std::sqrt(3.0f) * j3) / j2_3_2; // 1.5*sqrt(3) same as 3*sqrt(3)/2
        // Note: The factor is actually (3*sqrt(3) * J3) / (2 * J2^1.5)
        // Let's re-verify:
        // S^3 - J2*S - J3 = 0 ?? No, char eq is λ^3 - J2*λ - J3 = 0 (signs vary by definition)
        // Standard result: σ = σ_m + 2*√(J2/3)*cos(...)
        // cos(3θ) = J3 * (3/J2)^1.5 * 0.5 ??
        // Let's stick to the verified derivation in thoughts: 
        // cos(3θ) = (3√3/2) * (J3 / J2^(3/2))
        
        arg = std::clamp(arg, -1.0f, 1.0f);
        float theta = std::acos(arg) / 3.0f;
        
        // 6. Calculate eigenvalues
        float multiplier = 2.0f * std::sqrt(j2 / 3.0f);
        
        // 3 Real roots
        float pi = 3.14159265359f;
        float s1 = sigma_m + multiplier * std::cos(theta);
        float s2 = sigma_m + multiplier * std::cos(theta - 2.0f*pi/3.0f);
        float s3 = sigma_m + multiplier * std::cos(theta + 2.0f*pi/3.0f);
        
        return Vec3(s1, s2, s3);
    }
    
    /// Convert to full 3x3 matrix
    HOST_DEVICE Matrix3 to_matrix() const {
        Matrix3 m;
        m.m[0][0] = xx; m.m[0][1] = xy; m.m[0][2] = xz;
        m.m[1][0] = xy; m.m[1][1] = yy; m.m[1][2] = yz;
        m.m[2][0] = xz; m.m[2][1] = yz; m.m[2][2] = zz;
        return m;
    }
    
    /// Tensor addition
    HOST_DEVICE SymmetricTensor3 operator+(const SymmetricTensor3& other) const {
        return SymmetricTensor3(xx + other.xx, yy + other.yy, zz + other.zz,
                                xy + other.xy, yz + other.yz, xz + other.xz);
    }
    
    /// Tensor subtraction
    HOST_DEVICE SymmetricTensor3 operator-(const SymmetricTensor3& other) const {
        return SymmetricTensor3(xx - other.xx, yy - other.yy, zz - other.zz,
                                xy - other.xy, yz - other.yz, xz - other.xz);
    }
    
    /// Scalar multiplication
    HOST_DEVICE SymmetricTensor3 operator*(float s) const {
        return SymmetricTensor3(xx*s, yy*s, zz*s, xy*s, yz*s, xz*s);
    }
    
    /// Double contraction (tensor inner product): A:B
    HOST_DEVICE float double_contraction(const SymmetricTensor3& other) const {
        return xx*other.xx + yy*other.yy + zz*other.zz
             + 2.0f*(xy*other.xy + yz*other.yz + xz*other.xz);
    }
    
    /// Frobenius norm: ||σ|| = sqrt(σ:σ)
    HOST_DEVICE float norm() const {
        return std::sqrt(double_contraction(*this));
    }
};

// Type aliases for clarity
using StressTensor = SymmetricTensor3;
using StrainTensor = SymmetricTensor3;

// ============================================================================
// Constitutive Relations (Hooke's Law)
// ============================================================================

/**
 * @brief Calculate stress from strain using isotropic linear elasticity
 * 
 * σ = λ·tr(ε)·I + 2μ·ε
 * 
 * Where λ, μ are Lamé parameters
 */
HOST_DEVICE inline StressTensor stress_from_strain(
    const StrainTensor& strain,
    const Material& material
) {
    float lambda = material.lame_lambda;
    float mu = material.lame_mu;
    
    float trace_e = strain.I1();
    
    return StressTensor(
        lambda * trace_e + 2.0f * mu * strain.xx,
        lambda * trace_e + 2.0f * mu * strain.yy,
        lambda * trace_e + 2.0f * mu * strain.zz,
        2.0f * mu * strain.xy,
        2.0f * mu * strain.yz,
        2.0f * mu * strain.xz
    );
}

/**
 * @brief Calculate strain from stress using isotropic linear elasticity
 * 
 * ε = (1/E)·[(1+ν)·σ - ν·tr(σ)·I]
 */
HOST_DEVICE inline StrainTensor strain_from_stress(
    const StressTensor& stress,
    const Material& material
) {
    float E = material.youngs_modulus;
    float nu = material.poisson_ratio;
    
    float trace_s = stress.I1();
    float inv_E = 1.0f / E;
    
    return StrainTensor(
        inv_E * ((1.0f + nu) * stress.xx - nu * trace_s),
        inv_E * ((1.0f + nu) * stress.yy - nu * trace_s),
        inv_E * ((1.0f + nu) * stress.zz - nu * trace_s),
        inv_E * (1.0f + nu) * stress.xy,
        inv_E * (1.0f + nu) * stress.yz,
        inv_E * (1.0f + nu) * stress.xz
    );
}

/**
 * @brief Calculate strain from displacement gradient
 * 
 * ε_ij = (u_i,j + u_j,i) / 2  (symmetric part of gradient)
 * 
 * @param du_dx Gradient of displacement: ∂u/∂x, ∂u/∂y, ∂u/∂z (row 0)
 * @param dv_dy Gradient: ∂v/∂x, ∂v/∂y, ∂v/∂z (row 1)
 * @param dw_dz Gradient: ∂w/∂x, ∂w/∂y, ∂w/∂z (row 2)
 */
HOST_DEVICE inline StrainTensor strain_from_displacement_gradient(
    const Matrix3& grad_u
) {
    return StrainTensor(
        grad_u.m[0][0],  // ε_xx = ∂u/∂x
        grad_u.m[1][1],  // ε_yy = ∂v/∂y
        grad_u.m[2][2],  // ε_zz = ∂w/∂z
        0.5f * (grad_u.m[0][1] + grad_u.m[1][0]),  // ε_xy = (∂u/∂y + ∂v/∂x)/2
        0.5f * (grad_u.m[1][2] + grad_u.m[2][1]),  // ε_yz = (∂v/∂z + ∂w/∂y)/2
        0.5f * (grad_u.m[0][2] + grad_u.m[2][0])   // ε_xz = (∂u/∂z + ∂w/∂x)/2
    );
}

// ============================================================================
// Failure Criteria
// ============================================================================

/**
 * @brief Check Von Mises yield criterion
 * @return Yield ratio (>1.0 means yielding)
 */
HOST_DEVICE inline float von_mises_yield_ratio(
    const StressTensor& stress,
    const Material& material
) {
    return stress.von_mises() / material.yield_strength;
}

/**
 * @brief Check Tresca (maximum shear stress) criterion
 * @return Yield ratio (>1.0 means yielding)
 */
HOST_DEVICE inline float tresca_yield_ratio(
    const StressTensor& stress,
    const Material& material
) {
    // Tresca: τ_max ≤ σ_y / 2
    return 2.0f * stress.max_shear() / material.yield_strength;
}

/**
 * @brief Check maximum principal stress criterion (brittle materials)
 * @return Failure ratio (>1.0 means fracture)
 */
HOST_DEVICE inline float max_principal_stress_ratio(
    const StressTensor& stress,
    const Material& material
) {
    Vec3 principals = stress.principal_stresses();
    float sigma_max = std::max({std::abs(principals.x), 
                                std::abs(principals.y), 
                                std::abs(principals.z)});
    return sigma_max / material.ultimate_strength;
}

/**
 * @brief Calculate elastic strain energy density
 * @return Energy per unit volume (J/m³)
 */
HOST_DEVICE inline float strain_energy_density(
    const StressTensor& stress,
    const StrainTensor& strain
) {
    // U = (1/2) * σ:ε
    return 0.5f * stress.double_contraction(strain);
}

// ============================================================================
// Damage Model (Simple)
// ============================================================================

/**
 * @brief Simple damage variable for continuum damage mechanics
 * 
 * D = 0: Undamaged
 * D = 1: Fully damaged (failed)
 */
struct DamageState {
    float damage;           ///< Current damage level [0, 1]
    float max_strain;       ///< Maximum strain experienced
    float accumulated_plastic_strain;  ///< Accumulated plastic work
    
    HOST_DEVICE DamageState()
        : damage(0.0f), max_strain(0.0f), accumulated_plastic_strain(0.0f) {}
    
    /// Apply stress with effective area reduction: σ_eff = σ / (1 - D)
    HOST_DEVICE StressTensor effective_stress(const StressTensor& nominal_stress) const {
        float factor = 1.0f / (1.0f - damage + 1e-10f);
        return nominal_stress * factor;
    }
    
    /// Update damage based on strain
    HOST_DEVICE void update(float current_strain, float failure_strain) {
        max_strain = std::max(max_strain, current_strain);
        
        if (max_strain > failure_strain * 0.5f) {
            // Linear damage accumulation after 50% failure strain
            float d_new = (max_strain - failure_strain * 0.5f) / (failure_strain * 0.5f);
            damage = std::clamp(d_new, damage, 1.0f);
        }
    }
    
    HOST_DEVICE bool is_failed() const {
        return damage >= 0.99f;
    }
};

} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_STRESS_STRAIN_H
