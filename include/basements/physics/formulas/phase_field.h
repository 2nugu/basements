#ifndef BASEMENTS_PHYSICS_FORMULAS_PHASE_FIELD_H
#define BASEMENTS_PHYSICS_FORMULAS_PHASE_FIELD_H

#include "basements/core/types.h"
#include "basements/core/math/vec3.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace phase_field {

    using Vec3 = basements::math::Vec3;

    // ========================================================================
    // Phase Field Theory (Ginzburg-Landau / Cahn-Hilliard)
    // ========================================================================

    /// Calculate Ginzburg-Landau Free Energy Density
    /// F = α(T-Tc)|ψ|² + ½β|ψ|⁴ + γ|∇ψ|²
    /// Here we compute the local potential part: V(ψ) = α(T-Tc)|ψ|² + ½β|ψ|⁴
    /// @param order_parameter psi (ψ)
    /// @param alpha Parameter α (related to coherence length)
    /// @param beta Parameter β (non-linear coupling)
    /// @param temperature Current T
    /// @param critical_temperature Tc
    HOST_DEVICE inline constexpr float calculate_gl_potential_energy_density(
        float order_parameter,
        float alpha,
        float beta,
        float temperature,
        float critical_temperature
    ) {
        float psi_sq = order_parameter * order_parameter;
        float term1 = alpha * (temperature - critical_temperature) * psi_sq;
        float term2 = 0.5f * beta * psi_sq * psi_sq;
        return term1 + term2;
    }

    /// Calculate Chemical Potential (Cahn-Hilliard)
    /// μ = ∂f/∂c - κ∇²c
    /// Here computing derivative of double-well potential f(c) = A*c²(1-c)²
    /// df/dc = 2Ac(1-c)(1-2c)
    /// @param concentration c (normalized 0 to 1 usually)
    /// @param energy_barrier A (height of barrier)
    HOST_DEVICE inline constexpr float calculate_double_well_potential_derivative(
        float concentration,
        float energy_barrier
    ) {
        // f(c) = A * c^2 * (1-c)^2
        // f'(c) = 2Ac(1-c)(1-2c)
        return 2.0f * energy_barrier * concentration * (1.0f - concentration) * (1.0f - 2.0f * concentration);
    }

    /// Calculate Allen-Cahn Reaction Term (Non-conserved order parameter)
    /// ∂η/∂t = -L * (∂f/∂η - κ∇²η)
    /// Returns local part: -L * ∂f/∂η
    /// Example potential: f(η) = ¼(1-η²)²  -> f'(η) = -η(1-η²) = η³ - η
    HOST_DEVICE inline constexpr float calculate_allen_cahn_reaction_term(
        float order_parameter,
        float mobility_L
    ) {
        // Derivative of f(eta) = 1/4(1-eta^2)^2 is f'(eta) = eta^3 - eta
        float reaction = order_parameter * order_parameter * order_parameter - order_parameter;
        return -mobility_L * reaction;
    }

    /// Calculate Gradient Energy Density
    /// E_grad = ½κ|∇φ|²
    HOST_DEVICE inline float calculate_gradient_energy_density(
        const Vec3& gradient_phi,
        float kappa_coefficient
    ) {
        return 0.5f * kappa_coefficient * gradient_phi.length_squared();
    }
    
    // ========================================================================
    // Liquid Crystal Physics (Landau-de Gennes)
    // ========================================================================
    
    /// Calculate Scalar Order Parameter S
    /// S = ½ <3 cos²θ - 1>
    /// @param cos_theta_avg Average of cos²θ
    HOST_DEVICE inline constexpr float calculate_nematic_order_parameter(float cos_theta_sq_avg) {
        return 0.5f * (3.0f * cos_theta_sq_avg - 1.0f);
    }

} // namespace phase_field
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_PHASE_FIELD_H
