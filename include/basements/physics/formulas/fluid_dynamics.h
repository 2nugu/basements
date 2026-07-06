#ifndef BASEMENTS_PHYSICS_FORMULAS_FLUID_DYNAMICS_H
#define BASEMENTS_PHYSICS_FORMULAS_FLUID_DYNAMICS_H

#include "basements/core/math/vec3.h"
#include "basements/core/types.h"
#include <cmath>

namespace basements {
namespace physics {
namespace formulas {
namespace fluid_dynamics {

using namespace basements::math;

// Physical constants
constexpr float WATER_DENSITY_KG_PER_CUBIC_METER = 1000.0f;        // kg/m³ at 4°C
constexpr float AIR_DENSITY_KG_PER_CUBIC_METER = 1.225f;           // kg/m³ at sea level, 15°C
constexpr float WATER_VISCOSITY_PASCAL_SECONDS = 0.001f;           // Pa·s at 20°C
constexpr float AIR_VISCOSITY_PASCAL_SECONDS = 1.81e-5f;           // Pa·s at 20°C
constexpr float ATMOSPHERIC_PRESSURE_PASCALS = 101325.0f;          // Pa

// ============================================================
// Continuity Equation (Mass Conservation)
// ============================================================

/// Calculate velocity from continuity equation (incompressible flow)
/// Formula: v₂ = v₁A₁/A₂
/// @param initial_velocity_meters_per_second Initial velocity (m/s)
/// @param initial_area_square_meters Initial cross-sectional area (m²)
/// @param final_area_square_meters Final cross-sectional area (m²)
/// @return Final velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_velocity_from_continuity_meters_per_second(
    float initial_velocity_meters_per_second,
    float initial_area_square_meters,
    float final_area_square_meters
) {
    return (final_area_square_meters > EPSILON) 
        ? (initial_velocity_meters_per_second * initial_area_square_meters / final_area_square_meters) 
        : 0.0f;
}

/// Calculate required area from continuity equation (incompressible flow)
/// Formula: A₂ = v₁A₁/v₂
/// @param initial_velocity_meters_per_second Initial velocity (m/s)
/// @param initial_area_square_meters Initial cross-sectional area (m²)
/// @param final_velocity_meters_per_second Final velocity (m/s)
/// @return Required final area in square meters (m²)
HOST_DEVICE inline constexpr float calculate_area_from_continuity_square_meters(
    float initial_velocity_meters_per_second,
    float initial_area_square_meters,
    float final_velocity_meters_per_second
) {
    return (constexpr_abs(final_velocity_meters_per_second) > EPSILON) 
        ? (initial_velocity_meters_per_second * initial_area_square_meters / final_velocity_meters_per_second) 
        : 0.0f;
}

/// Calculate mass flow rate
/// Formula: ṁ = ρAv
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param area_square_meters Cross-sectional area (m²)
/// @param velocity_meters_per_second Flow velocity (m/s)
/// @return Mass flow rate in kilograms per second (kg/s)
HOST_DEVICE inline constexpr float calculate_mass_flow_rate_kg_per_second(
    float density_kg_per_cubic_meter,
    float area_square_meters,
    float velocity_meters_per_second
) {
    return density_kg_per_cubic_meter * area_square_meters * velocity_meters_per_second;
}

/// Calculate volume flow rate
/// Formula: Q = Av
/// @param area_square_meters Cross-sectional area (m²)
/// @param velocity_meters_per_second Flow velocity (m/s)
/// @return Volume flow rate in cubic meters per second (m³/s)
HOST_DEVICE inline constexpr float calculate_volume_flow_rate_cubic_meters_per_second(
    float area_square_meters,
    float velocity_meters_per_second
) {
    return area_square_meters * velocity_meters_per_second;
}

/// Calculate velocity from flow rate
/// Formula: v = Q/A
/// @param flow_rate_cubic_meters_per_second Volume flow rate (m³/s)
/// @param area_square_meters Cross-sectional area (m²)
/// @return Velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_velocity_from_flow_rate_meters_per_second(
    float flow_rate_cubic_meters_per_second,
    float area_square_meters
) {
    return (area_square_meters > EPSILON) 
        ? (flow_rate_cubic_meters_per_second / area_square_meters) 
        : 0.0f;
}

// ============================================================
// Bernoulli's Equation
// ============================================================

/// Calculate pressure from Bernoulli's equation
/// Formula: P₂ = P₁ + ½ρ(v₁² - v₂²) + ρg(h₁ - h₂)
/// @param pressure1_pascals Initial pressure (Pa)
/// @param velocity1_meters_per_second Initial velocity (m/s)
/// @param velocity2_meters_per_second Final velocity (m/s)
/// @param height1_meters Initial height (m)
/// @param height2_meters Final height (m)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Final pressure in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_pressure_from_bernoulli_pascals(
    float pressure1_pascals,
    float velocity1_meters_per_second,
    float velocity2_meters_per_second,
    float height1_meters,
    float height2_meters,
    float density_kg_per_cubic_meter,
    float gravity_meters_per_second_squared = 9.81f
) {
    return pressure1_pascals 
           + 0.5f * density_kg_per_cubic_meter 
           * (velocity1_meters_per_second * velocity1_meters_per_second - velocity2_meters_per_second * velocity2_meters_per_second) 
           + density_kg_per_cubic_meter * gravity_meters_per_second_squared * (height1_meters - height2_meters);
}

/// Calculate velocity from Bernoulli's equation (assuming constant height)
/// Formula: v₂ = √(v₁² + 2(P₁-P₂)/ρ)
/// @param velocity1_meters_per_second Initial velocity (m/s)
/// @param pressure1_pascals Initial pressure (Pa)
/// @param pressure2_pascals Final pressure (Pa)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @return Final velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_velocity_from_bernoulli_meters_per_second(
    float velocity1_meters_per_second,
    float pressure1_pascals,
    float pressure2_pascals,
    float density_kg_per_cubic_meter
) {
    float v2_squared = velocity1_meters_per_second * velocity1_meters_per_second 
                       + 2.0f * (pressure1_pascals - pressure2_pascals) / density_kg_per_cubic_meter;
    return (v2_squared >= 0.0f) ? std::sqrt(v2_squared) : 0.0f;
}

/// Calculate efflux velocity (Torricelli's theorem)
/// Formula: v = √(2gh)
/// @param height_meters Fluid height above hole (m)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Efflux velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_efflux_velocity_meters_per_second(
    float height_meters,
    float gravity_meters_per_second_squared = 9.81f
) {
    return std::sqrt(2.0f * gravity_meters_per_second_squared * height_meters);
}

// ============================================================
// Pressure
// ============================================================

/// Calculate hydrostatic pressure
/// Formula: P = P₀ + ρgh
/// @param surface_pressure_pascals Surface pressure (Pa)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param depth_meters Depth below surface (m)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Pressure at depth in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_hydrostatic_pressure_pascals(
    float surface_pressure_pascals,
    float density_kg_per_cubic_meter,
    float depth_meters,
    float gravity_meters_per_second_squared = 9.81f
) {
    return surface_pressure_pascals + density_kg_per_cubic_meter * gravity_meters_per_second_squared * depth_meters;
}

/// Calculate depth from pressure
/// Formula: h = (P - P₀)/(ρg)
/// @param pressure_pascals Pressure at depth (Pa)
/// @param surface_pressure_pascals Surface pressure (Pa)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Depth in meters (m)
HOST_DEVICE inline constexpr float calculate_depth_from_pressure_meters(
    float pressure_pascals,
    float surface_pressure_pascals,
    float density_kg_per_cubic_meter,
    float gravity_meters_per_second_squared = 9.81f
) {
    float denominator = density_kg_per_cubic_meter * gravity_meters_per_second_squared;
    return (constexpr_abs(denominator) > EPSILON) 
        ? ((pressure_pascals - surface_pressure_pascals) / denominator) 
        : 0.0f;
}

/// Calculate gauge pressure
/// Formula: P_gauge = P - P_atm
/// @param absolute_pressure_pascals Absolute pressure (Pa)
/// @param atmospheric_pressure_pascals Atmospheric pressure (Pa)
/// @return Gauge pressure in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_gauge_pressure_pascals(
    float absolute_pressure_pascals,
    float atmospheric_pressure_pascals = ATMOSPHERIC_PRESSURE_PASCALS
) {
    return absolute_pressure_pascals - atmospheric_pressure_pascals;
}

/// Calculate absolute pressure
/// Formula: P_abs = P_gauge + P_atm
/// @param gauge_pressure_pascals Gauge pressure (Pa)
/// @param atmospheric_pressure_pascals Atmospheric pressure (Pa)
/// @return Absolute pressure in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_absolute_pressure_pascals(
    float gauge_pressure_pascals,
    float atmospheric_pressure_pascals = ATMOSPHERIC_PRESSURE_PASCALS
) {
    return gauge_pressure_pascals + atmospheric_pressure_pascals;
}

// ============================================================
// Buoyancy (Archimedes' Principle)
// ============================================================

/// Calculate buoyant force
/// Formula: F_b = ρ_fluid × V_displaced × g
/// @param fluid_density_kg_per_cubic_meter Density of fluid (kg/m³)
/// @param volume_displaced_cubic_meters Volume of fluid displaced (m³)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Buoyant force in Newtons (N)
HOST_DEVICE inline constexpr float calculate_buoyant_force_newtons(
    float fluid_density_kg_per_cubic_meter,
    float volume_displaced_cubic_meters,
    float gravity_meters_per_second_squared = 9.81f
) {
    return fluid_density_kg_per_cubic_meter * volume_displaced_cubic_meters * gravity_meters_per_second_squared;
}

/// Calculate displaced volume from buoyant force
/// Formula: V = F_b/(ρg)
/// @param buoyant_force_newtons Buoyant force (N)
/// @param fluid_density_kg_per_cubic_meter Density of fluid (kg/m³)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Displaced volume in cubic meters (m³)
HOST_DEVICE inline constexpr float calculate_volume_from_buoyant_force_cubic_meters(
    float buoyant_force_newtons,
    float fluid_density_kg_per_cubic_meter,
    float gravity_meters_per_second_squared = 9.81f
) {
    float denominator = fluid_density_kg_per_cubic_meter * gravity_meters_per_second_squared;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (buoyant_force_newtons / denominator) 
        : 0.0f;
}

/// Calculate apparent weight in fluid
/// Formula: W_apparent = W_actual - F_buoyant
/// @param actual_weight_newtons Weight in vacuum/air (N)
/// @param buoyant_force_newtons Buoyant force (N)
/// @return Apparent weight in Newtons (N)
HOST_DEVICE inline constexpr float calculate_apparent_weight_newtons(
    float actual_weight_newtons,
    float buoyant_force_newtons
) {
    return actual_weight_newtons - buoyant_force_newtons;
}

/// Calculate fraction submerged for floating object
/// Formula: f = ρ_object/ρ_fluid
/// @param object_density_kg_per_cubic_meter Density of object (kg/m³)
/// @param fluid_density_kg_per_cubic_meter Density of fluid (kg/m³)
/// @return Fraction of volume submerged (0-1)
HOST_DEVICE inline constexpr float calculate_fraction_submerged(
    float object_density_kg_per_cubic_meter,
    float fluid_density_kg_per_cubic_meter
) {
    return (fluid_density_kg_per_cubic_meter > EPSILON) 
        ? (object_density_kg_per_cubic_meter / fluid_density_kg_per_cubic_meter) 
        : 0.0f;
}

// ============================================================
// Viscosity & Drag
// ============================================================

/// Calculate drag force using Stokes' law (for small spheres in laminar flow)
/// Formula: F_d = 6πηrv
/// @param viscosity_pascal_seconds Dynamic viscosity (Pa·s)
/// @param radius_meters Sphere radius (m)
/// @param velocity_meters_per_second Velocity (m/s)
/// @return Drag force in Newtons (N)
HOST_DEVICE inline constexpr float calculate_stokes_drag_newtons(
    float viscosity_pascal_seconds,
    float radius_meters,
    float velocity_meters_per_second
) {
    return 6.0f * PI * viscosity_pascal_seconds * radius_meters * velocity_meters_per_second;
}

/// Calculate terminal velocity (Stokes regime)
/// Formula: v_t = 2r²g(ρ_obj - ρ_fluid)/(9η)
/// @param radius_meters Sphere radius (m)
/// @param object_density_kg_per_cubic_meter Object density (kg/m³)
/// @param fluid_density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param viscosity_pascal_seconds Dynamic viscosity (Pa·s)
/// @param gravity_meters_per_second_squared Gravity acceleration (m/s²)
/// @return Terminal velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_terminal_velocity_stokes_meters_per_second(
    float radius_meters,
    float object_density_kg_per_cubic_meter,
    float fluid_density_kg_per_cubic_meter,
    float viscosity_pascal_seconds,
    float gravity_meters_per_second_squared = 9.81f
) {
    float numerator = 2.0f * radius_meters * radius_meters * gravity_meters_per_second_squared * 
                      (object_density_kg_per_cubic_meter - fluid_density_kg_per_cubic_meter);
    float denominator = 9.0f * viscosity_pascal_seconds;
    return (constexpr_abs(denominator) > EPSILON) 
        ? (numerator / denominator) 
        : 0.0f;
}

/// Calculate kinematic viscosity from dynamic viscosity
/// Formula: ν = η/ρ
/// @param dynamic_viscosity_pascal_seconds Dynamic viscosity (Pa·s)
/// @param density_kg_per_cubic_meter Density (kg/m³)
/// @return Kinematic viscosity in square meters per second (m²/s)
HOST_DEVICE inline constexpr float calculate_kinematic_viscosity_square_meters_per_second(
    float dynamic_viscosity_pascal_seconds,
    float density_kg_per_cubic_meter
) {
    return (density_kg_per_cubic_meter > EPSILON) 
        ? (dynamic_viscosity_pascal_seconds / density_kg_per_cubic_meter) 
        : 0.0f;
}

// ============================================================
// Reynolds Number
// ============================================================

/// Calculate Reynolds number using dynamic viscosity
/// Formula: Re = ρvL/η
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param velocity_meters_per_second Flow velocity (m/s)
/// @param characteristic_length_meters Characteristic length (m)
/// @param viscosity_pascal_seconds Dynamic viscosity (Pa·s)
/// @return Reynolds number (dimensionless)
HOST_DEVICE inline constexpr float calculate_reynolds_number(
    float density_kg_per_cubic_meter,
    float velocity_meters_per_second,
    float characteristic_length_meters,
    float viscosity_pascal_seconds
) {
    return (viscosity_pascal_seconds > EPSILON) 
        ? (density_kg_per_cubic_meter * velocity_meters_per_second * characteristic_length_meters / viscosity_pascal_seconds) 
        : 0.0f;
}

/// Calculate Reynolds number using kinematic viscosity
/// Formula: Re = vL/ν
/// @param velocity_meters_per_second Flow velocity (m/s)
/// @param characteristic_length_meters Characteristic length (m)
/// @param kinematic_viscosity_square_meters_per_second Kinematic viscosity (m²/s)
/// @return Reynolds number (dimensionless)
HOST_DEVICE inline constexpr float calculate_reynolds_number_kinematic(
    float velocity_meters_per_second,
    float characteristic_length_meters,
    float kinematic_viscosity_square_meters_per_second
) {
    return (kinematic_viscosity_square_meters_per_second > EPSILON) 
        ? (velocity_meters_per_second * characteristic_length_meters / kinematic_viscosity_square_meters_per_second) 
        : 0.0f;
}

/// Flow regime classification
enum class FlowRegime {
    LAMINAR,      // Re < 2300 (pipe flow)
    TRANSITIONAL, // 2300 < Re < 4000
    TURBULENT     // Re > 4000
};

/// Determine flow regime based on Reynolds number
/// @param reynolds_number Reynolds number
/// @return FlowRegime enum value
HOST_DEVICE inline constexpr FlowRegime determine_flow_regime(float reynolds_number) {
    if (reynolds_number < 2300.0f) return FlowRegime::LAMINAR;
    if (reynolds_number < 4000.0f) return FlowRegime::TRANSITIONAL;
    return FlowRegime::TURBULENT;
}

// ============================================================
// Poiseuille's Law (Pipe Flow)
// ============================================================

/// Calculate flow rate using Poiseuille's law
/// Formula: Q = πΔPr⁴/(8ηL)
/// @param pressure_drop_pascals Pressure difference (Pa)
/// @param radius_meters Pipe radius (m)
/// @param length_meters Pipe length (m)
/// @param viscosity_pascal_seconds Fluid viscosity (Pa·s)
/// @return Volume flow rate in cubic meters per second (m³/s)
HOST_DEVICE inline constexpr float calculate_poiseuille_flow_rate_cubic_meters_per_second(
    float pressure_drop_pascals,
    float radius_meters,
    float length_meters,
    float viscosity_pascal_seconds
) {
    float numerator = PI * pressure_drop_pascals * 
                      radius_meters * radius_meters * radius_meters * radius_meters;
    float denominator = 8.0f * viscosity_pascal_seconds * length_meters;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (numerator / denominator) : 0.0f;
}

/// Calculate average velocity in pipe (Poiseuille flow)
/// Formula: v_avg = ΔPr²/(8ηL)
/// @param pressure_drop_pascals Pressure difference (Pa)
/// @param radius_meters Pipe radius (m)
/// @param length_meters Pipe length (m)
/// @param viscosity_pascal_seconds Fluid viscosity (Pa·s)
/// @return Average velocity in meters per second (m/s)
HOST_DEVICE inline constexpr float calculate_poiseuille_average_velocity_meters_per_second(
    float pressure_drop_pascals,
    float radius_meters,
    float length_meters,
    float viscosity_pascal_seconds
) {
    float numerator = pressure_drop_pascals * radius_meters * radius_meters;
    float denominator = 8.0f * viscosity_pascal_seconds * length_meters;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (numerator / denominator) : 0.0f;
}

/// Calculate pressure drop for given flow rate
/// Formula: ΔP = 8ηLQ/(πr⁴)
/// @param viscosity_pascal_seconds Fluid viscosity (Pa·s)
/// @param length_meters Pipe length (m)
/// @param flow_rate_cubic_meters_per_second Flow rate (m³/s)
/// @param radius_meters Pipe radius (m)
/// @return Pressure drop in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_pressure_drop_from_flow_pascals(
    float viscosity_pascal_seconds,
    float length_meters,
    float flow_rate_cubic_meters_per_second,
    float radius_meters
) {
    float numerator = 8.0f * viscosity_pascal_seconds * length_meters * flow_rate_cubic_meters_per_second;
    float denominator = PI * radius_meters * radius_meters * radius_meters * radius_meters;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (numerator / denominator) : 0.0f;
}

// ============================================================
// Surface Tension
// ============================================================

/// Calculate surface tension force
/// Formula: F = γL
/// @param surface_tension_newtons_per_meter Surface tension (N/m)
/// @param length_meters Length of contact line (m)
/// @return Force in Newtons (N)
HOST_DEVICE inline constexpr float calculate_surface_tension_force_newtons(
    float surface_tension_newtons_per_meter,
    float length_meters
) {
    return surface_tension_newtons_per_meter * length_meters;
}

/// Calculate pressure difference across curved surface (Laplace pressure for sphere/droplet)
/// Formula: ΔP = 2γ/r
/// @param surface_tension_newtons_per_meter Surface tension (N/m)
/// @param radius_meters Radius of curvature (m)
/// @return Pressure difference in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_laplace_pressure_sphere_pascals(
    float surface_tension_newtons_per_meter,
    float radius_meters
) {
    return (radius_meters > EPSILON) ? (2.0f * surface_tension_newtons_per_meter / radius_meters) : 0.0f;
}

/// Calculate pressure difference in bubble (two surfaces)
/// Formula: ΔP = 4γ/r
/// @param surface_tension_newtons_per_meter Surface tension (N/m)
/// @param radius_meters Bubble radius (m)
/// @return Pressure difference in Pascals (Pa)
HOST_DEVICE inline constexpr float calculate_bubble_pressure_pascals(
    float surface_tension_newtons_per_meter,
    float radius_meters
) {
    return (radius_meters > EPSILON) ? (4.0f * surface_tension_newtons_per_meter / radius_meters) : 0.0f;
}

/// Calculate capillary rise height
/// Formula: h = 2γcosθ/(ρgr)
/// @param surface_tension_newtons_per_meter Surface tension (N/m)
/// @param contact_angle_radians Contact angle (rad)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param radius_meters Capillary radius (m)
/// @param gravity_meters_per_second_squared Gravity (m/s²)
/// @return Capillary rise height in meters (m)
HOST_DEVICE inline float calculate_capillary_rise_meters(
    float surface_tension_newtons_per_meter,
    float contact_angle_radians,
    float density_kg_per_cubic_meter,
    float radius_meters,
    float gravity_meters_per_second_squared = 9.81f
) {
    float numerator = 2.0f * surface_tension_newtons_per_meter * std::cos(contact_angle_radians);
    float denominator = density_kg_per_cubic_meter * gravity_meters_per_second_squared * radius_meters;
    return (std::abs(denominator) > EPSILON) ? 
           (numerator / denominator) : 0.0f;
}

// ============================================================
// Drag Coefficient
// ============================================================

/// Calculate drag force
/// Formula: F_d = ½ρv²C_dA
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param drag_coefficient Drag coefficient (dimensionless)
/// @param area_square_meters Reference area (m²)
/// @return Drag force in Newtons (N)
HOST_DEVICE inline constexpr float calculate_drag_force_newtons(
    float density_kg_per_cubic_meter,
    float velocity_meters_per_second,
    float drag_coefficient,
    float area_square_meters
) {
    return 0.5f * density_kg_per_cubic_meter * velocity_meters_per_second * velocity_meters_per_second * drag_coefficient * area_square_meters;
}

/// Calculate drag coefficient from measured force
/// Formula: C_d = 2F_d/(ρv²A)
/// @param drag_force_newtons Measured drag force (N)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param velocity_meters_per_second Velocity (m/s)
/// @param area_square_meters Reference area (m²)
/// @return Drag coefficient (dimensionless)
HOST_DEVICE inline constexpr float calculate_drag_coefficient(
    float drag_force_newtons,
    float density_kg_per_cubic_meter,
    float velocity_meters_per_second,
    float area_square_meters
) {
    float denominator = 0.5f * density_kg_per_cubic_meter * velocity_meters_per_second * velocity_meters_per_second * area_square_meters;
    return (constexpr_abs(denominator) > EPSILON) ? 
           (drag_force_newtons / denominator) : 0.0f;
}

/// Calculate terminal velocity (drag regime)
/// Formula: v_t = √(2mg/(ρC_dA))
/// @param mass_kilograms Object mass (kg)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @param drag_coefficient Drag coefficient
/// @param area_square_meters Reference area (m²)
/// @param gravity_meters_per_second_squared Gravity (m/s²)
/// @return Terminal velocity in meters per second (m/s)
HOST_DEVICE inline float calculate_terminal_velocity_drag_meters_per_second(
    float mass_kilograms,
    float density_kg_per_cubic_meter,
    float drag_coefficient,
    float area_square_meters,
    float gravity_meters_per_second_squared = 9.81f
) {
    float numerator = 2.0f * mass_kilograms * gravity_meters_per_second_squared;
    float denominator = density_kg_per_cubic_meter * drag_coefficient * area_square_meters;
    return (std::abs(denominator) > EPSILON) ? 
           std::sqrt(numerator / denominator) : 0.0f;
}

// ============================================================
// Vorticity & Circulation
// ============================================================

/// Calculate 2D vorticity magnitude
/// Formula: ω = ∂v/∂x - ∂u/∂y
/// @param dv_dx Gradient of y-velocity in x-direction
/// @param du_dy Gradient of x-velocity in y-direction
/// @return Vorticity in radians per second (rad/s) or inverse seconds (1/s)
HOST_DEVICE inline constexpr float calculate_vorticity_2d_per_second(
    float dv_dx, 
    float du_dy
) {
    return dv_dx - du_dy;
}

/// Calculate circulation along closed path (approximation)
/// Formula: Γ = ∮v·dl ≈ vL
/// @param velocity_meters_per_second Velocity along path (m/s)
/// @param length_meters Path length (m)
/// @return Circulation in meters squared per second (m²/s)
HOST_DEVICE inline constexpr float calculate_circulation_square_meters_per_second(
    float velocity_meters_per_second,
    float length_meters
) {
    return velocity_meters_per_second * length_meters;
}

// ============================================================
// Compressibility
// ============================================================

/// Calculate speed of sound in fluid
/// Formula: c = √(K/ρ)
/// @param bulk_modulus_pascals Bulk modulus of elasticity (Pa)
/// @param density_kg_per_cubic_meter Fluid density (kg/m³)
/// @return Speed of sound in meters per second (m/s)
HOST_DEVICE inline float calculate_speed_of_sound_fluid_meters_per_second(
    float bulk_modulus_pascals,
    float density_kg_per_cubic_meter
) {
    return (density_kg_per_cubic_meter > EPSILON) ? 
           std::sqrt(bulk_modulus_pascals / density_kg_per_cubic_meter) : 0.0f;
}

/// Calculate Mach number
/// Formula: M = v/c
/// @param velocity_meters_per_second Flow velocity (m/s)
/// @param sound_speed_meters_per_second Local speed of sound (m/s)
/// @return Mach number (dimensionless)
HOST_DEVICE inline constexpr float calculate_mach_number(
    float velocity_meters_per_second,
    float sound_speed_meters_per_second
) {
    return (sound_speed_meters_per_second > EPSILON) ? 
           (velocity_meters_per_second / sound_speed_meters_per_second) : 0.0f;
}

/// Check compressibility regime
enum class CompressibilityRegime {
    INCOMPRESSIBLE,  // M < 0.3
    SUBSONIC,        // 0.3 < M < 0.8
    TRANSONIC,       // 0.8 < M < 1.2
    SUPERSONIC,      // 1.2 < M < 5
    HYPERSONIC       // M > 5
};

/// Determine compressibility regime
/// @param mach_number Mach number
/// @return CompressibilityRegime enum value
HOST_DEVICE inline constexpr CompressibilityRegime determine_compressibility_regime(float mach_number) {
    if (mach_number < 0.3f) return CompressibilityRegime::INCOMPRESSIBLE;
    if (mach_number < 0.8f) return CompressibilityRegime::SUBSONIC;
    if (mach_number < 1.2f) return CompressibilityRegime::TRANSONIC;
    if (mach_number < 5.0f) return CompressibilityRegime::SUPERSONIC;
    return CompressibilityRegime::HYPERSONIC;
}

} // namespace fluid_dynamics
} // namespace formulas
} // namespace physics
} // namespace basements

#endif // BASEMENTS_PHYSICS_FORMULAS_FLUID_DYNAMICS_H
