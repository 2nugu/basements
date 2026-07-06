
#include <iostream>
#include <cmath>
#include <limits>

// Define EPSILON if not defined in headers (removed to avoid conflict with common.h)

// Mock types if needed, but we should include the actual types
#include "basements/core/math/vec3.h"
#include "basements/core/types.h"

// Include ALL formula headers to check for compilation errors
#include "basements/physics/formulas/phase_field.h"
#include "basements/physics/formulas/kinematics.h"
#include "basements/physics/formulas/dynamics.h"
#include "basements/physics/formulas/energy.h"
#include "basements/physics/formulas/rotation.h"
#include "basements/physics/formulas/thermodynamics.h"
#include "basements/physics/formulas/electromagnetism.h"
#include "basements/physics/formulas/fluid_dynamics.h"
#include "basements/physics/formulas/waves.h"
#include "basements/physics/formulas/optics.h"
#include "basements/physics/formulas/relativity.h"
#include "basements/physics/formulas/quantum.h"
#include "basements/physics/formulas/geophysics.h"
#include "basements/physics/formulas/atmospheric.h"
#include "basements/physics/formulas/oceanography.h"
#include "basements/physics/formulas/astronomy.h"
#include "basements/physics/formulas/geometry.h"

int main() {
    std::cout << "Starting Formula Compilation Check..." << std::endl;

    // Basic instantiation check to ensure functions are usable
    using namespace basements::physics::formulas;

    // Kinematics
    // Using calculate_velocity_from_constant_acceleration (v = v0 + at)
    float velocity = kinematics::calculate_velocity_from_constant_acceleration(0.0f, 9.8f, 10.0f);
    std::cout << "Kinematics check: " << velocity << std::endl;

    // Dynamics
    float force = dynamics::calculate_force_newtons_second_law(10.0f, 9.8f);
    std::cout << "Dynamics check: " << force << std::endl;

    // Thermodynamics
    float kelvin = thermodynamics::convert_celsius_to_kelvin(25.0f);
    std::cout << "Thermodynamics check: " << kelvin << std::endl;

    // Geophysics
    float g_alt = geophysics::calculate_gravity_at_altitude_meters_per_second_squared(1000.0f);
    std::cout << "Geophysics check: " << g_alt << std::endl;

    // Atmospheric
    float pressure = atmospheric::calculate_pressure_from_density_pascals(1.225f, 288.15f);
    std::cout << "Atmospheric check: " << pressure << std::endl;

    // Oceanography
    float wave_speed = oceanography::calculate_deep_water_wave_speed_meters_per_second(10.0f);
    std::cout << "Oceanography check: " << wave_speed << std::endl;

    // Astronomy
    float orbital_period = astronomy::calculate_orbital_period_seconds(1.496e11f, 1.989e30f);
    std::cout << "Astronomy check: " << orbital_period << std::endl;

    // Geometry
    basements::math::Vec3 p1(0,0,0);
    basements::math::Vec3 p2(1,1,1);
    float distance = geometry::calculate_point_to_plane_distance(p1, p2, basements::math::Vec3(0,1,0));
    std::cout << "Geometry check: " << distance << std::endl;

    // Sound Waves
    float sound_speed = waves::calculate_speed_of_sound_ideal_gas_meters_per_second(1.4f, 8.314f, 298.15f, 0.02896f);
    std::cout << "Sound Waves check: " << sound_speed << std::endl;

    // Phase Field
    float gl_energy = phase_field::calculate_gl_potential_energy_density(0.5f, 1.0f, 1.0f, 200.0f, 250.0f);
    std::cout << "Phase Field check: " << gl_energy << std::endl;

    std::cout << "All formulas compiled successfully!" << std::endl;
    return 0;
}
