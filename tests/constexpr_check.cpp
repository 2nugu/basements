#include "basements/core/math/common.h"
#include "basements/physics/formulas/dynamics.h"
#include "basements/physics/formulas/energy.h"
#include "basements/physics/formulas/kinematics.h"
#include "basements/physics/formulas/rotation.h"
#include "basements/physics/formulas/thermodynamics.h"
#include "basements/physics/formulas/waves.h"
#include "basements/physics/formulas/optics.h"
#include "basements/physics/formulas/fluid_dynamics.h"
#include "basements/physics/formulas/quantum.h"
#include "basements/physics/formulas/relativity.h"
#include "basements/physics/formulas/electromagnetism.h"
#include "basements/physics/formulas/astronomy.h"
#include "basements/physics/formulas/atmospheric.h"

// Helper for float comparison in static_assert
constexpr bool float_equal(float a, float b) {
    return basements::math::approx_equal(a, b);
}

void verification_test() {
    using namespace basements::physics::formulas;

    // dynamics.h
    static_assert(float_equal(dynamics::calculate_force_newtons(2.0f, 3.0f), 6.0f), "Dynamics: F=ma failed");
    static_assert(float_equal(dynamics::calculate_force_friction_newtons(0.5f, 10.0f), 5.0f), "Dynamics: Friction failed");

    // energy.h
    static_assert(float_equal(energy::calculate_kinetic_energy_joules(2.0f, 3.0f), 9.0f), "Energy: KE failed"); // 0.5 * 2 * 3^2 = 9
    static_assert(float_equal(energy::calculate_work_joules(10.0f, 5.0f), 50.0f), "Energy: Work failed");

    // kinematics.h
    static_assert(float_equal(kinematics::calculate_displacement_meters(10.0f, 2.0f, 0.5f, 5.0f), 30.0f), "Kinematics: Displacement failed"); // 10*2 + 0.5*5*4 = 20 + 10 = 30

    // rotation.h
    static_assert(float_equal(rotation::calculate_torque_newton_meters(2.0f, 5.0f), 10.0f), "Rotation: Torque failed");

    // thermodynamics.h
    static_assert(float_equal(thermodynamics::convert_celsius_to_kelvin(0.0f), 273.15f), "Thermodynamics: C to K failed");

    // waves.h
    static_assert(float_equal(waves::calculate_wave_speed_meters_per_second(50.0f, 2.0f), 100.0f), "Waves: Speed failed");

    // optics.h
    static_assert(float_equal(optics::calculate_magnification(10.0f, 5.0f), -2.0f), "Optics: Magnification failed");

    // fluid_dynamics.h
    static_assert(float_equal(fluid_dynamics::calculate_pressure_from_bernoulli_pascals(100.0f, 10.0f, 0.0f, 0.0f, 0.0f, 1.0f), 150.0f), "Fluid Dynamics: Bernoulli failed"); // 100 + 0.5*1*(100-0) = 150


    // quantum.h
    // E = hf. h approx 6.626e-34.
    constexpr float h = quantum::PLANCK_CONSTANT_JOULE_SECONDS;
    static_assert(float_equal(quantum::calculate_photon_energy_joules(1.0e14f), h * 1.0e14f), "Quantum: Photon energy failed");

    // relativity.h
    // E = mc^2.
    constexpr float c = relativity::SPEED_OF_LIGHT_METERS_PER_SECOND;
    static_assert(float_equal(relativity::calculate_rest_energy_joules(1.0f), c * c), "Relativity: E=mc^2 failed");

    // electromagnetism.h
    constexpr float k = electromagnetism::COULOMB_CONSTANT_NEWTON_METER_SQUARED_PER_COULOMB_SQUARED;
    static_assert(float_equal(electromagnetism::calculate_coulomb_force_newtons(1e-6f, 1e-6f, 1.0f), k * 1e-12f), "Electromagnetism: Coulomb failed");

    // astronomy.h
    static_assert(float_equal(astronomy::calculate_stellar_luminosity_watts(1.0f, 100.0f), 4.0f * PI * 1.0f * astronomy::STEFAN_BOLTZMANN_CONSTANT * 100000000.0f), "Astronomy: Luminosity failed");

    // atmospheric.h
    static_assert(float_equal(atmospheric::calculate_pressure_from_density_pascals(1.2f, 300.0f), 1.2f * atmospheric::GAS_CONSTANT_DRY_AIR * 300.0f), "Atmospheric: Ideal Gas failed");
}

int main() {
    verification_test();
    return 0;
}
