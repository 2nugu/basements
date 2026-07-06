#include <basements/core/math/basements.h>
#include <basements/dynamics/forces.h>
#include <iostream>
#include <iomanip>

using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;

void print_body_state(const RigidBody& body, const std::string& label) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << label << ":\n";
    std::cout << "  Position: (" << body.position.x << ", " << body.position.y << ", " << body.position.z << ")\n";
    std::cout << "  Velocity: (" << body.linear_velocity.x << ", " << body.linear_velocity.y << ", " << body.linear_velocity.z << ")\n";
    std::cout << "  Force: (" << body.force.x << ", " << body.force.y << ", " << body.force.z << ")\n\n";
}

int main() {
    std::cout << "=== Force Generator System Demo ===\n\n";
    
    // ========================================================================
    // Demo 1: Uniform Gravity on Different Celestial Bodies
    // ========================================================================
    std::cout << "--- Demo 1: Falling Ball on Different Planets ---\n";
    
    RigidBody ball;
    ball.position = Vec3(0, 100, 0);
    ball.set_mass(1.0f);
    
    // Test on Earth
    {
        std::cout << "\n[Earth Gravity: 9.81 m/s²]\n";
        auto gravity = UniformGravity::Earth();
        ball.force = Vec3(0, 0, 0);
        gravity.apply(&ball, 1, 0.016f);
        print_body_state(ball, "After applying Earth gravity");
    }
    
    // Test on Moon
    {
        std::cout << "[Moon Gravity: 1.62 m/s²]\n";
        auto gravity = UniformGravity::Moon();
        ball.force = Vec3(0, 0, 0);
        gravity.apply(&ball, 1, 0.016f);
        print_body_state(ball, "After applying Moon gravity");
    }
    
    // Test on Jupiter
    {
        std::cout << "[Jupiter Gravity: 24.79 m/s²]\n";
        auto gravity = UniformGravity::Jupiter();
        ball.force = Vec3(0, 0, 0);
        gravity.apply(&ball, 1, 0.016f);
        print_body_state(ball, "After applying Jupiter gravity");
    }
    
    // Custom gravity
    {
        std::cout << "[Custom Gravity: 50 m/s² downward]\n";
        auto gravity = UniformGravity::Custom(Vec3(0, -50.0f, 0));
        ball.force = Vec3(0, 0, 0);
        gravity.apply(&ball, 1, 0.016f);
        print_body_state(ball, "After applying custom gravity");
    }
    
    // ========================================================================
    // Demo 2: Runtime Parameter Adjustment
    // ========================================================================
    std::cout << "\n--- Demo 2: Runtime Parameter Adjustment ---\n";
    
    auto adjustable_gravity = UniformGravity::Earth();
    std::cout << "Initial gravity magnitude: " << adjustable_gravity.get_parameter("magnitude") << " m/s²\n";
    
    // Adjust to Mars gravity
    adjustable_gravity.set_parameter("magnitude", 3.71f);
    std::cout << "After adjustment: " << adjustable_gravity.get_parameter("magnitude") << " m/s²\n";
    
    ball.force = Vec3(0, 0, 0);
    adjustable_gravity.apply(&ball, 1, 0.016f);
    print_body_state(ball, "Ball with adjusted gravity");
    
    // ========================================================================
    // Demo 3: Aerodynamic Drag in Different Fluids
    // ========================================================================
    std::cout << "\n--- Demo 3: Drag Force in Air vs Water ---\n";
    
    RigidBody falling_sphere;
    falling_sphere.position = Vec3(0, 100, 0);
    falling_sphere.linear_velocity = Vec3(0, -10, 0);  // 10 m/s downward
    falling_sphere.set_mass(1.0f);
    
    // Air drag
    {
        std::cout << "\n[Air Drag - Sphere radius 0.5m]\n";
        auto drag = AerodynamicDrag::AirSphere(0.5f);
        std::cout << "Fluid density: " << drag.get_parameter("density") << " kg/m³\n";
        std::cout << "Drag coefficient: " << drag.get_parameter("drag_coefficient") << "\n";
        
        falling_sphere.force = Vec3(0, 0, 0);
        drag.apply(&falling_sphere, 1, 0.016f);
        print_body_state(falling_sphere, "Sphere falling in air");
    }
    
    // Water drag
    {
        std::cout << "[Water Drag - Same sphere]\n";
        auto drag = AerodynamicDrag::WaterSphere(0.5f);
        std::cout << "Fluid density: " << drag.get_parameter("density") << " kg/m³\n";
        
        falling_sphere.force = Vec3(0, 0, 0);
        drag.apply(&falling_sphere, 1, 0.016f);
        print_body_state(falling_sphere, "Sphere falling in water");
    }
    
    // ========================================================================
    // Demo 4: N-Body Gravity (Planetary System)
    // ========================================================================
    std::cout << "\n--- Demo 4: N-Body Gravity (Sun-Earth-Moon) ---\n";
    
    RigidBody bodies[3];
    
    // Sun (center)
    bodies[0].position = Vec3(0, 0, 0);
    bodies[0].set_mass(1.989e30f);  // Solar mass
    bodies[0].type = BodyType::STATIC;
    
    // Earth
    bodies[1].position = Vec3(1.496e11f, 0, 0);  // 1 AU
    bodies[1].set_mass(5.972e24f);  // Earth mass
    
    // Moon
    bodies[2].position = Vec3(1.496e11f + 3.844e8f, 0, 0);  // Earth + lunar distance
    bodies[2].set_mass(7.342e22f);  // Lunar mass
    
    auto nbody = NBodyGravity::RealPhysics();
    std::cout << "Gravitational constant: " << nbody.get_parameter("G") << " m³/kg/s²\n\n";
    
    // Clear forces
    for (int i = 0; i < 3; ++i) bodies[i].force = Vec3(0, 0, 0);
    
    // Apply N-body gravity
    nbody.apply(bodies, 3, 0.016f);
    
    std::cout << "Sun force: (" << bodies[0].force.x << ", " << bodies[0].force.y << ", " << bodies[0].force.z << ") N\n";
    std::cout << "Earth force: (" << bodies[1].force.x << ", " << bodies[1].force.y << ", " << bodies[1].force.z << ") N\n";
    std::cout << "Moon force: (" << bodies[2].force.x << ", " << bodies[2].force.y << ", " << bodies[2].force.z << ") N\n";
    
    // ========================================================================
    // Demo 5: Lennard-Jones (Molecular Dynamics)
    // ========================================================================
    std::cout << "\n--- Demo 5: Lennard-Jones Force (Atoms) ---\n";
    
    RigidBody atoms[2];
    atoms[0].position = Vec3(0, 0, 0);
    atoms[0].set_mass(1.0f);
    atoms[1].position = Vec3(1.5f, 0, 0);  // 1.5 sigma apart
    atoms[1].set_mass(1.0f);
    
    LennardJonesForce lj(1.0f, 1.0f);  // epsilon=1, sigma=1
    
    atoms[0].force = Vec3(0, 0, 0);
    atoms[1].force = Vec3(0, 0, 0);
    lj.apply(atoms, 2, 0.016f);
    
    std::cout << "Atom 0 force: (" << atoms[0].force.x << ", " << atoms[0].force.y << ", " << atoms[0].force.z << ")\n";
    std::cout << "Atom 1 force: (" << atoms[1].force.x << ", " << atoms[1].force.y << ", " << atoms[1].force.z << ")\n";
    std::cout << "(Attractive force pulling atoms together)\n";
    
    std::cout << "\n=== Demo Complete ===\n";
    return 0;
}
