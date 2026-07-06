#include <basements/physics/rigid_body.h>
#include <basements/physics/dynamics/forces.h>
#include <iostream>

using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;

int main() {
    std::cout << "=== Force Generator Test ===\n\n";
    
    // Test 1: Earth vs Moon gravity
    std::cout << "Test 1: Gravity on different planets\n";
    
    RigidBody ball;
    ball.position = Vec3(0, 100, 0);
    ball.set_mass(1.0f);
    
    auto earth_gravity = UniformGravity::Earth();
    auto moon_gravity = UniformGravity::Moon();
    
    ball.force = Vec3(0, 0, 0);
    earth_gravity.apply(&ball, 1, 0.016f);
    std::cout << "Earth force: " << ball.force.y << " N (expected: -9.81)\n";
    
    ball.force = Vec3(0, 0, 0);
    moon_gravity.apply(&ball, 1, 0.016f);
    std::cout << "Moon force: " << ball.force.y << " N (expected: -1.62)\n";
    
    // Test 2: Runtime parameter adjustment (direct member access)
    std::cout << "\nTest 2: Runtime parameter adjustment\n";
    auto adjustable = UniformGravity::Earth();
    std::cout << "Initial gravity: " << adjustable.g.length() << " m/s²\n";
    
    // Adjust to Mars gravity
    adjustable.g = Vec3(0, -3.71f, 0);
    std::cout << "After adjustment: " << adjustable.g.length() << " m/s²\n";
    
    ball.force = Vec3(0, 0, 0);
    adjustable.apply(&ball, 1, 0.016f);
    std::cout << "Mars force: " << ball.force.y << " N (expected: -3.71)\n";
    
    // Test 3: Drag force
    std::cout << "\nTest 3: Aerodynamic drag\n";
    RigidBody sphere;
    sphere.position = Vec3(0, 100, 0);
    sphere.linear_velocity = Vec3(0, -10, 0);
    sphere.set_mass(1.0f);
    
    auto air_drag = AerodynamicDrag::AirSphere(0.5f);
    sphere.force = Vec3(0, 0, 0);
    air_drag.apply(&sphere, 1, 0.016f);
    std::cout << "Air drag force: " << sphere.force.y << " N (upward, opposing motion)\n";
    
    // Test 4: Runtime drag adjustment
    std::cout << "\nTest 4: Adjusting drag parameters\n";
    air_drag.rho = 1000.0f;  // Change to water density
    sphere.force = Vec3(0, 0, 0);
    air_drag.apply(&sphere, 1, 0.016f);
    std::cout << "Water drag force: " << sphere.force.y << " N (much stronger)\n";
    
    std::cout << "\n=== All Tests Complete ===\n";
    return 0;
}
