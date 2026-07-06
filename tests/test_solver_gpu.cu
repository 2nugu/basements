#include <gtest/gtest.h>
#include "basements/core/math/basements.h"
#include "basements/physics/dynamics/solver_gpu.h"
#include <iostream>

using namespace basements;
using namespace basements::math;
using namespace basements::physics;
using namespace basements::dynamics;

TEST(SolverGPU, SimpleBounce) {
    // 1. Setup Bodies
    const int NUM_BODIES = 2;
    RigidBody h_bodies[NUM_BODIES];

    // Body 0: Static Floor (Infinite Mass)
    h_bodies[0].make_static();
    h_bodies[0].position = Vec3(0, -10.0f, 0); 
    // Although static, give it a shape for concept (Box or Sphere doesn't matter for Solver logic only constraints matter)

    // Body 1: Dynamic Sphere (Mass 1.0)
    h_bodies[1].make_dynamic(1.0f);
    h_bodies[1].position = Vec3(0, 0, 0);
    h_bodies[1].linear_velocity = Vec3(0, -10.0f, 0); // Moving down towards floor
    h_bodies[1].set_inertia_tensor(Matrix3::inertia_sphere(1.0f, 1.0f));

    // 2. Setup Constraint
    const int NUM_CONTACTS = 1;
    ContactConstraint h_contacts[NUM_CONTACTS];
    
    // Simulate a collision detected by Narrowphase
    h_contacts[0].bodyA_index = 0; // Floor
    h_contacts[0].bodyB_index = 1; // Falling Sphere
    
    // Impact point (approximate)
    h_contacts[0].contact_point = Vec3(0, 0, 0); 
    
    // Normal pointing from B to A? Or A to B?
    // Convention in solver.h: rA = pt - posA.
    // Impulse applied to A directly (-impulse), to B (+impulse).
    // If B is falling down (-Y) and hits A (Floor), B needs +Y impulse.
    // So impulse vector must be +Y.
    // If impulse = normal * lambda (lambda > 0), normal should be +Y.
    // Normal usually points FROM A TO B or B TO A. 
    // Let's check solver.h:
    // apply_impulse(bB, impulse, rB) -> bB gets +impulse.
    // So if we want B to bounce UP, impulse must be UP.
    // So Normal must be UP (0, 1, 0).
    h_contacts[0].normal = Vec3(0, 1.0f, 0); 
    
    h_contacts[0].penetration = 0.1f;
    h_contacts[0].restitution = 1.0f; // Perfectly elastic bounce
    h_contacts[0].friction = 0.0f;

    // 3. Allocate Device Memory
    RigidBody* d_bodies;
    ContactConstraint* d_contacts;
    
    cudaError_t err;
    err = cudaMalloc(&d_bodies, NUM_BODIES * sizeof(RigidBody));
    if(err != cudaSuccess) printf("Malloc Bodies Failed: %s\n", cudaGetErrorString(err));
    
    err = cudaMalloc(&d_contacts, NUM_CONTACTS * sizeof(ContactConstraint));
    if(err != cudaSuccess) printf("Malloc Contacts Failed: %s\n", cudaGetErrorString(err));
    
    err = cudaMemcpy(d_bodies, h_bodies, NUM_BODIES * sizeof(RigidBody), cudaMemcpyHostToDevice);
    if(err != cudaSuccess) printf("Memcpy Bodies Failed: %s\n", cudaGetErrorString(err));
    
    err = cudaMemcpy(d_contacts, h_contacts, NUM_CONTACTS * sizeof(ContactConstraint), cudaMemcpyHostToDevice);
    if(err != cudaSuccess) printf("Memcpy Contacts Failed: %s\n", cudaGetErrorString(err));

    // 4. Run Solver
    SolverGPU solver;
    float dt = 0.016f;
    
    // We expect the solver to:
    // 1. Detect negative relative velocity (approaching)
    // 2. Restitution causes target velocity to be positive (bouncing)
    // 3. Apply impulse to flip velocity of Body 1 from -10 to +10 (approx)
    
    solver.solve(d_contacts, NUM_CONTACTS, d_bodies, NUM_BODIES, dt, 10);

    // 5. Verify Results
    cudaMemcpy(h_bodies, d_bodies, NUM_BODIES * sizeof(RigidBody), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_contacts, d_contacts, NUM_CONTACTS * sizeof(ContactConstraint), cudaMemcpyDeviceToHost);

    std::cout << "Body 1 Velocity Y after solve: " << h_bodies[1].linear_velocity.y << std::endl;
    std::cout << "Accumulated Impulse: " << h_contacts[0].accumulated_impulse_normal << std::endl;

    // Verification
    // Elastic collision with infinite mass floor: v_final = -e * v_initial
    // v_initial = -10. v_final should be +10.
    // Gravity is not applied in the solver, only constraints. 
    // (Gravity is usually applied before solve in integration, but here we just test resolution)
    
    EXPECT_GT(h_bodies[1].linear_velocity.y, 0.0f); // Should be moving up
    EXPECT_NEAR(h_bodies[1].linear_velocity.y, 10.0f, 1.0f); // Allow some error/bias

    cudaFree(d_bodies);
    cudaFree(d_contacts);
}
