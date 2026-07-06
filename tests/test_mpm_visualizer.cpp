// #include <glad/glad.h>
#include <GLFW/glfw3.h> 

#include "basements/physics/mpm/mpm_solver.h"
#include "basements/physics/mpm/particle.h"
#include "basements/core/math/vec3.h"
#include <iostream>
#include <vector>
#include <cmath>

#pragma comment(lib, "opengl32.lib")

// Explicitly use namespaces to avoid ambiguity
// using namespace basements::mpm;
// using namespace basements::math;

basements::mpm::MPMSolver solver;
bool running = false;
int window_width = 800;
int window_height = 800;

void init_mpm() {
    solver = basements::mpm::MPMSolver();
    solver.initialize(0.05f); // 5cm grid
    
    // Create Sand Column
    for (float x = 0.4f; x <= 0.6f; x += 0.025f) {
        for (float y = 0.2f; y <= 0.8f; y += 0.025f) {
            for (float z = 0.4f; z <= 0.6f; z += 0.025f) {
                // Add jitter
                float jx = (rand() % 100 / 100.0f - 0.5f) * 0.01f;
                float jz = (rand() % 100 / 100.0f - 0.5f) * 0.01f;
                // Use explicit namespace for Vec3
                solver.add_particle(
                    basements::math::Vec3(x + jx, y, z + jz), 
                    basements::math::Vec3(0,0,0)
                );
            }
        }
    }
    std::cout << "Initialized " << solver.get_particles().size() << " particles." << std::endl;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_SPACE) running = !running;
        if (key == GLFW_KEY_R) {
            running = false;
            init_mpm();
        }
        if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, true);
    }
}

int main() {
    if (!glfwInit()) return -1;
    
    GLFWwindow* window = glfwCreateWindow(window_width, window_height, "MPM Sand Visualizer", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    
    glfwMakeContextCurrent(window);
    
    glfwSetKeyCallback(window, key_callback);
    
    init_mpm();
    
    std::cout << "Controls:\n[Space] Play/Pause\n[R] Reset\n[Esc] Exit" << std::endl;

    while (!glfwWindowShouldClose(window)) {
        if (running) {
            for(int i=0; i<4; ++i) solver.step(0.005f); 
        }
        
        glClearColor(0.2f, 0.2f, 0.25f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, 1.5, 0, 1.5, -2, 2); 
        
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(30, 1, 0, 0); 
        glRotatef(-45, 0, 1, 0); 
        glTranslatef(0.2f, -0.2f, 0); 
        
        glPointSize(3.0f);
        glBegin(GL_POINTS);
        
        const auto& particles = solver.get_particles();
        for (const auto& p : particles) {
            // Explicitly access members
            float speed = p.velocity.length();
            float t = std::min(speed / 5.0f, 1.0f);
            
            float r = 0.9f; 
            float g = 0.8f - t * 0.5f;
            float b = 0.5f - t * 0.5f;
            
            glColor3f(r, g, b);
            glVertex3f(p.position.x, p.position.y, p.position.z);
        }
        glEnd();
        
        glColor3f(0.5f, 0.5f, 0.5f);
        glBegin(GL_LINES);
        for(float i=0; i<=1.5f; i+=0.1f) {
            glVertex3f(i, 0, 0); glVertex3f(i, 0, 1.5f);
            glVertex3f(0, 0, i); glVertex3f(1.5f, 0, i);
        }
        glEnd();

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwTerminate();
    return 0;
}
