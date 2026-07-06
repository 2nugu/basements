#include "basements/graphics/renderer.h"
#include "basements/graphics/gl_loader.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <iostream>

namespace basements {
namespace editor {

// Internal data structure, not nested in Renderer class to avoid private access issues
struct RendererData {
    unsigned int cube_vao = 0;
    unsigned int cube_vbo = 0;
    
    unsigned int grid_vao = 0;
    unsigned int grid_vbo = 0;
    int grid_vertex_count = 0; // [NEW]

    std::unique_ptr<Shader> shader;
    
    // Scene data
    glm::mat4 view_projection;
};

// Static storage
static RendererData* s_impl = nullptr;

void Renderer::initialize() {
    s_impl = new RendererData();

    // 1. Setup Cube
    float vertices[] = {
        // positions
        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,

        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f, -0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,

         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,

        -0.5f, -0.5f, -0.5f,
         0.5f, -0.5f, -0.5f,
         0.5f, -0.5f,  0.5f,
         0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f,  0.5f,
        -0.5f, -0.5f, -0.5f,

        -0.5f,  0.5f, -0.5f,
         0.5f,  0.5f, -0.5f,
         0.5f,  0.5f,  0.5f,
         0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f,  0.5f,
        -0.5f,  0.5f, -0.5f
    };

    glGenVertexArrays(1, &s_impl->cube_vao);
    glGenBuffers(1, &s_impl->cube_vbo);

    glBindVertexArray(s_impl->cube_vao);
    glBindBuffer(GL_ARRAY_BUFFER, s_impl->cube_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    
    // Shader
    s_impl->shader = std::make_unique<Shader>(
        "editor/shaders/basic.vert",
        "editor/shaders/basic.frag"
    );

    // Setup Grid (XZ Plane)
    std::vector<float> grid_vertices;
    const int size = 10;
    const float step = 1.0f;
    for(int i = -size; i <= size; ++i) {
        // X lines
        grid_vertices.push_back((float)-size); grid_vertices.push_back(0); grid_vertices.push_back((float)i);
        grid_vertices.push_back((float)size);  grid_vertices.push_back(0); grid_vertices.push_back((float)i);
        // Z lines
        grid_vertices.push_back((float)i); grid_vertices.push_back(0); grid_vertices.push_back((float)-size);
        grid_vertices.push_back((float)i); grid_vertices.push_back(0); grid_vertices.push_back((float)size);
    }
    
    glGenVertexArrays(1, &s_impl->grid_vao);
    glGenBuffers(1, &s_impl->grid_vbo);
    glBindVertexArray(s_impl->grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, s_impl->grid_vbo);
    glBufferData(GL_ARRAY_BUFFER, grid_vertices.size() * sizeof(float), grid_vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glBindVertexArray(0);
}

void Renderer::shutdown() {
    delete s_impl;
    s_impl = nullptr;
}

void Renderer::set_clear_color(const glm::vec3& color) {
    glClearColor(color.r, color.g, color.b, 1.0f);
}

void Renderer::clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::begin_scene(const Camera& camera) {
    if (!s_impl) return;

    glm::mat4 projection;
    float p[16];
    camera.get_projection_matrix(p, 16.0f/9.0f); // Aspect ratio logic needed later
    // Convert float* to glm::mat4 manually or memcpy
    memcpy(&projection[0][0], p, 16 * sizeof(float));

    glm::mat4 view;
    float v[16];
    camera.get_view_matrix(v);
    memcpy(&view[0][0], v, 16 * sizeof(float));
    
    s_impl->view_projection = projection * view;
    
    s_impl->shader->bind();
    
    // We pass P and V separately in shader
    s_impl->shader->set_mat4("projection", &projection[0][0]);
    s_impl->shader->set_mat4("view", &view[0][0]);
}

void Renderer::end_scene() {
    if (s_impl) s_impl->shader->unbind();
}

void Renderer::draw_cube(const glm::vec3& position, const glm::vec3& size, const glm::vec3& color) {
    if (!s_impl) return;

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(position.x, position.y, position.z));
    model = glm::scale(model, glm::vec3(size.x, size.y, size.z));

    s_impl->shader->bind();
    s_impl->shader->set_vec3("objectColor", color);
    s_impl->shader->set_mat4("model", &model[0][0]);
    
    glBindVertexArray(s_impl->cube_vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}

void Renderer::draw_grid(int size, float step) {
    if (!s_impl) return;

    // Helper to rebuild grid if needed (simple check for now, can be optimized)
    // For now, we'll just draw a fixed large grid or rebuild it if parameters drastically change?
    // Actually, let's just use immediate mode-ish logic but cached in VBO for performance
    // OR just fix the logic to use the existing buffer effectively.
    
    // Check if we need to build the grid buffer
    if (s_impl->grid_vao == 0) {
        std::vector<float> grid_vertices;
        int half_size = size; 
        
        // Lines along X
        for(int i = -half_size; i <= half_size; i++) {
            float z = i * step;
            // Line Start
            grid_vertices.push_back((float)-half_size * step); grid_vertices.push_back(0.0f); grid_vertices.push_back(z);
            // Line End
            grid_vertices.push_back((float)half_size * step); grid_vertices.push_back(0.0f); grid_vertices.push_back(z);
        }

        // Lines along Z
        for(int i = -half_size; i <= half_size; i++) {
            float x = i * step;
            // Line Start
            grid_vertices.push_back(x); grid_vertices.push_back(0.0f); grid_vertices.push_back((float)-half_size * step);
            // Line End
            grid_vertices.push_back(x); grid_vertices.push_back(0.0f); grid_vertices.push_back((float)half_size * step);
        }

        glGenVertexArrays(1, &s_impl->grid_vao);
        glGenBuffers(1, &s_impl->grid_vbo);
        glBindVertexArray(s_impl->grid_vao);
        glBindBuffer(GL_ARRAY_BUFFER, s_impl->grid_vbo);
        glBufferData(GL_ARRAY_BUFFER, grid_vertices.size() * sizeof(float), grid_vertices.data(), GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        
        // Count: (2 * halt_size + 1) lines * 2 vertices * 2 axes
        s_impl->grid_vertex_count = (int)grid_vertices.size() / 3;
    }

    glm::mat4 model = glm::mat4(1.0f);
    s_impl->shader->bind();
    s_impl->shader->set_vec3("objectColor", 0.4f, 0.4f, 0.4f); // Slightly brighter gray
    s_impl->shader->set_mat4("model", &model[0][0]);

    glBindVertexArray(s_impl->grid_vao);
    glDrawArrays(GL_LINES, 0, s_impl->grid_vertex_count);
    glBindVertexArray(0);
}

void Renderer::draw_mesh(Mesh& mesh, const glm::vec3& position, const glm::vec3& color) {
    if (!s_impl) return;

    s_impl->shader->bind();
    s_impl->shader->set_vec3("objectColor", color);

    glm::mat4 model = glm::translate(glm::mat4(1.0f), position);
    s_impl->shader->set_mat4("model", &model[0][0]);

    mesh.draw();
}

void Renderer::draw_line(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color) {
    if (!s_impl) return;

    // TODO: Optimize this to batch lines or use a persistent buffer
    // For now, we use a simple VBO for immediate line drawing
    static unsigned int line_vao = 0;
    static unsigned int line_vbo = 0;

    if (line_vao == 0) {
        glGenVertexArrays(1, &line_vao);
        glGenBuffers(1, &line_vbo);
        
        glBindVertexArray(line_vao);
        glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
        // Allocate space for 2 vertices * 3 floats
        glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
        
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBindVertexArray(0);
    }

    float vertices[6] = {
        start.x, start.y, start.z,
        end.x,   end.y,   end.z
    };

    glBindBuffer(GL_ARRAY_BUFFER, line_vbo);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), vertices, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    s_impl->shader->bind();
    s_impl->shader->set_vec3("objectColor", color);
    glm::mat4 model = glm::mat4(1.0f);
    s_impl->shader->set_mat4("model", &model[0][0]);

    glBindVertexArray(line_vao);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);
}

void Renderer::draw_aabb(const glm::vec3& min, const glm::vec3& max, const glm::vec3& color) {
    // Draw 12 edges of AABB
    // Bottom face
    draw_line(glm::vec3(min.x, min.y, min.z), glm::vec3(max.x, min.y, min.z), color);
    draw_line(glm::vec3(max.x, min.y, min.z), glm::vec3(max.x, min.y, max.z), color);
    draw_line(glm::vec3(max.x, min.y, max.z), glm::vec3(min.x, min.y, max.z), color);
    draw_line(glm::vec3(min.x, min.y, max.z), glm::vec3(min.x, min.y, min.z), color);
    
    // Top face
    draw_line(glm::vec3(min.x, max.y, min.z), glm::vec3(max.x, max.y, min.z), color);
    draw_line(glm::vec3(max.x, max.y, min.z), glm::vec3(max.x, max.y, max.z), color);
    draw_line(glm::vec3(max.x, max.y, max.z), glm::vec3(min.x, max.y, max.z), color);
    draw_line(glm::vec3(min.x, max.y, max.z), glm::vec3(min.x, max.y, min.z), color);
    
    // Vertical edges
    draw_line(glm::vec3(min.x, min.y, min.z), glm::vec3(min.x, max.y, min.z), color);
    draw_line(glm::vec3(max.x, min.y, min.z), glm::vec3(max.x, max.y, min.z), color);
    draw_line(glm::vec3(max.x, min.y, max.z), glm::vec3(max.x, max.y, max.z), color);
    draw_line(glm::vec3(min.x, min.y, max.z), glm::vec3(min.x, max.y, max.z), color);
}


} // namespace editor
} // namespace basements
