#include "basements/engine/procedural/world_generator.h"
#include "FastNoiseLite.h"
#include <iostream>

namespace basements {

namespace basements {

    std::shared_ptr<editor::Mesh> WorldGenerator::generate_terrain(const HeightmapParams& params) {
        FastNoiseLite noise;
        noise.SetSeed(params.seed);
        noise.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
        noise.SetFractalType(FastNoiseLite::FractalType_FBm);
        noise.SetFractalOctaves(params.octaves);
        noise.SetFractalLacunarity(params.lacunarity);
        noise.SetFractalGain(params.gain);
        noise.SetFrequency(params.noise_frequency);

        std::vector<editor::Vertex> vertices;
        std::vector<unsigned int> indices;

        // 1. Generate Vertices
        // Center the terrain around (0,0)
        float start_x = -params.width / 2.0f * params.scale;
        float start_z = -params.depth / 2.0f * params.scale;

        for (int z = 0; z < params.depth; ++z) {
            for (int x = 0; x < params.width; ++x) {
                float world_x = start_x + x * params.scale;
                float world_z = start_z + z * params.scale;
                
                // Get noise value (-1 to 1) and map to height
                float y = noise.GetNoise((float)x, (float)z) * params.height_multiplier;

                editor::Vertex v;
                v.position = glm::vec3(world_x, y, world_z);
                
                // Set default normal (up) - will be recalculated
                v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
                
                // Simple UV mapping
                v.texcoord = glm::vec2((float)x / params.width, (float)z / params.depth); // Note: Vertex has texcoord, not tex_coords

                vertices.push_back(v);
            }
        }

        // 2. Generate Indices (Triangles)
        for (int z = 0; z < params.depth - 1; ++z) {
            for (int x = 0; x < params.width - 1; ++x) {
                int topLeft = z * params.width + x;
                int topRight = topLeft + 1;
                int bottomLeft = (z + 1) * params.width + x;
                int bottomRight = bottomLeft + 1;

                // Triangle 1
                indices.push_back(topLeft);
                indices.push_back(bottomLeft);
                indices.push_back(topRight);

                // Triangle 2
                indices.push_back(topRight);
                indices.push_back(bottomLeft);
                indices.push_back(bottomRight);
            }
        }

        // 3. Recalculate Normals for lighting
        calculate_normals(vertices, indices);

        // 4. Create Mesh
        auto mesh = std::make_shared<editor::Mesh>();
        mesh->vertices = vertices;
        mesh->indices = indices;
        
        // This will upload to GPU
        mesh->setup_mesh();

        std::cout << "[WorldGenerator] Generated terrain with " << vertices.size() << " vertices." << std::endl;

        return mesh;
    }

    void WorldGenerator::calculate_normals(std::vector<editor::Vertex>& vertices, const std::vector<unsigned int>& indices) {
        // Reset normals
        for (auto& v : vertices) {
            v.normal = glm::vec3(0.0f);
        }

        // Accumulate face normals
        for (size_t i = 0; i < indices.size(); i += 3) {
            unsigned int i0 = indices[i];
            unsigned int i1 = indices[i + 1];
            unsigned int i2 = indices[i + 2];

            glm::vec3 v0 = vertices[i0].position;
            glm::vec3 v1 = vertices[i1].position;
            glm::vec3 v2 = vertices[i2].position;

            glm::vec3 edge1 = v1 - v0;
            glm::vec3 edge2 = v2 - v0;
            glm::vec3 normal = glm::cross(edge1, edge2); // Basic face normal

            vertices[i0].normal += normal;
            vertices[i1].normal += normal;
            vertices[i2].normal += normal;
        }

        // Normalize
        for (auto& v : vertices) {
            v.normal = glm::normalize(v.normal);
        }
    }

}

}
