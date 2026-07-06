#pragma once

#include "basements/graphics/mesh.h"
#include <vector>
#include <memory>

// Forward declaration - we might include the full header here if it's small, 
// but let's try to keep it clean if possible. FastNoiseLite is header-only though.
// For now, let's include it in the cpp to avoid pollution, or just here if needed for member.
// Since FastNoiseLite is a class, we need it if we store it as a member, or we can use void* or PIMPL.
// For simplicity in this project, including the header is fine, it's a single header.
// However, to be safe with include paths, let's just use it in CPP.

namespace basements {

    struct HeightmapParams {
        int width = 100;
        int depth = 100;
        float scale = 1.0f;
        float height_multiplier = 10.0f;
        float noise_frequency = 0.01f;
        int seed = 1337;
        int octaves = 3;
        float lacunarity = 2.0f;
        float gain = 0.5f;
    };

    class WorldGenerator {
    public:
        // Generates a mesh based on noise parameters
        // Returns a shared_ptr to a Mesh that can be attached to a SceneNode
        static std::shared_ptr<editor::Mesh> generate_terrain(const HeightmapParams& params);

    private:
        static void calculate_normals(std::vector<editor::Vertex>& vertices, const std::vector<unsigned int>& indices);
    };

}
