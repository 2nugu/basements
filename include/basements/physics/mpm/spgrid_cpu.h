#pragma once

#include <vector>
#include <unordered_map>
#include <cstdint>
#include "basements/core/math/vec3.h"
#include "basements/core/math/common.h"

namespace basements {
namespace mpm {

    constexpr int BLOCK_SIZE_LOG2 = 2; // 2^2 = 4
    constexpr int BLOCK_SIZE = 1 << BLOCK_SIZE_LOG2; // 4
    constexpr int BLOCK_MASK = BLOCK_SIZE - 1;       // 3 (0b11)

    struct GridNode {
        basements::math::Vec3 velocity = basements::math::Vec3(0,0,0);
        // Internal stress force accumulated during p2g (MLS-MPM):
        //   f_i = -Σ_p (4/dx^2) V_p^0 τ_p (x_i - x_p) w_ip
        // Applied during update_grid as v += (f/m + g)·dt.
        basements::math::Vec3 force = basements::math::Vec3(0,0,0);
        float mass = 0.0f;
        bool active = false;
    };

    struct GridBlock {
        GridNode nodes[BLOCK_SIZE * BLOCK_SIZE * BLOCK_SIZE];
        int origin_x, origin_y, origin_z;

        void clear() {
            for (auto& node : nodes) {
                node.velocity = basements::math::Vec3(0,0,0);
                node.force    = basements::math::Vec3(0,0,0);
                node.mass = 0.0f;
                node.active = false;
            }
        }
    };

    class SPGridCPU {
    public:
        using BlockMap = std::unordered_map<uint64_t, GridBlock>;

        SPGridCPU(float grid_spacing) : dx(grid_spacing), inv_dx(1.0f / grid_spacing) {}

        void clear() {
            for (auto& pair : blocks) {
                pair.second.clear();
            }
        }

        GridBlock* get_block(int bi, int bj, int bk) {
            uint64_t key = hash_coord(bi, bj, bk);
            auto it = blocks.find(key);
            if (it != blocks.end()) {
                return &it->second;
            }
            
            GridBlock& block = blocks[key];
            block.origin_x = bi * BLOCK_SIZE;
            block.origin_y = bj * BLOCK_SIZE;
            block.origin_z = bk * BLOCK_SIZE;
            block.clear();
            return &block;
        }

        GridNode* get_node(int i, int j, int k) {
            int bi = i >> BLOCK_SIZE_LOG2; 
            int bj = j >> BLOCK_SIZE_LOG2;
            int bk = k >> BLOCK_SIZE_LOG2;
            
            GridBlock* block = get_block(bi, bj, bk);
            
            int local_i = i & BLOCK_MASK; 
            int local_j = j & BLOCK_MASK;
            int local_k = k & BLOCK_MASK;
            
            int local_idx = (local_i * BLOCK_SIZE * BLOCK_SIZE) + (local_j * BLOCK_SIZE) + local_k;
            return &block->nodes[local_idx];
        }
        
        float get_dx() const { return dx; }
        float get_inv_dx() const { return inv_dx; }
        const BlockMap& get_blocks() const { return blocks; }
        BlockMap& get_blocks_mutable() { return blocks; }

    private:
        float dx;
        float inv_dx;
        BlockMap blocks;

        uint64_t hash_coord(int x, int y, int z) {
            uint64_t result = 0;
            result |= ((uint64_t)(x + 100000) & 0x1FFFFF);       
            result |= ((uint64_t)(y + 100000) & 0x1FFFFF) << 21; 
            result |= ((uint64_t)(z + 100000) & 0x1FFFFF) << 42; 
            return result;
        }
    };

} // namespace mpm
} // namespace basements
