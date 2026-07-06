/**
 * @file memory_manager.h
 * @brief Unified Memory Architecture (Tiered RAM-VRAM Management)
 * 
 * Implements the "Pay only for what you use" memory strategy:
 * - Tier 0 (Hot): GPU VRAM - Active simulation data
 * - Tier 1 (Warm): Pinned Host RAM - Fast CPU-GPU transfer
 * - Tier 2 (Cold): Paged Host RAM - Sleeping/distant objects
 * - Tier 3 (Archived): SSD - Inactive map chunks (future)
 */

#ifndef BASEMENTS_MEMORY_MEMORY_MANAGER_H
#define BASEMENTS_MEMORY_MEMORY_MANAGER_H

#include "basements/core/math/common.h"
#include <cstdlib>
#include <cstring>
#include <vector>
#include <unordered_map>
#include <mutex>
#include <memory>

#ifdef _WIN32
#include <malloc.h> // _aligned_malloc, _aligned_free
#endif

#ifdef BASEMENTS_USE_CUDA
#include <cuda_runtime.h>
#endif

namespace basements {
namespace memory {

/**
 * @brief Memory Tier Classification
 */
enum class MemoryTier : uint8_t {
    HOT    = 0,  // GPU VRAM (Fastest, Limited)
    WARM   = 1,  // Pinned Host RAM (Fast DMA)
    COLD   = 2,  // Paged Host RAM (OS Managed)
    ARCHIVED = 3 // SSD/Disk (Future)
};

/**
 * @brief Memory Block Metadata
 */
struct MemoryBlock {
    void* ptr;
    size_t size;
    MemoryTier tier;
    bool is_pinned;
    uint64_t last_access_frame;
    
    MemoryBlock() : ptr(nullptr), size(0), tier(MemoryTier::COLD), 
                    is_pinned(false), last_access_frame(0) {}
};

/**
 * @brief Unified Memory Manager
 * 
 * Provides a single interface for allocating memory across different tiers.
 * Automatically promotes/demotes data between tiers based on access patterns.
 */
class MemoryManager {
public:
    // Singleton access
    static MemoryManager& instance() {
        static MemoryManager mgr;
        return mgr;
    }
    
    /**
     * @brief Allocate memory at a specific tier
     * 
     * @param size Bytes to allocate
     * @param tier Target memory tier
     * @return Allocation handle (ID)
     */
    uint64_t allocate(size_t size, MemoryTier tier = MemoryTier::COLD) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        MemoryBlock block;
        block.size = size;
        block.tier = tier;
        block.last_access_frame = current_frame_;
        
        switch (tier) {
            case MemoryTier::HOT:
                block.ptr = allocate_gpu(size);
                stats_.gpu_allocated += size;
                break;
            case MemoryTier::WARM:
                block.ptr = allocate_pinned(size);
                block.is_pinned = true;
                stats_.pinned_allocated += size;
                break;
            case MemoryTier::COLD:
            default:
                block.ptr = allocate_paged(size);
                stats_.paged_allocated += size;
                break;
        }
        
        if (!block.ptr) {
            return INVALID_HANDLE;
        }
        
        uint64_t handle = next_handle_++;
        blocks_[handle] = block;
        return handle;
    }
    
    /**
     * @brief Free allocated memory
     */
    void free(uint64_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = blocks_.find(handle);
        if (it == blocks_.end()) return;
        
        MemoryBlock& block = it->second;
        
        switch (block.tier) {
            case MemoryTier::HOT:
                free_gpu(block.ptr);
                stats_.gpu_allocated -= block.size;
                break;
            case MemoryTier::WARM:
                free_pinned(block.ptr);
                stats_.pinned_allocated -= block.size;
                break;
            case MemoryTier::COLD:
            default:
                free_paged(block.ptr);
                stats_.paged_allocated -= block.size;
                break;
        }
        
        blocks_.erase(it);
    }
    
    /**
     * @brief Get raw pointer from handle
     */
    void* get_ptr(uint64_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = blocks_.find(handle);
        if (it == blocks_.end()) return nullptr;
        
        it->second.last_access_frame = current_frame_;
        return it->second.ptr;
    }
    
    /**
     * @brief Promote memory to a faster tier (COLD -> WARM -> HOT)
     */
    bool promote(uint64_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = blocks_.find(handle);
        if (it == blocks_.end()) return false;
        
        MemoryBlock& block = it->second;
        MemoryTier target_tier;
        
        switch (block.tier) {
            case MemoryTier::COLD:
                target_tier = MemoryTier::WARM;
                break;
            case MemoryTier::WARM:
                target_tier = MemoryTier::HOT;
                break;
            default:
                return false; // Already at fastest tier
        }
        
        return migrate_block(block, target_tier);
    }
    
    /**
     * @brief Demote memory to a slower tier (HOT -> WARM -> COLD)
     */
    bool demote(uint64_t handle) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        auto it = blocks_.find(handle);
        if (it == blocks_.end()) return false;
        
        MemoryBlock& block = it->second;
        MemoryTier target_tier;
        
        switch (block.tier) {
            case MemoryTier::HOT:
                target_tier = MemoryTier::WARM;
                break;
            case MemoryTier::WARM:
                target_tier = MemoryTier::COLD;
                break;
            default:
                return false; // Already at slowest tier
        }
        
        return migrate_block(block, target_tier);
    }
    
    /**
     * @brief Advance frame counter (called once per simulation step)
     */
    void tick() { current_frame_++; }
    
    /**
     * @brief Memory Statistics
     */
    struct Stats {
        size_t gpu_allocated = 0;
        size_t pinned_allocated = 0;
        size_t paged_allocated = 0;
        
        size_t total() const { 
            return gpu_allocated + pinned_allocated + paged_allocated; 
        }
    };
    
    /**
     * @brief Memory Budget Configuration
     */
    struct BudgetConfig {
        size_t gpu_budget = 0;      // 0 = unlimited
        size_t pinned_budget = 0;   // 0 = unlimited
        size_t paged_budget = 0;    // 0 = unlimited
        bool auto_evict = true;     // Automatically demote when over budget
    };
    
    void set_budget(const BudgetConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        budget_ = config;
    }
    
    const BudgetConfig& get_budget() const { return budget_; }
    
    const Stats& get_stats() const { return stats_; }
    
    /**
     * @brief Enforce memory budgets by demoting least recently used blocks
     */
    void enforce_budgets() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Check GPU budget
        if (budget_.gpu_budget > 0 && stats_.gpu_allocated > budget_.gpu_budget) {
            evict_lru_from_tier(MemoryTier::HOT);
        }
        
        // Check Pinned budget
        if (budget_.pinned_budget > 0 && stats_.pinned_allocated > budget_.pinned_budget) {
            evict_lru_from_tier(MemoryTier::WARM);
        }
    }
    
    /**
     * @brief Prefetch hint for async data transfer (CUDA)
     */
    void prefetch_to_gpu(uint64_t handle) {
#ifdef BASEMENTS_USE_CUDA
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = blocks_.find(handle);
        if (it == blocks_.end()) return;
        
        MemoryBlock& block = it->second;
        if (block.tier == MemoryTier::HOT) return; // Already on GPU
        
        // Async prefetch using CUDA streams
        cudaMemPrefetchAsync(block.ptr, block.size, 0, 0);
#else
        (void)handle; // No-op without CUDA
#endif
    }
    
    static constexpr uint64_t INVALID_HANDLE = 0;
    
private:
    MemoryManager() : next_handle_(1), current_frame_(0) {
        // Default budget: 4GB GPU, 16GB Pinned, Unlimited Paged
        budget_.gpu_budget = 4ULL * 1024 * 1024 * 1024;
        budget_.pinned_budget = 16ULL * 1024 * 1024 * 1024;
        budget_.paged_budget = 0; // Unlimited
        budget_.auto_evict = true;
    }
    ~MemoryManager() { clear_all(); }
    
    MemoryManager(const MemoryManager&) = delete;
    MemoryManager& operator=(const MemoryManager&) = delete;
    
    void clear_all() {
        for (auto& [handle, block] : blocks_) {
            switch (block.tier) {
                case MemoryTier::HOT: free_gpu(block.ptr); break;
                case MemoryTier::WARM: free_pinned(block.ptr); break;
                default: free_paged(block.ptr); break;
            }
        }
        blocks_.clear();
    }
    
    // ========================================
    // Tier-Specific Allocators
    // ========================================
    
    void* allocate_gpu(size_t size) {
#ifdef BASEMENTS_USE_CUDA
        void* ptr = nullptr;
        cudaError_t err = cudaMalloc(&ptr, size);
        return (err == cudaSuccess) ? ptr : nullptr;
#else
        // Fallback: simulate GPU with aligned malloc
    #ifdef _WIN32
        return _aligned_malloc(size, 256);
    #else
        return std::aligned_alloc(256, size);
    #endif
#endif
    }
    
    void free_gpu(void* ptr) {
#ifdef BASEMENTS_USE_CUDA
        cudaFree(ptr);
#else
    #ifdef _WIN32
        _aligned_free(ptr);
    #else
        std::free(ptr);
    #endif
#endif
    }
    
    void* allocate_pinned(size_t size) {
#ifdef BASEMENTS_USE_CUDA
        void* ptr = nullptr;
        cudaError_t err = cudaMallocHost(&ptr, size);
        return (err == cudaSuccess) ? ptr : nullptr;
#else
        // Fallback: standard aligned allocation
    #ifdef _WIN32
        return _aligned_malloc(size, 64);
    #else
        return std::aligned_alloc(64, size);
    #endif
#endif
    }
    
    void free_pinned(void* ptr) {
#ifdef BASEMENTS_USE_CUDA
        cudaFreeHost(ptr);
#else
    #ifdef _WIN32
        _aligned_free(ptr);
    #else
        std::free(ptr);
    #endif
#endif
    }
    
    void* allocate_paged(size_t size) {
        return std::malloc(size);
    }
    
    void free_paged(void* ptr) {
        std::free(ptr);
    }
    
    // ========================================
    // Tier Migration (with data copy)
    // ========================================
    
    bool migrate_block(MemoryBlock& block, MemoryTier target_tier) {
        void* new_ptr = nullptr;
        
        // Allocate in target tier
        switch (target_tier) {
            case MemoryTier::HOT:
                new_ptr = allocate_gpu(block.size);
                if (!new_ptr) return false;
                stats_.gpu_allocated += block.size;
                break;
            case MemoryTier::WARM:
                new_ptr = allocate_pinned(block.size);
                if (!new_ptr) return false;
                stats_.pinned_allocated += block.size;
                break;
            case MemoryTier::COLD:
                new_ptr = allocate_paged(block.size);
                if (!new_ptr) return false;
                stats_.paged_allocated += block.size;
                break;
            default:
                return false;
        }
        
        // Copy data
        copy_data(new_ptr, target_tier, block.ptr, block.tier, block.size);
        
        // Free old allocation (update stats)
        switch (block.tier) {
            case MemoryTier::HOT:
                free_gpu(block.ptr);
                stats_.gpu_allocated -= block.size;
                break;
            case MemoryTier::WARM:
                free_pinned(block.ptr);
                stats_.pinned_allocated -= block.size;
                break;
            case MemoryTier::COLD:
                free_paged(block.ptr);
                stats_.paged_allocated -= block.size;
                break;
            default: break;
        }
        
        // Update block metadata
        block.ptr = new_ptr;
        block.tier = target_tier;
        block.is_pinned = (target_tier == MemoryTier::WARM);
        
        return true;
    }
    
    void copy_data(void* dst, MemoryTier dst_tier, 
                   void* src, MemoryTier src_tier, size_t size) {
        (void)dst_tier; (void)src_tier; // Used in CUDA path
#ifdef BASEMENTS_USE_CUDA
        cudaMemcpyKind kind = cudaMemcpyDefault;
        
        if (src_tier == MemoryTier::HOT && dst_tier != MemoryTier::HOT) {
            kind = cudaMemcpyDeviceToHost;
        } else if (src_tier != MemoryTier::HOT && dst_tier == MemoryTier::HOT) {
            kind = cudaMemcpyHostToDevice;
        } else if (src_tier == MemoryTier::HOT && dst_tier == MemoryTier::HOT) {
            kind = cudaMemcpyDeviceToDevice;
        } else {
            kind = cudaMemcpyHostToHost;
        }
        
        cudaMemcpy(dst, src, size, kind);
#else
        std::memcpy(dst, src, size);
#endif
    }
    
    // ========================================
    // State
    // ========================================
    std::unordered_map<uint64_t, MemoryBlock> blocks_;
    std::mutex mutex_;
    uint64_t next_handle_;
    uint64_t current_frame_;
    Stats stats_;
    BudgetConfig budget_;
    
    /**
     * @brief Evict Least Recently Used block from a tier
     */
    void evict_lru_from_tier(MemoryTier tier) {
        uint64_t oldest_handle = 0;
        uint64_t oldest_frame = UINT64_MAX;
        
        for (auto& [handle, block] : blocks_) {
            if (block.tier == tier && block.last_access_frame < oldest_frame) {
                oldest_frame = block.last_access_frame;
                oldest_handle = handle;
            }
        }
        
        if (oldest_handle != 0) {
            // Demote (without lock since we already hold it)
            auto it = blocks_.find(oldest_handle);
            if (it != blocks_.end()) {
                MemoryBlock& block = it->second;
                MemoryTier target = (tier == MemoryTier::HOT) ? MemoryTier::WARM : MemoryTier::COLD;
                migrate_block(block, target);
            }
        }
    }
};

} // namespace memory
} // namespace basements

#endif // BASEMENTS_MEMORY_MEMORY_MANAGER_H
