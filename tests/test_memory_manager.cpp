/**
 * @file test_memory_manager.cpp
 * @brief Unit tests for Unified Memory Manager
 */

#include <gtest/gtest.h>
#include "basements/core/memory/memory_manager.h"

using namespace basements::memory;

TEST(MemoryManagerTest, BasicAllocation) {
    auto& mgr = MemoryManager::instance();
    
    // Allocate 1KB in COLD tier (standard RAM)
    uint64_t handle = mgr.allocate(1024, MemoryTier::COLD);
    EXPECT_NE(handle, MemoryManager::INVALID_HANDLE);
    
    void* ptr = mgr.get_ptr(handle);
    EXPECT_NE(ptr, nullptr);
    
    // Write something to verify it works
    int* data = static_cast<int*>(ptr);
    data[0] = 42;
    EXPECT_EQ(data[0], 42);
    
    mgr.free(handle);
}

TEST(MemoryManagerTest, TierPromotion) {
    auto& mgr = MemoryManager::instance();
    
    // Start in COLD
    uint64_t handle = mgr.allocate(512, MemoryTier::COLD);
    EXPECT_NE(handle, MemoryManager::INVALID_HANDLE);
    
    // Write initial data
    int* ptr = static_cast<int*>(mgr.get_ptr(handle));
    ptr[0] = 123;
    
    // Promote to WARM (Pinned)
    bool promoted = mgr.promote(handle);
    EXPECT_TRUE(promoted);
    
    // Data should be preserved after promotion
    ptr = static_cast<int*>(mgr.get_ptr(handle));
    EXPECT_EQ(ptr[0], 123);
    
    mgr.free(handle);
}

TEST(MemoryManagerTest, TierDemotion) {
    auto& mgr = MemoryManager::instance();
    
    // Start in WARM
    uint64_t handle = mgr.allocate(256, MemoryTier::WARM);
    EXPECT_NE(handle, MemoryManager::INVALID_HANDLE);
    
    // Write initial data
    float* ptr = static_cast<float*>(mgr.get_ptr(handle));
    ptr[0] = 3.14f;
    
    // Demote to COLD
    bool demoted = mgr.demote(handle);
    EXPECT_TRUE(demoted);
    
    // Data should be preserved after demotion
    ptr = static_cast<float*>(mgr.get_ptr(handle));
    EXPECT_FLOAT_EQ(ptr[0], 3.14f);
    
    mgr.free(handle);
}

TEST(MemoryManagerTest, StatsTracking) {
    auto& mgr = MemoryManager::instance();
    
    auto stats_before = mgr.get_stats();
    
    uint64_t h1 = mgr.allocate(1024, MemoryTier::COLD);
    uint64_t h2 = mgr.allocate(2048, MemoryTier::WARM);
    
    auto stats_after = mgr.get_stats();
    
    EXPECT_GE(stats_after.paged_allocated, stats_before.paged_allocated + 1024);
    EXPECT_GE(stats_after.pinned_allocated, stats_before.pinned_allocated + 2048);
    
    mgr.free(h1);
    mgr.free(h2);
}
