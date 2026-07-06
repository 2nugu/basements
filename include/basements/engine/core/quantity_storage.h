#ifndef BASEMENTS_ENGINE_CORE_QUANTITY_STORAGE_H
#define BASEMENTS_ENGINE_CORE_QUANTITY_STORAGE_H

#include "quantity_handle.h"
#include <vector>
#include <cstdint>
#include <cstring>

namespace basements {
namespace engine {

/// Flat, cache-friendly storage for quantities
/// Optimized for batch processing and GPU transfer
class QuantityStorage {
public:
    QuantityStorage() = default;

    /// Reserve space for N quantities to avoid reallocation
    void reserve_quantity_storage(size_t quantity_count) {
        quantity_values_.reserve(quantity_count);
        dirty_flag_bits_.reserve((quantity_count + 63) / 64);  // Bit-packed (64 flags per word)
    }

    /// Add a new quantity to storage, returns handle for access
    QuantityHandle add_quantity(float initial_value = 0.0f) {
        uint32_t new_quantity_index = static_cast<uint32_t>(quantity_values_.size());
        quantity_values_.push_back(initial_value);
        
        // Ensure dirty flag bits array is large enough
        size_t required_bit_words = (quantity_values_.size() + 63) / 64;
        if (dirty_flag_bits_.size() < required_bit_words) {
            dirty_flag_bits_.resize(required_bit_words, 0);
        }
        
        mark_dirty(QuantityHandle(new_quantity_index));
        return QuantityHandle(new_quantity_index);
    }

    /// Get value (O(1))
    float get(QuantityHandle handle) const {
        return quantity_values_[handle.index];
    }

    /// Set value (O(1))
    void set(QuantityHandle handle, float value) {
        quantity_values_[handle.index] = value;
        mark_dirty(handle);
    }

    /// Check if dirty (O(1))
    bool is_dirty(QuantityHandle handle) const {
        size_t word = handle.index / 64;
        size_t bit = handle.index % 64;
        return (dirty_flag_bits_[word] >> bit) & 1;
    }

    /// Mark as dirty (O(1), const because dirty flags are mutable metadata)
    void mark_dirty(QuantityHandle handle) const {
        size_t word = handle.index / 64;
        size_t bit = handle.index % 64;
        dirty_flag_bits_[word] |= (1ULL << bit);
    }

    /// Clear dirty flag (O(1))
    void clear_dirty(QuantityHandle handle) const {
        size_t word = handle.index / 64;
        size_t bit = handle.index % 64;
        dirty_flag_bits_[word] &= ~(1ULL << bit);
    }

    /// Get raw data pointer (for GPU transfer)
    const float* data() const { return quantity_values_.data(); }
    float* data() { return quantity_values_.data(); }

    /// Get size
    size_t size() const { return quantity_values_.size(); }

    /// Clear all
    void clear() {
        quantity_values_.clear();
        dirty_flag_bits_.clear();
    }

    /// Batch operations (SIMD-friendly)
    void add_batch(const QuantityHandle* handles, const float* values, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            quantity_values_[handles[i].index] += values[i];
            mark_dirty(handles[i]);
        }
    }

    void multiply_batch(const QuantityHandle* handles, float scalar, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            quantity_values_[handles[i].index] *= scalar;
            mark_dirty(handles[i]);
        }
    }

private:
    std::vector<float> quantity_values_;
    mutable std::vector<uint64_t> dirty_flag_bits_;  // Bit-packed dirty flags
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_QUANTITY_STORAGE_H
