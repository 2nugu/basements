#ifndef BASEMENTS_ENGINE_CORE_COMPUTE_GRAPH_H
#define BASEMENTS_ENGINE_CORE_COMPUTE_GRAPH_H

#include "quantity_handle.h"
#include "quantity_storage.h"
#include <vector>
#include <cstdint>
#include <cmath>

namespace basements {
namespace engine {

/// Operation codes for compute graph
enum class OpCode : uint8_t {
    // Arithmetic
    ADD,        // dst = src1 + src2
    SUB,        // dst = src1 - src2
    MUL,        // dst = src1 * src2
    DIV,        // dst = src1 / src2
    
    // Math functions
    SQRT,       // dst = sqrt(src1)
    SQR,        // dst = src1 * src1
    ABS,        // dst = abs(src1)
    NEG,        // dst = -src1
    
    // Constants
    LOAD_CONST, // dst = constant
    COPY,       // dst = src1
    
    // Vector operations (3D)
    DOT3,       // dst = dot(vec1, vec2)
    CROSS3_X,   // dst = cross(vec1, vec2).x
    CROSS3_Y,   // dst = cross(vec1, vec2).y
    CROSS3_Z,   // dst = cross(vec1, vec2).z
    LENGTH3,    // dst = length(vec)
    
    // Conditional
    MAX,        // dst = max(src1, src2)
    MIN,        // dst = min(src1, src2)
};

/// Single instruction in compute graph
struct Instruction {
    OpCode operation_code;
    uint16_t destination_quantity_index;    // Where to store the result
    uint16_t source_quantity_index_1;       // First operand
    uint16_t source_quantity_index_2;       // Second operand (unused for unary ops)
    float constant_value;                    // Constant value (for LOAD_CONST operation)
    
    Instruction() 
        : operation_code(OpCode::COPY)
        , destination_quantity_index(0)
        , source_quantity_index_1(0)
        , source_quantity_index_2(0)
        , constant_value(0.0f) {}
    
    Instruction(OpCode operation, uint16_t destination, uint16_t source1, uint16_t source2 = 0, float constant = 0.0f)
        : operation_code(operation)
        , destination_quantity_index(destination)
        , source_quantity_index_1(source1)
        , source_quantity_index_2(source2)
        , constant_value(constant) {}
};

/// Compute graph for efficient batch execution
class ComputeGraph {
public:
    ComputeGraph() = default;

    /// Add a single instruction to the compute graph
    void add_instruction(const Instruction& instruction_to_add) {
        instructions_.push_back(instruction_to_add);
    }

    /// Clear all instructions from the compute graph
    void clear_all_instructions() {
        instructions_.clear();
    }

    /// Execute graph on storage (CPU)
    void execute(QuantityStorage& storage) const {
        for (const auto& inst : instructions_) {
            execute_instruction(inst, storage);
        }
    }

    /// Get instruction count
    size_t size() const { return instructions_.size(); }

    /// Get instructions (for GPU transfer)
    const Instruction* data() const { return instructions_.data(); }

    /// Reserve space
    void reserve(size_t n) { instructions_.reserve(n); }

private:
    void execute_instruction(const Instruction& inst, QuantityStorage& storage) const {
        QuantityHandle dst(inst.destination_quantity_index);
        QuantityHandle src1(inst.source_quantity_index_1);
        QuantityHandle src2(inst.source_quantity_index_2);
        
        float v1 = storage.get(src1);
        float v2 = storage.get(src2);
        float result = 0.0f;
        
        switch (inst.operation_code) {
            case OpCode::ADD:
                result = v1 + v2;
                break;
            case OpCode::SUB:
                result = v1 - v2;
                break;
            case OpCode::MUL:
                result = v1 * v2;
                break;
            case OpCode::DIV:
                // Safe division: check for near-zero divisor
                if (std::abs(v2) > 1e-10f) {
                    result = v1 / v2;
                } else {
                    result = 0.0f;
                }
                break;
            case OpCode::SQRT:
                // Safe sqrt: check for negative input
                if (v1 >= 0.0f) {
                    result = std::sqrt(v1);
                } else {
                    result = 0.0f;
                }
                break;
            case OpCode::SQR:
                result = v1 * v1;
                break;
            case OpCode::ABS:
                result = std::abs(v1);
                break;
            case OpCode::NEG:
                result = -v1;
                break;
            case OpCode::LOAD_CONST:
                result = inst.constant_value;
                break;
            case OpCode::COPY:
                result = v1;
                break;
            case OpCode::MAX:
                result = (v1 > v2) ? v1 : v2;
                break;
            case OpCode::MIN:
                result = (v1 < v2) ? v1 : v2;
                break;
            default:
                result = 0.0f;
                break;
        }
        
        storage.set(dst, result);
    }

    std::vector<Instruction> instructions_;
};

/// Builder for common physics computations
class ComputeGraphBuilder {
public:
    ComputeGraphBuilder(ComputeGraph& graph, QuantityStorage& storage)
        : graph_(graph), storage_(storage) {}

    /// Build: kinetic_energy = 0.5 * mass * velocity^2
    void build_kinetic_energy(
        QuantityHandle dst,
        QuantityHandle mass,
        QuantityHandle velocity
    ) {
        // Allocate temporary for velocity^2
        QuantityHandle temp_v2 = storage_.add_quantity(0.0f);
        
        // temp_v2 = velocity^2
        graph_.add_instruction({OpCode::SQR, static_cast<uint16_t>(temp_v2.index), static_cast<uint16_t>(velocity.index)});
        
        // temp_v2 = mass * velocity^2
        graph_.add_instruction({OpCode::MUL, static_cast<uint16_t>(temp_v2.index), static_cast<uint16_t>(mass.index), static_cast<uint16_t>(temp_v2.index)});
        
        // Allocate constant 0.5
        QuantityHandle half = storage_.add_quantity(0.5f);
        
        // dst = 0.5 * (mass * velocity^2)
        graph_.add_instruction({OpCode::MUL, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(half.index), static_cast<uint16_t>(temp_v2.index)});
    }

    /// Build: potential_energy = mass * gravity * height
    void build_potential_energy(
        QuantityHandle dst,
        QuantityHandle mass,
        QuantityHandle gravity,
        QuantityHandle height
    ) {
        // temp = mass * gravity
        graph_.add_instruction({OpCode::MUL, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(mass.index), static_cast<uint16_t>(gravity.index)});
        
        // dst = temp * height
        graph_.add_instruction({OpCode::MUL, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(dst.index), static_cast<uint16_t>(height.index)});
    }

    /// Build: total_energy = kinetic + potential
    void build_total_energy(
        QuantityHandle dst,
        QuantityHandle kinetic,
        QuantityHandle potential
    ) {
        graph_.add_instruction({OpCode::ADD, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(kinetic.index), static_cast<uint16_t>(potential.index)});
    }

    /// Build: momentum = mass * velocity
    void build_momentum(
        QuantityHandle dst,
        QuantityHandle mass,
        QuantityHandle velocity
    ) {
        graph_.add_instruction({OpCode::MUL, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(mass.index), static_cast<uint16_t>(velocity.index)});
    }

    /// Build: speed = sqrt(vx^2 + vy^2 + vz^2)
    void build_speed(
        QuantityHandle dst,
        QuantityHandle vx,
        QuantityHandle vy,
        QuantityHandle vz
    ) {
        // Allocate temporaries
        QuantityHandle temp_vx2 = storage_.add_quantity(0.0f);
        QuantityHandle temp_vy2 = storage_.add_quantity(0.0f);
        QuantityHandle temp_vz2 = storage_.add_quantity(0.0f);
        QuantityHandle temp_sum = storage_.add_quantity(0.0f);
        
        // vx^2, vy^2, vz^2
        graph_.add_instruction({OpCode::SQR, static_cast<uint16_t>(temp_vx2.index), static_cast<uint16_t>(vx.index)});
        graph_.add_instruction({OpCode::SQR, static_cast<uint16_t>(temp_vy2.index), static_cast<uint16_t>(vy.index)});
        graph_.add_instruction({OpCode::SQR, static_cast<uint16_t>(temp_vz2.index), static_cast<uint16_t>(vz.index)});
        
        // sum = vx^2 + vy^2
        graph_.add_instruction({OpCode::ADD, static_cast<uint16_t>(temp_sum.index), static_cast<uint16_t>(temp_vx2.index), static_cast<uint16_t>(temp_vy2.index)});
        
        // sum = sum + vz^2
        graph_.add_instruction({OpCode::ADD, static_cast<uint16_t>(temp_sum.index), static_cast<uint16_t>(temp_sum.index), static_cast<uint16_t>(temp_vz2.index)});
        
        // dst = sqrt(sum)
        graph_.add_instruction({OpCode::SQRT, static_cast<uint16_t>(dst.index), static_cast<uint16_t>(temp_sum.index)});
    }

private:
    ComputeGraph& graph_;
    QuantityStorage& storage_;
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_CORE_COMPUTE_GRAPH_H
