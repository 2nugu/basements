#ifndef BASEMENTS_ENGINE_ERROR_HANDLING_H
#define BASEMENTS_ENGINE_ERROR_HANDLING_H

#include <exception>
#include <string>

namespace basements {
namespace engine {

/// Error codes for physics engine
enum class EngineError {
    NONE = 0,
    INVALID_HANDLE,
    CIRCULAR_DEPENDENCY,
    OUT_OF_MEMORY,
    INVALID_STATE,
    DIVISION_BY_ZERO,
    NUMERICAL_INSTABILITY,
    GPU_ERROR,
    SERIALIZATION_ERROR,
    PLUGIN_ERROR
};

/// Convert error code to string
inline const char* error_to_string(EngineError error) {
    switch (error) {
        case EngineError::NONE: return "No error";
        case EngineError::INVALID_HANDLE: return "Invalid handle";
        case EngineError::CIRCULAR_DEPENDENCY: return "Circular dependency detected";
        case EngineError::OUT_OF_MEMORY: return "Out of memory";
        case EngineError::INVALID_STATE: return "Invalid engine state";
        case EngineError::DIVISION_BY_ZERO: return "Division by zero";
        case EngineError::NUMERICAL_INSTABILITY: return "Numerical instability";
        case EngineError::GPU_ERROR: return "GPU error";
        case EngineError::SERIALIZATION_ERROR: return "Serialization error";
        case EngineError::PLUGIN_ERROR: return "Plugin error";
        default: return "Unknown error";
    }
}

/// Physics engine exception
class EngineException : public std::exception {
public:
    EngineException(EngineError error_code, const std::string& message)
        : error_code_(error_code)
        , message_(std::string(error_to_string(error_code)) + ": " + message)
    {}

    const char* what() const noexcept override {
        return message_.c_str();
    }

    EngineError error_code() const noexcept {
        return error_code_;
    }

private:
    EngineError error_code_;
    std::string message_;
};

/// Error handler callback type
using ErrorHandler = void(*)(EngineError error, const std::string& message);

/// Global error handler
class ErrorManager {
public:
    static void set_error_handler(ErrorHandler handler) {
        error_handler_ = handler;
    }

    static void report_error(EngineError error, const std::string& message) {
        if (error_handler_) {
            error_handler_(error, message);
        }
    }

    static void throw_error(EngineError error, const std::string& message) {
        report_error(error, message);
        throw EngineException(error, message);
    }

private:
    static ErrorHandler error_handler_;
};

// Initialize static member
inline ErrorHandler ErrorManager::error_handler_ = nullptr;

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_ERROR_HANDLING_H
