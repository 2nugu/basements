#ifndef BASEMENTS_ENGINE_SERIALIZATION_H
#define BASEMENTS_ENGINE_SERIALIZATION_H

#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include <string>
#include <fstream>
#include <sstream>

namespace basements {
namespace engine {

using namespace basements::math;

/// Simple JSON-like serialization for physics state
class Serializer {
public:
    /// Serialize Vec3 to string
    static std::string serialize_vec3(const Vec3& v) {
        std::ostringstream oss;
        oss << "{\"x\":" << v.x << ",\"y\":" << v.y << ",\"z\":" << v.z << "}";
        return oss.str();
    }

    /// Serialize Quaternion to string
    static std::string serialize_quaternion(const Quaternion& q) {
        std::ostringstream oss;
        oss << "{\"w\":" << q.w << ",\"x\":" << q.x << ",\"y\":" << q.y << ",\"z\":" << q.z << "}";
        return oss.str();
    }

    /// Serialize float array to string
    static std::string serialize_float_array(const float* data, size_t count) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < count; ++i) {
            if (i > 0) oss << ",";
            oss << data[i];
        }
        oss << "]";
        return oss.str();
    }

    /// Save string to file
    static bool save_to_file(const std::string& filename, const std::string& content) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        file << content;
        file.close();
        return true;
    }

    /// Load string from file
    static bool load_from_file(const std::string& filename, std::string& content) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        std::stringstream buffer;
        buffer << file.rdbuf();
        content = buffer.str();
        file.close();
        return true;
    }
};

/// Serializable interface
class Serializable {
public:
    virtual ~Serializable() = default;

    /// Serialize to string
    virtual std::string serialize() const = 0;

    /// Deserialize from string
    virtual bool deserialize(const std::string& data) = 0;

    /// Save to file
    bool save_to_file(const std::string& filename) const {
        return Serializer::save_to_file(filename, serialize());
    }

    /// Load from file
    bool load_from_file(const std::string& filename) {
        std::string content;
        if (!Serializer::load_from_file(filename, content)) {
            return false;
        }
        return deserialize(content);
    }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_SERIALIZATION_H
