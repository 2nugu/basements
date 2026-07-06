#ifndef BASEMENTS_EDITOR_SHADER_H
#define BASEMENTS_EDITOR_SHADER_H

#include <string>
#include <map>
#include "basements/core/math/vec3.h"
#include <glm/glm.hpp> // [NEW]

namespace basements {
namespace editor {

class Shader {
public:
    Shader(const std::string& vertex_src, const std::string& fragment_src);
    ~Shader();

    void bind() const;
    void unbind() const;

    void set_bool(const std::string& name, bool value) const;
    void set_int(const std::string& name, int value) const;
    void set_float(const std::string& name, float value) const;
    void set_vec3(const std::string& name, const math::Vec3& value) const;
    void set_vec3(const std::string& name, const glm::vec3& value) const; // [NEW] GLM support
    void set_vec3(const std::string& name, float x, float y, float z) const;
    void set_mat4(const std::string& name, const float* value) const; // Keep simple for now

    unsigned int get_id() const { return id_; }

private:
    unsigned int id_;
    
    void check_compile_errors(unsigned int shader, std::string type);
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_SHADER_H
