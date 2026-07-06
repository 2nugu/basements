#include "basements/graphics/shader.h"
#include "basements/graphics/gl_loader.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <glm/glm.hpp> // [NEW]

namespace basements {
namespace editor {

static std::string load_file(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Shader] Cannot open: " << path << std::endl;
        return "";
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

Shader::Shader(const std::string& vertex_path, const std::string& fragment_path) {
    std::string vs_src = load_file(vertex_path);
    std::string fs_src = load_file(fragment_path);

    unsigned int vertex = glCreateShader(GL_VERTEX_SHADER);
    const char* v_src = vs_src.c_str();
    glShaderSource(vertex, 1, &v_src, nullptr);
    glCompileShader(vertex);
    check_compile_errors(vertex, "VERTEX");

    unsigned int fragment = glCreateShader(GL_FRAGMENT_SHADER);
    const char* f_src = fs_src.c_str();
    glShaderSource(fragment, 1, &f_src, nullptr);
    glCompileShader(fragment);
    check_compile_errors(fragment, "FRAGMENT");

    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    glLinkProgram(id_);
    check_compile_errors(id_, "PROGRAM");

    glDeleteShader(vertex);
    glDeleteShader(fragment);
}

Shader::~Shader() {
    glDeleteProgram(id_);
}

void Shader::bind() const {
    glUseProgram(id_);
}

void Shader::unbind() const {
    glUseProgram(0);
}

void Shader::set_bool(const std::string& name, bool value) const {
    glUniform1i(glGetUniformLocation(id_, name.c_str()), (int)value);
}

void Shader::set_int(const std::string& name, int value) const {
    glUniform1i(glGetUniformLocation(id_, name.c_str()), value);
}

void Shader::set_float(const std::string& name, float value) const {
    glUniform1f(glGetUniformLocation(id_, name.c_str()), value);
}

void Shader::set_vec3(const std::string& name, const math::Vec3& value) const {
    glUniform3f(glGetUniformLocation(id_, name.c_str()), value.x, value.y, value.z);
}

void Shader::set_vec3(const std::string& name, const glm::vec3& value) const {
     glUniform3f(glGetUniformLocation(id_, name.c_str()), value.x, value.y, value.z);
}

void Shader::set_vec3(const std::string& name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(id_, name.c_str()), x, y, z);
}

void Shader::set_mat4(const std::string& name, const float* value) const {
    glUniformMatrix4fv(glGetUniformLocation(id_, name.c_str()), 1, GL_FALSE, value);
}

void Shader::check_compile_errors(unsigned int shader, std::string type) {
    int success;
    char info_log[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, nullptr, info_log);
            std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, nullptr, info_log);
            std::cerr << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << info_log << "\n -- --------------------------------------------------- -- " << std::endl;
        }
    }
}

} // namespace editor
} // namespace basements
