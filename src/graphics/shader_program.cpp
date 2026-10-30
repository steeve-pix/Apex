#include "graphics/shader_program.hpp"
#include "graphics/shader.hpp"

#include <stdexcept>
#include <string>

namespace apex::graphics {
    ShaderProgram::ShaderProgram() : id_{glCreateProgram()} {
        if (id_ == 0) {
            throw std::runtime_error("Failed to create shader program");
        }
    }

    ShaderProgram::~ShaderProgram() {
        glDeleteProgram(id_);
    }

    void ShaderProgram::link(const Shader &vertex, const Shader &fragment) const {
        glAttachShader(id_, vertex.get());
        glAttachShader(id_, fragment.get());

        glLinkProgram(id_);

        glDetachShader(id_, vertex.get());
        glDetachShader(id_, fragment.get());

        GLint linked{};
        glGetProgramiv(id_, GL_LINK_STATUS, &linked);

        if (linked == GL_TRUE) {
            return;
        }

        GLint log_length{};
        glGetProgramiv(id_, GL_INFO_LOG_LENGTH, &log_length);

        std::string log;

        if (log_length > 0) {
            log.resize(static_cast<std::string::size_type>(log_length));
            glGetProgramInfoLog(id_, log_length, nullptr, log.data());
        }

        throw std::runtime_error("Shader program linking failed:\n" + log);
    }

    void ShaderProgram::use() const {
        glUseProgram(id_);
    }

    GLint ShaderProgram::uniform_location(const char *name) const {
        const GLint location = glGetUniformLocation(id_, name);

        if (location == -1) {
            throw std::runtime_error(std::string("Uniform not found: ") + name);
        }

        return location;
    }
}
