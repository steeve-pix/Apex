#include "graphics/shader.hpp"

#include <stdexcept>
#include <string>

namespace apex::graphics {
    Shader::Shader(GLenum type) : id_{glCreateShader(type)} {
        if (id_ == 0) {
            throw std::runtime_error("Failed to create shader");
        }
    }

    Shader::~Shader() {
        glDeleteShader(id_);
    }

    void Shader::compile(const char *source) const {
        glShaderSource(id_, 1, &source, nullptr);
        glCompileShader(id_);

        GLint compiled{};
        glGetShaderiv(id_,GL_COMPILE_STATUS, &compiled);

        if (compiled == GL_TRUE) {
            return;
        }

        GLint log_length{};
        glGetShaderiv(id_,GL_INFO_LOG_LENGTH, &log_length);

        std::string log;

        if (log_length > 0) {
            log.resize(static_cast<std::string::size_type>(log_length));

            glGetShaderInfoLog(id_, log_length, nullptr, log.data());
        }

        throw std::runtime_error("Shader compilation failed:\n" + log);
    }

    [[nodiscard]] GLuint Shader::get() const noexcept {
        return id_;
    }
}
