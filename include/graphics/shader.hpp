#pragma once

#include <glad/gl.h>

namespace apex::graphics {

    class Shader {
    public:
        explicit Shader(GLenum type);
        ~Shader();

        Shader(const Shader&) = delete;
        Shader& operator=(const Shader&) = delete;

        void compile(const char* source) const;

        [[nodiscard]] GLuint get() const noexcept;

    private:
        GLuint id_;
    };

}