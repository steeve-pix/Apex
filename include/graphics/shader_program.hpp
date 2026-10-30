#pragma once

#include <glad/gl.h>

namespace apex::graphics {
    class Shader;

    class ShaderProgram {
    public:
        ShaderProgram();
        ~ShaderProgram();

        ShaderProgram(const ShaderProgram&) = delete;
        ShaderProgram& operator=(const ShaderProgram&) = delete;

        void link(const Shader& vertex, const Shader& fragment) const;
        void use() const;

    private:
        GLuint id_;
    };
}
