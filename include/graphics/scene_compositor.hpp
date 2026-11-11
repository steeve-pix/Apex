#pragma once

#include <glad/gl.h>

#include "graphics/shader_program.hpp"

namespace apex::graphics {
    class SceneCompositor {
    public:
        SceneCompositor();
        ~SceneCompositor();

        SceneCompositor(const SceneCompositor &) = delete;
        SceneCompositor &operator=(const SceneCompositor &) = delete;

        void begin(int width, int height);
        void present(bool grade, bool overview) const;

    private:
        ShaderProgram program_;
        GLuint array_{};
        GLuint scene_{};
        GLuint resolve_{};
        GLuint color_{};
        GLuint depth_{};
        GLuint image_{};
        GLint grade_{};
        GLint overview_{};
        GLint samples_{};
        int width_{};
        int height_{};
    };
}
