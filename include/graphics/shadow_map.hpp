#pragma once

#include <glad/gl.h>

namespace apex::graphics {
    class ShadowMap {
    public:
        explicit ShadowMap(int resolution = 2048);

        ~ShadowMap();

        ShadowMap(const ShadowMap &) = delete;

        ShadowMap &operator=(const ShadowMap &) = delete;

        void bind_for_writing() const;

        void bind_texture(GLuint texture_unit = 0) const;

    private:
        GLuint framebuffer_{};
        GLuint depth_texture_{};
        int resolution_{};
    };
}
