#include "graphics/shadow_map.hpp"

#include <array>
#include <stdexcept>

namespace apex::graphics {
    ShadowMap::ShadowMap(int resolution)
        : resolution_{resolution} {
        GLint maximum_size{};
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximum_size);

        if (resolution <= 0 || resolution > maximum_size) {
            throw std::invalid_argument("Shadow resolution exceeds the supported range");
        }

        glGenFramebuffers(1, &framebuffer_);
        glGenTextures(1, &depth_texture_);

        if (framebuffer_ == 0 || depth_texture_ == 0) {
            glDeleteFramebuffers(1, &framebuffer_);
            glDeleteTextures(1, &depth_texture_);

            throw std::runtime_error("Failed to create shadow map resources");
        }

        GLint previous_texture{};
        GLint previous_draw_framebuffer{};
        GLint previous_read_framebuffer{};

        glGetIntegerv(GL_TEXTURE_BINDING_2D, &previous_texture);
        glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previous_draw_framebuffer);
        glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous_read_framebuffer);

        glBindTexture(GL_TEXTURE_2D, depth_texture_);

        glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT24, resolution_, resolution_, 0,GL_DEPTH_COMPONENT,GL_FLOAT,
                     nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

        const std::array border_color{1.0F, 1.0F, 1.0F, 1.0F};
        glTexParameterfv(GL_TEXTURE_2D,GL_TEXTURE_BORDER_COLOR, border_color.data());

        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
        glFramebufferTexture2D(GL_FRAMEBUFFER,GL_DEPTH_ATTACHMENT,GL_TEXTURE_2D, depth_texture_, 0);

        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);

        const GLenum status =
                glCheckFramebufferStatus(GL_FRAMEBUFFER);

        glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(previous_texture));
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(previous_draw_framebuffer));
        glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(previous_read_framebuffer));

        if (status != GL_FRAMEBUFFER_COMPLETE) {
            glDeleteFramebuffers(1, &framebuffer_);
            glDeleteTextures(1, &depth_texture_);

            throw std::runtime_error("Shadow framebuffer is incomplete");
        }
    }

    ShadowMap::~ShadowMap() {
        glDeleteFramebuffers(1, &framebuffer_);
        glDeleteTextures(1, &depth_texture_);
    }

    void ShadowMap::bind_for_writing() const {
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
        glViewport(0, 0, resolution_, resolution_);

        glDepthMask(GL_TRUE);
        glClearDepth(1.0);
        glClear(GL_DEPTH_BUFFER_BIT);
    }

    void ShadowMap::bind_texture(GLuint texture_unit) const {
        GLint maximum_units{};
        glGetIntegerv(
            GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS,
            &maximum_units
        );

        if (texture_unit >= static_cast<GLuint>(maximum_units)) {
            throw std::invalid_argument("Shadow texture unit exceeds the supported range");
        }

        glActiveTexture(GL_TEXTURE0 + texture_unit);
        glBindTexture(GL_TEXTURE_2D, depth_texture_);
    }
}
