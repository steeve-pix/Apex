#pragma once

#include <glad/gl.h>

namespace apex::graphics {
    class GroundMesh {
    public:
        explicit GroundMesh(float half_extent = 3.0F);

        ~GroundMesh();

        GroundMesh(const GroundMesh &) = delete;

        GroundMesh &operator=(const GroundMesh &) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
    };
}
