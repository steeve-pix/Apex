#pragma once
#include <glad/gl.h>

namespace apex::graphics {
    class Triangle {
    public:
        Triangle();

        ~Triangle();

        Triangle(const Triangle &) = delete;

        Triangle &operator=(const Triangle &) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
