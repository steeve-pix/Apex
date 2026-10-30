#include "graphics/triangle.hpp"

#include <array>
#include <stdexcept>

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x;
            float y;
        };
    }

    Triangle::Triangle() {
        constexpr std::array vertices{
            Vertex{0.12F, 0.00F},
            Vertex{-0.06F, 0.06F},
            Vertex{-0.06F, -0.06F}
        };

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create Triangle resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                     vertices.data(),
                     GL_STATIC_DRAW
        );

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);

        glEnableVertexAttribArray(0);

        glBindVertexArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    Triangle::~Triangle() {
        glDeleteBuffers(1, &vertex_buffer_);

        glDeleteVertexArrays(1, &vertex_array_);
    }

    void Triangle::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
    }
}
