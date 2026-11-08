#include "graphics/ground_mesh.hpp"

#include <array>
#include <cmath>
#include <stdexcept>


namespace apex::graphics {
    namespace {
        struct Vertex {
            float x;
            float y;
        };
    }

    GroundMesh::GroundMesh(float half_extent) {
        if (!std::isfinite(half_extent) || half_extent <= 0.0F) {
            throw std::invalid_argument("Ground extent must be finite and positive");
        }

        const std::array vertices{
            Vertex{-half_extent, -half_extent},
            Vertex{+half_extent, -half_extent},
            Vertex{-half_extent, +half_extent},
            Vertex{+half_extent, +half_extent},
        };

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create ground mesh resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(sizeof(vertices)), vertices.data(),GL_STATIC_DRAW);

        glVertexAttribPointer(0, 2,GL_FLOAT,GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    GroundMesh::~GroundMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void GroundMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
}
