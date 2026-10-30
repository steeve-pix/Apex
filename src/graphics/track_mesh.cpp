#include "graphics/track_mesh.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x{};
            float y{};
        };
    }

    TrackMesh::TrackMesh(const simulation::TrackSettings &settings, int segment_count) {
        const double half_width = settings.road_width * 0.5;

        if (!std::isfinite(settings.centerline_radius) || !std::isfinite(settings.road_width) || half_width <= 0.0 ||
            settings.centerline_radius <= half_width) {
            throw std::invalid_argument("Invalid track dimensions");
        }

        constexpr int max_segments =
                std::numeric_limits<GLsizei>::max() / 2 - 1;

        if (segment_count < 3 || segment_count > max_segments) {
            throw std::invalid_argument("Invalid track segment count");
        }

        const double inner_radius =
                settings.centerline_radius - half_width;
        const double outer_radius =
                settings.centerline_radius + half_width;

        std::vector<Vertex> vertices;
        vertices.reserve((static_cast<std::size_t>(segment_count) + 1) * 2);

        for (int i = 0; i <= segment_count; ++i) {
            const double angle =
                    2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / segment_count;

            const double x = std::cos(angle);
            const double y = std::sin(angle);

            vertices.push_back({
                static_cast<float>(outer_radius * x),
                static_cast<float>(outer_radius * y)
            });

            vertices.push_back({
                static_cast<float>(inner_radius * x),
                static_cast<float>(inner_radius * y)
            });
        }

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create track mesh resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
            GL_STATIC_DRAW);

        glVertexAttribPointer(0, 2,GL_FLOAT,GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    TrackMesh::~TrackMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void TrackMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, vertex_count_);
    }
}
