#include "graphics/track_mesh.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace apex::graphics {
    TrackMesh::TrackMesh(const simulation::TrackSettings &settings, int segment_count) {
        const double half_width = settings.road_width * 0.5;

        if (!std::isfinite(settings.centerline_radius) ||
            !std::isfinite(settings.road_width) ||
            half_width <= 0.0 ||
            settings.centerline_radius <= half_width) {
            throw std::invalid_argument("Invalid track dimensions");
        }

        constexpr int maximum_segments =
                std::numeric_limits<GLsizei>::max() / 2 - 1;

        if (segment_count < 3 || segment_count > maximum_segments) {
            throw std::invalid_argument("Invalid track segment count");
        }

        const double inner_radius =
                settings.centerline_radius - half_width;
        const double outer_radius =
                settings.centerline_radius + half_width;

        if (!std::isfinite(outer_radius) ||
            outer_radius > std::numeric_limits<float>::max()) {
            throw std::invalid_argument("Track dimensions exceed the rendering range");
        }

        std::vector<Vertex> vertices;
        vertices.reserve((static_cast<std::size_t>(segment_count) + 1) * 2);

        for (int index = 0; index <= segment_count; ++index) {
            const double angle = 2.0 * std::numbers::pi_v<double> * static_cast<double>(index) / static_cast<double>(
                                     segment_count);

            const double x = std::cos(angle);
            const double y = std::sin(angle);

            vertices.push_back({static_cast<float>(outer_radius * x), static_cast<float>(outer_radius * y)});

            vertices.push_back({static_cast<float>(inner_radius * x), static_cast<float>(inner_radius * y)});
        }

        upload(vertices);
    }

    TrackMesh::TrackMesh(const simulation::TrackBoundaries &boundaries) {
        const std::size_t point_count = boundaries.left.size();

        if (point_count < 3 || boundaries.right.size() != point_count) {
            throw std::invalid_argument("Track boundaries require matching point counts of at least three");
        }

        constexpr auto maximum_points = static_cast<std::size_t>(std::numeric_limits<GLsizei>::max() / 2 - 1);

        if (point_count > maximum_points) {
            throw std::invalid_argument("Track boundaries contain too many points");
        }

        constexpr auto maximum_coordinate =
                static_cast<double>(std::numeric_limits<float>::max());

        std::vector<Vertex> vertices;
        vertices.reserve((point_count + 1) * 2);

        for (std::size_t index = 0; index <= point_count; ++index) {
            const std::size_t point_index = index % point_count;

            for (const auto &point: {boundaries.right[point_index], boundaries.left[point_index]}) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || std::abs(point.x) > maximum_coordinate ||
                    std::abs(point.y) > maximum_coordinate) {
                    throw std::invalid_argument("Track boundary coordinates exceed the rendering range");
                }

                vertices.push_back({static_cast<float>(point.x), static_cast<float>(point.y)});
            }
        }

        upload(vertices);
    }

    void TrackMesh::upload(std::span<const Vertex> vertices) {
        const auto maximum_vertices = static_cast<std::size_t>(std::numeric_limits<GLsizei>::max());
        const auto maximum_buffer_vertices = static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / sizeof(
                                                 Vertex);

        if (vertices.size() > maximum_vertices || vertices.size() > maximum_buffer_vertices) {
            throw std::invalid_argument("Track mesh exceeds GPU buffer limits");
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

        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size_bytes()), vertices.data(),GL_STATIC_DRAW);

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
