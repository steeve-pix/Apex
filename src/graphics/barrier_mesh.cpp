#include "graphics/barrier_mesh.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x;
            float y;
            float z;
        };

        Vertex ring_vertex(double radius, double angle, float height) {
            return {
                static_cast<float>(radius * std::cos(angle)),
                static_cast<float>(radius * std::sin(angle)),
                height
            };
        }

        void append_quad(std::vector<Vertex> &vertice, Vertex a, Vertex b, Vertex c, Vertex d) {
            vertice.insert(vertice.end(), {a, b, c, a, c, d});
        }

        void append_ring(std::vector<Vertex> &vertices, double inner_radius, double outer_radius,
                         const BarrierSettings &settings) {
            const double angle_step =
                    2.0 * std::numbers::pi / settings.segment_count;

            for (int segment = 0; segment < settings.segment_count; ++segment) {
                const double start = segment * angle_step;
                const double end = (segment + 1) * angle_step;

                const auto inner_start =
                        ring_vertex(inner_radius, start, 0.0F);
                const auto inner_end =
                        ring_vertex(inner_radius, end, 0.0F);
                const auto outer_start =
                        ring_vertex(outer_radius, start, 0.0F);
                const auto outer_end =
                        ring_vertex(outer_radius, end, 0.0F);

                const auto inner_start_top =
                        ring_vertex(inner_radius, start, settings.height);
                const auto inner_end_top =
                        ring_vertex(inner_radius, end, settings.height);
                const auto outer_start_top =
                        ring_vertex(outer_radius, start, settings.height);
                const auto outer_end_top =
                        ring_vertex(outer_radius, end, settings.height);

                append_quad(vertices, inner_start_top, outer_start_top, outer_end_top, inner_end_top);
                append_quad(vertices, outer_start, outer_end, outer_end_top, outer_start_top);
                append_quad(vertices, inner_start, inner_start_top, inner_end_top, inner_end);
            }
        }
    }

    BarrierMesh::BarrierMesh(const simulation::TrackSettings &track, const BarrierSettings &settings) {
        const double half_width = track.road_width * 0.5;

        if (!std::isfinite(track.centerline_radius) || !std::isfinite(track.road_width) || half_width <= 0.0 || track.
            centerline_radius <= half_width) {
            throw std::invalid_argument("Invalid track dimensions");
        }

        if (!std::isfinite(settings.height) || !std::isfinite(settings.thickness) || !std::isfinite(settings.clearance)
            || settings.height <= 0.0F || settings.thickness <= 0.0F || settings.clearance < 0.0F) {
            throw std::invalid_argument("Invalid barrier dimensions");
        }

        constexpr int vertices_per_segment = 36;
        constexpr int max_segments =
                std::numeric_limits<GLsizei>::max() / vertices_per_segment;

        if (settings.segment_count < 3 || settings.segment_count > max_segments) {
            throw std::invalid_argument("Invalid barrier segment count");
        }

        const double inner_outer_radius =
                track.centerline_radius - half_width - settings.clearance;

        const double inner_inner_radius =
                inner_outer_radius - settings.thickness;

        const double outer_inner_radius =
                track.centerline_radius + half_width + settings.clearance;

        const double outer_outer_radius =
                outer_inner_radius + settings.thickness;

        if (inner_inner_radius <= 0.0 ||
            !std::isfinite(outer_outer_radius) ||
            outer_outer_radius > std::numeric_limits<float>::max()) {
            throw std::invalid_argument("Barriers do not fit the track");
        }

        std::vector<Vertex> vertices;
        vertices.reserve(static_cast<std::size_t>(settings.segment_count) * vertices_per_segment);

        append_ring(vertices, inner_inner_radius, inner_outer_radius, settings);
        append_ring(vertices, outer_inner_radius, outer_outer_radius, settings);

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create barrier mesh resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);

        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    BarrierMesh::~BarrierMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void BarrierMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
    }
}
