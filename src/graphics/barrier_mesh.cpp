#include "graphics/barrier_mesh.hpp"

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <vector>
#include <span>

#include "simulation/track_boundaries.hpp"

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

        Vertex boundary_vertex(const math::Vec2 &point, float height) {
            constexpr auto maximum_coordinate =
                    static_cast<double>(std::numeric_limits<float>::max());

            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                std::abs(point.x) > maximum_coordinate ||
                std::abs(point.y) > maximum_coordinate) {
                throw std::invalid_argument(
                    "Barrier coordinates exceed the rendering range"
                );
            }

            return {
                static_cast<float>(point.x),
                static_cast<float>(point.y),
                height
            };
        }

        void append_boundary_wall(std::vector<Vertex> &vertices, std::span<const math::Vec2> near_edge,
                                  std::span<const math::Vec2> far_edge, float height) {
            for (std::size_t index = 0; index < near_edge.size(); ++index) {
                const std::size_t next = (index + 1) % near_edge.size();

                const auto a = boundary_vertex(near_edge[index], 0.0F);
                const auto b = boundary_vertex(far_edge[index], 0.0F);
                const auto c = boundary_vertex(far_edge[next], 0.0F);
                const auto d = boundary_vertex(near_edge[next], 0.0F);

                const Vertex a_top{a.x, a.y, height};
                const Vertex b_top{b.x, b.y, height};
                const Vertex c_top{c.x, c.y, height};
                const Vertex d_top{d.x, d.y, height};

                append_quad(vertices, a_top, b_top, c_top, d_top);
                append_quad(vertices, b, c, c_top, b_top);
                append_quad(vertices, a, a_top, d_top, d);
            }
        }
    }

    BarrierMesh::BarrierMesh(const simulation::TrackSettings &track, const BarrierSettings &settings) {
        const double half_width = track.road_width * 0.5;

        if (!std::isfinite(track.road_width) || half_width <= 0.0) {
            throw std::invalid_argument("Invalid track width");
        }

        if (!track.path &&
            (!std::isfinite(track.centerline_radius) ||
             track.centerline_radius <= half_width)) {
            throw std::invalid_argument("Invalid circular track radius");
        }

        if (!std::isfinite(settings.height) || !std::isfinite(settings.thickness) || !std::isfinite(settings.clearance)
            || settings.height <= 0.0F || settings.thickness <= 0.0F || settings.clearance < 0.0F) {
            throw std::invalid_argument("Invalid barrier dimensions");
        }

        constexpr std::size_t vertices_per_segment = 36;

        const std::size_t segment_count = track.path
                                              ? track.path->points().size()
                                              : static_cast<std::size_t>(settings.segment_count);

        constexpr auto maximum_segments =
                static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) / vertices_per_segment;

        constexpr auto maximum_buffer_segments =
                static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max()) / (
                    vertices_per_segment * sizeof(Vertex));

        if (segment_count < 3 || segment_count > maximum_segments || segment_count > maximum_buffer_segments) {
            throw std::invalid_argument("Invalid barrier segment count");
        }

        std::vector<Vertex> vertices;
        vertices.reserve(segment_count * vertices_per_segment);

        if (track.path) {
            const double near_offset =
                    half_width + settings.clearance;

            const double far_offset =
                    near_offset + settings.thickness;

            const auto near_edges = simulation::make_track_boundaries(*track.path, near_offset);

            const auto far_edges = simulation::make_track_boundaries(*track.path, far_offset);

            append_boundary_wall(vertices, near_edges.left, far_edges.left, settings.height);
            append_boundary_wall(vertices, near_edges.right, far_edges.right, settings.height);
        } else {
            const double inner_outer_radius =
                    track.centerline_radius - half_width - settings.clearance;
            const double inner_inner_radius =
                    inner_outer_radius - settings.thickness;
            const double outer_inner_radius =
                    track.centerline_radius + half_width + settings.clearance;
            const double outer_outer_radius =
                    outer_inner_radius + settings.thickness;

            if (inner_inner_radius <= 0.0 || !std::isfinite(outer_outer_radius) || outer_outer_radius >
                std::numeric_limits<float>::max()) {
                throw std::invalid_argument("Barriers do not fit the track");
            }

            append_ring(vertices, inner_inner_radius, inner_outer_radius, settings);
            append_ring(vertices, outer_inner_radius, outer_outer_radius, settings);
        }

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
