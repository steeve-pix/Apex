#include "graphics/car_mesh.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <numbers>

#include <glm/vec3.hpp>

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 color;
        };

        struct BodySection {
            float x;
            float half_width;
            float bottom;
            float top;
        };

        void append_quad(std::vector<Vertex> &vertices, glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d,
                         glm::vec3 color) {
            vertices.insert(vertices.end(), {
                                {a, color}, {b, color}, {c, color},
                                {a, color}, {c, color}, {d, color}
                            });
        }

        void append_body(std::vector<Vertex> &vertices, const BodySection &rear, const BodySection &front,
                         glm::vec3 color) {
            const std::array corners{
                glm::vec3{rear.x, -rear.half_width, rear.bottom},
                glm::vec3{front.x, -front.half_width, front.bottom},
                glm::vec3{front.x, front.half_width, front.bottom},
                glm::vec3{rear.x, rear.half_width, rear.bottom},
                glm::vec3{rear.x, -rear.half_width, rear.top},
                glm::vec3{front.x, -front.half_width, front.top},
                glm::vec3{front.x, front.half_width, front.top},
                glm::vec3{rear.x, rear.half_width, rear.top}
            };

            // Bottom and top.
            append_quad(
                vertices, corners[0], corners[3],
                corners[2], corners[1], color
            );
            append_quad(
                vertices, corners[4], corners[5],
                corners[6], corners[7], color
            );

            // Left and right sides.
            append_quad(
                vertices, corners[0], corners[1],
                corners[5], corners[4], color
            );
            append_quad(
                vertices, corners[3], corners[7],
                corners[6], corners[2], color
            );

            // Rear and front.
            append_quad(
                vertices, corners[0], corners[4],
                corners[7], corners[3], color
            );
            append_quad(
                vertices, corners[1], corners[2],
                corners[6], corners[5], color
            );
        }

        void append_tire(std::vector<Vertex> &vertices, glm::vec3 center, float radius, float width) {
            constexpr int segments = 20;
            constexpr float angle_step =
                    2.0F * std::numbers::pi_v<float> / segments;

            const float half_width = width * 0.5F;

            const glm::vec3 tread_color{0.025F, 0.025F, 0.028F};
            const glm::vec3 sidewall_color{0.06F, 0.06F, 0.065F};

            const auto point_at = [center, radius](float angle, float lateral_offset) {
                return glm::vec3{
                    center.x + radius * std::cos(angle),
                    center.y + lateral_offset,
                    center.z + radius * std::sin(angle)
                };
            };

            const glm::vec3 left_center{center.x, center.y - half_width, center.z};
            const glm::vec3 right_center{center.x, center.y + half_width, center.z};

            for (int segment = 0; segment < segments; ++segment) {
                const float start = segment * angle_step;
                const float end = (segment + 1) * angle_step;

                const auto left_start = point_at(start, -half_width);
                const auto left_end = point_at(end, -half_width);
                const auto right_start = point_at(start, half_width);
                const auto right_end = point_at(end, half_width);

                append_quad(vertices, left_start, right_start, right_end, left_end, tread_color);

                vertices.insert(vertices.end(), {
                                    {left_center, sidewall_color},
                                    {left_start, sidewall_color},
                                    {left_end, sidewall_color},

                                    {right_center, sidewall_color},
                                    {right_end, sidewall_color},
                                    {right_start, sidewall_color}
                                });
            }
        }
    }

    CarMesh::CarMesh() {
        const glm::vec3 body_color{0.85F, 0.06F, 0.04F};

        std::vector<Vertex> vertices;

        const BodySection chassis_rear{
            -0.10F, 0.035F, 0.010F, 0.040F
        };
        const BodySection chassis_front{
            0.04F, 0.025F, 0.010F, 0.040F
        };
        const BodySection nose_tip{
            0.14F, 0.008F, 0.015F, 0.025F
        };

        append_body(vertices, chassis_rear, chassis_front, body_color);
        append_body(vertices, chassis_front, nose_tip, body_color);

        constexpr float axle_offset = 0.10F;
        constexpr float half_axle_width = 0.05F;
        constexpr float tire_radius = 0.025F;
        constexpr float tire_width = 0.018F;

        const std::array wheel_centers{
            glm::vec3{-axle_offset, -half_axle_width, tire_radius},
            glm::vec3{-axle_offset, half_axle_width, tire_radius},
            glm::vec3{axle_offset, -half_axle_width, tire_radius},
            glm::vec3{axle_offset, half_axle_width, tire_radius}
        };

        for (const auto &center: wheel_centers) {
            append_tire(vertices, center, tire_radius, tire_width);
        }

        const glm::vec3 wing_color{0.12F, 0.12F, 0.13F};
        const glm::vec3 cockpit_color{0.015F, 0.020F, 0.025F};

        const BodySection front_wing_rear{0.125F, 0.065F, 0.009F, 0.015F};
        const BodySection front_wing_front{0.155F, 0.065F, 0.009F, 0.015F};

        append_body(vertices, front_wing_rear, front_wing_front, wing_color);
        const BodySection rear_wing_support_rear{-0.120F, 0.008F, 0.025F, 0.060F};
        const BodySection rear_wing_support_front{-0.090F, 0.008F, 0.025F, 0.060F};

        append_body(vertices, rear_wing_support_rear, rear_wing_support_front, body_color);

        const BodySection rear_wing_rear{-0.140F, 0.060F, 0.060F, 0.067F};
        const BodySection rear_wing_front{-0.110F, 0.060F, 0.060F, 0.067F};

        append_body(vertices, rear_wing_rear, rear_wing_front, wing_color);

        const BodySection engine_cover_rear{-0.080F, 0.022F, 0.040F, 0.070F};
        const BodySection engine_cover_front{-0.030F, 0.016F, 0.040F, 0.050F};

        append_body(vertices, engine_cover_rear, engine_cover_front, body_color);

        const BodySection cockpit_rear{-0.025F, 0.014F, 0.0402F, 0.042F};
        const BodySection cockpit_front{0.025F, 0.014F, 0.0402F, 0.042F};

        append_body(vertices, cockpit_rear, cockpit_front, cockpit_color);

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error(
                "Failed to create car mesh resources"
            );
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
            vertices.data(),
            GL_STATIC_DRAW
        );

        glVertexAttribPointer(
            0, 3, GL_FLOAT, GL_FALSE,
            static_cast<GLsizei>(sizeof(Vertex)),
            nullptr
        );
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(
            1, 3, GL_FLOAT, GL_FALSE,
            static_cast<GLsizei>(sizeof(Vertex)),
            reinterpret_cast<const void *>(offsetof(Vertex, color))
        );
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    CarMesh::~CarMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void CarMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
    }
}
