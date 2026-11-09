#include "graphics/tree_mesh.hpp"

#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <glm/vec3.hpp>

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 color;
        };

        void append_triangle(
            std::vector<Vertex> &vertices,
            glm::vec3 a,
            glm::vec3 b,
            glm::vec3 c,
            glm::vec3 color
        ) {
            vertices.insert(vertices.end(), {
                                {a, color},
                                {b, color},
                                {c, color}
                            });
        }

        void append_quad(
            std::vector<Vertex> &vertices,
            glm::vec3 a,
            glm::vec3 b,
            glm::vec3 c,
            glm::vec3 d,
            glm::vec3 color
        ) {
            append_triangle(vertices, a, b, c, color);
            append_triangle(vertices, a, c, d, color);
        }

        glm::vec3 ring_point(float radius, float height, float angle) {
            return {
                radius * std::cos(angle),
                radius * std::sin(angle),
                height
            };
        }
    }

    TreeMesh::TreeMesh(const TreeSettings &settings) {
        if (!std::isfinite(settings.trunk_height) ||
            !std::isfinite(settings.trunk_radius) ||
            !std::isfinite(settings.crown_radius) ||
            settings.trunk_height <= 0.0F ||
            settings.trunk_radius <= 0.0F ||
            settings.crown_radius <= 0.0F) {
            throw std::invalid_argument(
                "Tree dimensions must be finite and positive"
            );
        }

        if (settings.segment_count < 3 || settings.segment_count > 128) {
            throw std::invalid_argument(
                "Tree segment count must be between 3 and 128"
            );
        }

        const float radius = settings.crown_radius;
        const float crown_center = settings.trunk_height + radius * 0.5F;
        const float crown_top = crown_center + radius;

        if (!std::isfinite(crown_top)) {
            throw std::invalid_argument("Tree height is too large");
        }

        const glm::vec3 bark_color{0.32F, 0.18F, 0.08F};
        const glm::vec3 leaf_color{0.22F, 0.48F, 0.10F};

        const glm::vec3 trunk_bottom{0.0F};
        const glm::vec3 trunk_top{0.0F, 0.0F, settings.trunk_height};
        const glm::vec3 bottom_tip{0.0F, 0.0F, crown_center - radius};
        const glm::vec3 top_tip{0.0F, 0.0F, crown_top};

        const float angle_step =
                2.0F * std::numbers::pi_v<float>
                / static_cast<float>(settings.segment_count);

        const float shoulder_radius = radius * std::sqrt(0.75F);

        std::vector<Vertex> vertices;
        vertices.reserve(
            static_cast<std::size_t>(settings.segment_count) * 30
        );

        for (int segment = 0; segment < settings.segment_count; ++segment) {
            const float start = static_cast<float>(segment) * angle_step;
            const float end = static_cast<float>(segment + 1) * angle_step;

            const auto base_a =
                    ring_point(settings.trunk_radius, 0.0F, start);
            const auto base_b =
                    ring_point(settings.trunk_radius, 0.0F, end);
            const auto trunk_a =
                    ring_point(settings.trunk_radius, settings.trunk_height, start);
            const auto trunk_b =
                    ring_point(settings.trunk_radius, settings.trunk_height, end);

            append_quad(
                vertices, base_a, base_b, trunk_b, trunk_a, bark_color
            );
            append_triangle(
                vertices, trunk_bottom, base_b, base_a, bark_color
            );
            append_triangle(
                vertices, trunk_top, trunk_a, trunk_b, bark_color
            );

            const auto lower_a = ring_point(
                shoulder_radius, crown_center - radius * 0.5F, start
            );
            const auto lower_b = ring_point(
                shoulder_radius, crown_center - radius * 0.5F, end
            );
            const auto middle_a = ring_point(radius, crown_center, start);
            const auto middle_b = ring_point(radius, crown_center, end);
            const auto upper_a = ring_point(
                shoulder_radius, crown_center + radius * 0.5F, start
            );
            const auto upper_b = ring_point(
                shoulder_radius, crown_center + radius * 0.5F, end
            );

            append_triangle(
                vertices, bottom_tip, lower_b, lower_a, leaf_color
            );
            append_quad(
                vertices, lower_a, lower_b, middle_b, middle_a, leaf_color
            );
            append_quad(
                vertices, middle_a, middle_b, upper_b, upper_a, leaf_color
            );
            append_triangle(
                vertices, upper_a, upper_b, top_tip, leaf_color
            );
        }

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create tree mesh resources");
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

    TreeMesh::~TreeMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void TreeMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
    }
}
