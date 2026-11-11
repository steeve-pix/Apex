#include "graphics/tree_mesh.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include <numbers>
#include <stdexcept>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 color;
            glm::vec3 normal{};
        };

        void append_quad(std::vector<Vertex> &vertices, glm::vec3 a, glm::vec3 b, glm::vec3 c,
                         glm::vec3 d, glm::vec3 color) {
            vertices.insert(vertices.end(), {{a,color},{b,color},{c,color},{a,color},{c,color},{d,color}});
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
            !std::isfinite(settings.crown_height_scale) ||
            settings.trunk_height <= 0.0F ||
            settings.trunk_radius <= 0.0F ||
            settings.crown_radius <= 0.0F ||
            settings.crown_height_scale <= 0.0F) {
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
        const float crown_height = radius * settings.crown_height_scale;
        const float crown_center = settings.trunk_height + crown_height * 0.5F;
        const float crown_top = crown_center + crown_height;

        if (!std::isfinite(crown_top)) {
            throw std::invalid_argument("Tree height is too large");
        }

        const glm::vec3 bark{.29F,.17F,.10F};
        const float angle_step = 2.0F * std::numbers::pi_v<float> / settings.segment_count;
        std::vector<Vertex> vertices;
        vertices.reserve(static_cast<std::size_t>(settings.segment_count) * (settings.branching ? 178 : 42));

        const auto trunk = [&](glm::vec3 base, glm::vec3 tip, float root_radius, float tip_radius, int sides) {
            const auto axis = glm::normalize(tip - base);
            const auto right = glm::normalize(glm::cross(axis, glm::vec3{0,1,0}));
            const auto front = glm::cross(right, axis);
            const auto radial = [&](int segment) {
                const float angle = 2.0F * std::numbers::pi_v<float> * segment / sides;
                return right * std::cos(angle) + front * std::sin(angle);
            };
            for (int segment = 0; segment < sides; ++segment) {
                const auto a = radial(segment);
                const auto b = radial(segment + 1);
                const float grain = .87F + .13F * std::abs(a.x);
                append_quad(vertices, base + a*root_radius, base + b*root_radius,
                            tip + b*tip_radius, tip + a*tip_radius, bark*grain);
            }
        };

        const auto crown = [&](glm::vec3 center, float width, float height, float phase, glm::vec3 leaf) {
            const auto vertex = [&](float level, float angle) {
                float profile = std::sqrt(1.0F - level*level);
                if (settings.conifer) {
                    profile *= .82F - .55F * level;
                }
                const float lobes = 1.0F + .17F*std::sin(angle*3.0F + level*3.8F + phase)
                                    + .095F*std::cos(angle*5.0F - level*4.6F + phase)
                                    + .044F*std::sin(angle*8.0F + level*7.2F - phase);
                auto local = ring_point(width * profile * lobes, height * level, angle);
                local.x += width * level * .18F;
                const auto normal = glm::normalize(glm::vec3{
                    local.x / width, local.y / width, local.z * width / (height*height)
                });
                const float grain = .92F + .08F*std::sin((local.x + local.y*.63F)*17.0F/width + phase);
                const auto shade = glm::vec3{.085F,.15F,.085F};
                return Vertex{center + local, (shade + (leaf - shade) * (.5F + .5F*level)) * grain, normal};
            };
            constexpr float rings[]{-1.0F,-.86F,-.62F,-.31F,0.0F,.31F,.62F,.86F,1.0F};
            for (int segment = 0; segment < settings.segment_count; ++segment) {
                const float start = segment * angle_step;
                const float end = (segment + 1) * angle_step;
                for (std::size_t ring = 1; ring < std::size(rings); ++ring) {
                    const auto a = vertex(rings[ring-1], start);
                    const auto b = vertex(rings[ring-1], end);
                    const auto c = vertex(rings[ring], end);
                    const auto d = vertex(rings[ring], start);
                    if (ring > 1) vertices.insert(vertices.end(), {a,b,c});
                    if (ring + 1 < std::size(rings)) vertices.insert(vertices.end(), {a,c,d});
                }
            }
        };

        const glm::vec3 branch_base{0,0,settings.trunk_height*.64F};
        const glm::vec3 trunk_knee{radius*.069F,-radius*.041F,settings.trunk_height*.53F};
        const glm::vec3 trunk_top{-radius*.038F,radius*.08F,settings.trunk_height};
        trunk({0,0,-radius*.035F},trunk_knee,settings.trunk_radius*1.16F,settings.trunk_radius*.78F,settings.segment_count);
        trunk(trunk_knee,trunk_top,settings.trunk_radius*.78F,settings.trunk_radius*.49F,settings.segment_count);
        if (settings.conifer) {
            constexpr glm::vec3 needles[]{
                {.23F,.31F,.18F}, {.27F,.36F,.21F}, {.32F,.40F,.23F}
            };
            for (int tier = 0; tier < 3; ++tier) {
                const float level = static_cast<float>(tier);
                crown({radius*.12F*level,0,settings.trunk_height + crown_height*(.24F + .33F*level)},
                      radius*(1.20F - .24F*level), crown_height*(.46F - .048F*level),
                      level*1.54F, needles[tier]);
            }
        } else if (settings.branching) {
            struct Bough {
                glm::vec3 relative_center;
                float width;
                float height;
                glm::vec3 leaf;
            };
            constexpr Bough boughs[]{
                {{-.58F,-.21F,.12F}, .54F, .42F, {.32F,.42F,.22F}},
                {{-.35F,-.62F,.40F}, .47F, .44F, {.38F,.47F,.24F}},
                {{ .34F,-.60F,.35F}, .46F, .52F, {.34F,.47F,.24F}},
                {{ .60F,-.19F,.57F}, .49F, .48F, {.40F,.49F,.26F}},
                {{ .52F, .39F,.32F}, .55F, .43F, {.32F,.43F,.23F}},
                {{-.07F, .60F,.24F}, .46F, .49F, {.32F,.42F,.22F}},
                {{-.55F, .44F,.60F}, .48F, .49F, {.39F,.48F,.25F}},
                {{ .14F, .17F,.91F}, .56F, .59F, {.45F,.52F,.26F}},
                {{-.27F,-.19F,.79F}, .46F, .51F, {.39F,.49F,.24F}},
            };
            float phase = .3F;
            for (const auto &bough : boughs) {
                const glm::vec3 center = glm::vec3{0,0,settings.trunk_height} + radius * bough.relative_center;
                trunk(branch_base, center, settings.trunk_radius*.64F, settings.trunk_radius*.21F, 6);
                crown(center, radius*bough.width, radius*bough.height, phase, bough.leaf);
                phase += 1.7F;
            }
        } else {
            crown({0,0,crown_center}, radius, crown_height, .4F, {.34F,.44F,.25F});
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

        glVertexAttribPointer(
            3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
            reinterpret_cast<const void *>(offsetof(Vertex, normal))
        );
        glEnableVertexAttribArray(3);

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
