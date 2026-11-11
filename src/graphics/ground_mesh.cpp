#include "graphics/ground_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

#include <glm/geometric.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "simulation/race.hpp"

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 normal;
            float water_depth;
        };

        float smooth_step(float start, float end, float value) {
            const float t = std::clamp((value - start) / (end - start), 0.0F, 1.0F);
            return t * t * (3.0F - 2.0F * t);
        }

        float landscape(float x, float y) {
            struct Hill { float x, y, width, depth, height; };
            constexpr Hill hills[]{
                { 16.2F,  1.7F, 4.0F, 9.0F, 3.4F},
                { 13.2F, 12.9F, 6.1F, 4.1F, 2.9F},
                {  7.1F, 16.9F, 4.9F, 4.0F, 4.15F},
                { -2.6F, 14.4F, 5.1F, 4.8F, 3.2F},
                { -7.8F, 15.9F, 4.7F, 4.2F, 2.60F},
                {-14.8F,  4.3F, 4.0F, 7.0F, 2.7F},
                { -7.0F,-11.7F, 9.2F, 4.1F, 2.0F},
                { -4.6F,  2.8F, 3.9F, 3.6F, 1.35F},
                {  3.0F,  2.0F, 3.4F, 3.0F, 1.05F},
            };
            float height = 0.0F;
            for (const auto &hill : hills) {
                const float along = (x - hill.x) / hill.width;
                const float across = (y - hill.y) / hill.depth;
                height += hill.height * std::exp(-along * along - across * across);
            }
            const float ridges = std::sin(.65F*x + .22F*y + .9F*std::sin(.41F*y))
                                 * std::sin(.52F*y - .14F*x);
            const float rock_beds = std::cos(1.17F*x - .78F*y + .85F*std::sin(.63F*x + .41F*y));
            return height * (.96F + .085F*ridges + .034F*rock_beds);
        }

        float reservoir_basin(float x, float y) {
            const float along = ((x + .75F) * .96F - (y + .30F) * .28F) / 4.65F;
            const float across = ((x + .75F) * .28F + (y + .30F) * .96F) / 2.35F;
            const float angle = std::atan2(across, along);
            const float shore = 1.0F + .09F*std::sin(3.0F*angle + .7F) + .06F*std::sin(5.0F*angle);
            const float radius = std::hypot(along, across) / shore;
            return 1.0F - smooth_step(.65F, 1.17F, radius);
        }
    }

    GroundMesh::GroundMesh(float half_extent, const simulation::TrackSettings *track, const GroundMesh *reservoir_bed)
        : half_extent_{half_extent} {
        if (!std::isfinite(half_extent) || half_extent <= 0.0F) {
            throw std::invalid_argument("Ground extent must be finite and positive");
        }

        const auto *path = track && track->road_width >= 0.46 ? track->path.get() : nullptr;
        segments_ = path ? 352 : reservoir_bed ? 128 : 1;
        spacing_ = half_extent_ / segments_ * 2.0F;
        const int stride = segments_ + 1;
        const auto index = [stride](int x, int y) { return y * stride + x; };
        heights_.resize(stride * stride);

        if (path) {
            std::vector<glm::vec2> centerline;
            const double length = path->length();
            const auto paddock = path->sample_at(simulation::start_line_distance);
            const float verge = static_cast<float>(track->road_width * .5 + simulation::runoff_width) + 1.13F;
            for (double distance = 0; distance < length; distance += .22) {
                const auto sample = path->sample_at(distance);
                centerline.emplace_back(sample.position.x, sample.position.y);
            }

            for (int row = 0; row < stride; ++row) {
                const float y = row * spacing_ - half_extent_;
                for (int column = 0; column < stride; ++column) {
                    const float x = column * spacing_ - half_extent_;
                    float closest_squared = (verge + 2.2F) * (verge + 2.2F);
                    for (const auto &point : centerline) {
                        const float dx = point.x - x;
                        const float dy = point.y - y;
                        closest_squared = std::min(closest_squared, dx*dx + dy*dy);
                    }
                    const float track_blend = smooth_step(verge, verge + 2.2F, std::sqrt(closest_squared));
                    const double dx = x - paddock.position.x;
                    const double dy = y - paddock.position.y;
                    const double along = dx*paddock.tangent.x + dy*paddock.tangent.y;
                    const double across = dy*paddock.tangent.x - dx*paddock.tangent.y;
                    const float outside_paddock = std::max({-5.2 - along, along - 3.4, std::abs(across) - 2.05});
                    const float basin = reservoir_basin(x, y);
                    const float relief = std::lerp(landscape(x,y), -.66F, basin);
                    heights_[index(column,row)] = relief * track_blend
                                                * smooth_step(.25F,1.7F,outside_paddock);
                }
            }
        }

        std::vector<Vertex> vertices;
        vertices.reserve(heights_.size());
        std::vector<GLuint> indices;
        indices.reserve(segments_ * segments_ * 6);
        for (int row = 0; row < stride; ++row) {
            for (int column = 0; column < stride; ++column) {
                const float dx = heights_[index(std::min(column+1,segments_),row)] -
                                 heights_[index(std::max(column-1,0),row)];
                const float dy = heights_[index(column,std::min(row+1,segments_))] -
                                 heights_[index(column,std::max(row-1,0))];
                const float x = column*spacing_-half_extent_;
                const float y = row*spacing_-half_extent_;
                const float depth = reservoir_bed ? reservoir_level - reservoir_bed->height_at(x,y) : 0.0F;
                vertices.push_back({{x,y,heights_[index(column,row)]},
                                    glm::normalize(glm::vec3{-dx,-dy,2.0F*spacing_}),depth});
                if (row < segments_ && column < segments_) {
                    const auto a = static_cast<GLuint>(index(column,row));
                    const auto b = a + 1;
                    const auto c = a + stride;
                    const auto d = c + 1;
                    indices.insert(indices.end(), {a,b,c,b,d,c});
                }
            }
        }

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &index_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0 || index_buffer_ == 0) {
            glDeleteBuffers(1, &index_buffer_);
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);
            throw std::runtime_error("Failed to create ground mesh resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, normal)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(3);
        if (reservoir_bed) {
            glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                                  reinterpret_cast<const void *>(offsetof(Vertex, water_depth)));
            glEnableVertexAttribArray(5);
        }
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLuint), indices.data(), GL_STATIC_DRAW);
        index_count_ = static_cast<GLsizei>(indices.size());
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    GroundMesh::~GroundMesh() {
        glDeleteBuffers(1, &index_buffer_);
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    float GroundMesh::height_at(float x, float y) const {
        const float grid_x = std::clamp((x + half_extent_) / spacing_, 0.0F, static_cast<float>(segments_));
        const float grid_y = std::clamp((y + half_extent_) / spacing_, 0.0F, static_cast<float>(segments_));
        const int column = std::min(static_cast<int>(grid_x), segments_-1);
        const int row = std::min(static_cast<int>(grid_y), segments_-1);
        const float u = grid_x - column, v = grid_y - row;
        const auto cell = row * (segments_ + 1) + column;
        const float a = heights_[cell], b = heights_[cell+1];
        const float c = heights_[cell+segments_+1], d = heights_[cell+segments_+2];
        // Match the diagonal used by the mesh triangles so tree trunks meet the surface.
        return u + v <= 1.0F ? a + u*(b-a) + v*(c-a) : d + (1-u)*(c-d) + (1-v)*(b-d);
    }

    void GroundMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
    }
}
