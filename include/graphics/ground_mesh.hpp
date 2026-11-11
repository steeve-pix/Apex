#pragma once

#include <vector>

#include <glad/gl.h>

namespace apex::simulation { struct TrackSettings; }

namespace apex::graphics {
    class GroundMesh {
    public:
        static constexpr float reservoir_level = -.16F;

        explicit GroundMesh(float half_extent = 3.0F, const simulation::TrackSettings *track = nullptr,
                            const GroundMesh *reservoir_bed = nullptr);

        ~GroundMesh();

        GroundMesh(const GroundMesh &) = delete;

        GroundMesh &operator=(const GroundMesh &) = delete;

        void draw() const;
        [[nodiscard]] float height_at(float x, float y) const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLuint index_buffer_{};
        GLsizei index_count_{};
        int segments_{1};
        float half_extent_{};
        float spacing_{};
        std::vector<float> heights_;
    };
}
