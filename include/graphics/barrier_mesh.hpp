#pragma once

#include <glad/gl.h>

#include "simulation/track.hpp"

namespace apex::graphics {
    struct BarrierSettings {
        float height{0.04F};
        float thickness{0.02F};
        float clearance{0.035F};
        int segment_count{128};
    };

    class BarrierMesh {
    public:
        explicit BarrierMesh(
            const simulation::TrackSettings &track,
            const BarrierSettings &settings = {}
        );

        ~BarrierMesh();

        BarrierMesh(const BarrierMesh &) = delete;

        BarrierMesh &operator=(const BarrierMesh &) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
