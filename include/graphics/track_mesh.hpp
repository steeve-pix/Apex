#pragma once

#include <glad/gl.h>

#include "simulation/track.hpp"

namespace apex::graphics {
    class TrackMesh {
    public:
        explicit TrackMesh(const simulation::TrackSettings &settings, int segment_count = 128);

        ~TrackMesh();

        TrackMesh(const TrackMesh &) = delete;

        TrackMesh &operator=(const TrackMesh &) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
