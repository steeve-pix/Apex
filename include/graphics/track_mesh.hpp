#pragma once

#include <span>

#include <glad/gl.h>

#include "simulation/track.hpp"
#include "simulation/track_boundaries.hpp"

namespace apex::graphics {
    class TrackMesh {
    public:
        explicit TrackMesh(const simulation::TrackSettings &settings, int segment_count = 128);

        explicit TrackMesh(const simulation::TrackBoundaries &boundaries);

        ~TrackMesh();

        TrackMesh(const TrackMesh &) = delete;

        TrackMesh &operator=(const TrackMesh &) = delete;

        void draw() const;

    private:
        struct Vertex {
            float x{};
            float y{};
            float distance_along{};
            float lateral_offset{};
            float bend{};
        };

        void build_from_boundaries(const simulation::TrackBoundaries &boundaries);

        void upload(std::span<const Vertex> vertices);

        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
