#pragma once

#include <array>
#include <optional>
#include <span>
#include <vector>

#include <glad/gl.h>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "simulation/track.hpp"

namespace apex::graphics {
    class GroundMesh;

    class RaceScenery {
    public:
        static constexpr int max_floodlights = 24;
        static constexpr int braking_zone_count = 2;

        RaceScenery(const simulation::TrackSettings &track, const GroundMesh &ground);
        ~RaceScenery();
        RaceScenery(const RaceScenery &) = delete;
        RaceScenery &operator=(const RaceScenery &) = delete;
        void draw() const;
        [[nodiscard]] std::span<const glm::vec3> floodlights() const { return floodlights_; }
        [[nodiscard]] std::span<const glm::vec2> floodlight_directions() const { return floodlight_directions_; }
        [[nodiscard]] const std::optional<simulation::TrackSample> &footbridge() const { return footbridge_; }
        [[nodiscard]] const auto &braking_zones() const { return braking_zones_; }

    private:
        GLuint array_{};
        GLuint buffer_{};
        GLsizei count_{};
        std::vector<glm::vec3> floodlights_;
        std::vector<glm::vec2> floodlight_directions_;
        std::optional<simulation::TrackSample> footbridge_;
        std::array<glm::vec2, braking_zone_count> braking_zones_{};
    };
}
