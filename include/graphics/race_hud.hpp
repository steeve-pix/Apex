#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string_view>

#include <glad/gl.h>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include "graphics/shader_program.hpp"
#include "simulation/race.hpp"

namespace apex::graphics {
    struct TrainingTelemetry;

    struct DriverNameplate {
        glm::vec2 anchor;
        std::string_view name;
        float depth;
        bool focused;
        glm::vec3 livery;
        bool eliminated{};
    };

    struct BroadcastDriver {
        std::size_t slot;
        std::string_view name;
        glm::vec3 livery;
        math::Vec2 position;
        double progress;
        std::optional<double> lap_time;
        bool eliminated;
    };

    class RaceHud {
    public:
        RaceHud();
        ~RaceHud();
        RaceHud(const RaceHud &) = delete;
        RaceHud &operator=(const RaceHud &) = delete;

        void draw_sky(const glm::mat4 &inverse_view_projection, glm::vec3 eye, glm::vec3 sun, bool blue_hour) const;
        void draw(int width, int height, const simulation::RaceStatus &status,
                  const simulation::TrackPath &path, const simulation::VehicleState &player,
                  std::span<const simulation::VehicleState> opponents, bool circuit_overview,
                  std::string_view focused_driver, std::span<const DriverNameplate> nameplates,
                  const TrainingTelemetry *training, std::string_view trackside_name,
                  std::span<const BroadcastDriver> standings, std::size_t focused_slot) const;

    private:
        ShaderProgram program_;
        GLuint sky_array_{};
        GLuint array_{};
        GLuint buffer_{};
        GLuint font_texture_{};
        GLint sky_{};
        GLint inverse_view_projection_{};
        GLint eye_{};
        GLint sun_{};
        GLint blue_hour_{};
    };
}
