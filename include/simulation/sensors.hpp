#pragma once
#include "track.hpp"
#include "math/vec2.hpp"

#include <array>
#include <cstddef>
#include <numbers>

namespace apex::simulation {
    struct RayReading {
        double distance{};
        bool hit{};
    };

    inline constexpr std::size_t sensor_count = 7;

    struct SensorSettings {
        std::array<double, sensor_count> angle_offset{
            -std::numbers::pi_v<double> / 2.0,
            -std::numbers::pi_v<double> / 3.0,
            -std::numbers::pi_v<double> / 6.0,
            0.0,
            std::numbers::pi_v<double> / 6.0,
            std::numbers::pi_v<double> / 3.0,
            std::numbers::pi_v<double> / 2.0,
        };

        double max_distance{1.5};
    };

    using SensorReadings = std::array<RayReading, sensor_count>;

    [[nodiscard]] RayReading cast_track_ray(const math::Vec2 &origin, double angle, const TrackSettings &track,
                                            double max_distance);

    [[nodiscard]] SensorReadings sense_track(const VehicleState &vehicle, const TrackSettings &track,
                                             const SensorSettings &sensors);
}
