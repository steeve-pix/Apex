#pragma once
#include "track.hpp"
#include "math/vec2.hpp"

namespace apex::simulation {
    struct RayReading {
        double distance{};
        bool hit{};
    };

    [[nodiscard]] RayReading cast_track_ray(const math::Vec2 &origin, double angle, const TrackSettings &track,
                                            double max_distance);
}
