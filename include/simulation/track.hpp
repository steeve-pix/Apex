#pragma once
#include "math/vec2.hpp"

namespace apex::simulation {
    struct TrackSettings {
        double centerline_radius{0.65};
        double road_width{0.30};
    };

    [[nodiscard]] bool is_on_track(
        const math::Vec2 &position,
        const TrackSettings &settings
    );
}
