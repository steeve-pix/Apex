#pragma once
#include "vehicle.hpp"
#include "math/vec2.hpp"

namespace apex::simulation {
    struct TrackSettings {
        double centerline_radius{0.65};
        double road_width{0.30};
        double off_road_drag{4.0};
    };

    [[nodiscard]] bool is_on_track(
        const math::Vec2 &position,
        const TrackSettings &settings
    );

    [[nodiscard]] VehicleSettings vehicle_settings_at(
        const math::Vec2 &position,
        const VehicleSettings &base_settings,
        const TrackSettings &track
    );
}
