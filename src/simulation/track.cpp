#include "simulation/track.hpp"

#include <cmath>

namespace apex::simulation {
    bool is_on_track(
        const math::Vec2 &position, const TrackSettings &settings) {
        const double distance = std::hypot(position.x, position.y);
        const double half_width = settings.road_width * 0.5;

        const double inner_radius =
                settings.centerline_radius - half_width;
        const double outer_radius =
                settings.centerline_radius + half_width;

        return distance >= inner_radius &&
               distance <= outer_radius;
    }
}
