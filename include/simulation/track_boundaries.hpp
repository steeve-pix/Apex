#pragma once

#include <vector>

#include "math/vec2.hpp"
#include "simulation/track_path.hpp"

namespace apex::simulation {
    struct TrackBoundaries {
        std::vector<math::Vec2> left;
        std::vector<math::Vec2> right;
    };

    [[nodiscard]] TrackBoundaries make_track_boundaries(const TrackPath &path, double half_width);
}
