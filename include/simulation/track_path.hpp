#pragma once

#include <span>
#include <vector>

#include "math/vec2.hpp"

namespace apex::simulation {
    struct TrackSample {
        math::Vec2 position{};
        math::Vec2 tangent{};
    };

    struct TrackProjection {
        TrackSample sample{};
        double lateral_offset{};
        double distance_along_track{};
    };

    class TrackPath {
    public:
        explicit TrackPath(std::vector<math::Vec2> points);

        [[nodiscard]] std::span<const math::Vec2> points() const noexcept;

        [[nodiscard]] double length() const noexcept;

        [[nodiscard]] TrackSample sample_at(double distance_along_track) const;

        [[nodiscard]] TrackProjection project(const math::Vec2 &position) const;

    private:
        std::vector<math::Vec2> points_;
        std::vector<double> cumulative_lengths_;
    };
}
