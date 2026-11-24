#pragma once

#include <cstddef>
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

    struct TrackElevationPoint {
        double lap_fraction{};
        double height{};
    };

    struct TrackElevation {
        double height{};
        double grade{};
    };

    class TrackPath {
    public:
        explicit TrackPath(std::vector<math::Vec2> points,
                           std::vector<TrackElevationPoint> elevation = {});

        [[nodiscard]] double distance_at_point(std::size_t index) const noexcept;
        [[nodiscard]] TrackElevation elevation_at(double distance_along_track) const;
        [[nodiscard]] bool has_elevation() const noexcept { return !elevation_.empty(); }

        [[nodiscard]] std::span<const math::Vec2> points() const noexcept;

        [[nodiscard]] double length() const noexcept;

        [[nodiscard]] TrackSample sample_at(double distance_along_track) const;
        [[nodiscard]] double bend_at(double distance_along_track) const;

        [[nodiscard]] TrackProjection project(const math::Vec2 &position) const;

    private:
        std::vector<math::Vec2> points_;
        std::vector<double> cumulative_lengths_;
        std::vector<TrackElevationPoint> elevation_;
    };
}
