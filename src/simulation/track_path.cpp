#include "simulation/track_path.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <algorithm>
#include <iterator>
#include <limits>

namespace apex::simulation {
    TrackPath::TrackPath(std::vector<math::Vec2> points, std::vector<TrackElevationPoint> elevation)
        : points_{std::move(points)}, elevation_{std::move(elevation)} {
        if (!elevation_.empty()) {
            if (elevation_.size() < 2 || elevation_.front().lap_fraction != 0.0
                || elevation_.back().lap_fraction != 1.0 || elevation_.front().height != elevation_.back().height) {
                throw std::invalid_argument("Track elevation must cover the lap and join at the same height");
            }
            double previous_fraction = -1.0;
            for (const auto &point : elevation_) {
                if (!(point.lap_fraction > previous_fraction) || !std::isfinite(point.height)) {
                    throw std::invalid_argument("Track elevation points must be ordered and finite");
                }
                previous_fraction = point.lap_fraction;
            }
        }

        if (points_.size() < 3) {
            throw std::invalid_argument("A closed track requires at least three points");
        }

        for (const auto &point: points_) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                throw std::invalid_argument("Track coordinates must be finite");
            }
        }

        cumulative_lengths_.reserve(points_.size() + 1);
        cumulative_lengths_.push_back(0.0);

        for (std::size_t index = 0; index < points_.size(); ++index) {
            const auto &start = points_[index];
            const auto &end = points_[(index + 1) % points_.size()];

            const double segment_length = std::hypot(
                end.x - start.x,
                end.y - start.y
            );

            if (!std::isfinite(segment_length) || segment_length <= 0.0) {
                throw std::invalid_argument("Track segments must have finite positive lengths");
            }

            const double previous_length = cumulative_lengths_.back();
            const double next_length = previous_length + segment_length;

            if (!std::isfinite(next_length) ||
                next_length <= previous_length) {
                throw std::invalid_argument("Track distances exceed the supported numeric range");
            }

            cumulative_lengths_.push_back(next_length);
        }
    }

    std::span<const math::Vec2> TrackPath::points() const noexcept {
        return {points_.data(), points_.size()};
    }

    double TrackPath::length() const noexcept {
        return cumulative_lengths_.back();
    }

    double TrackPath::distance_at_point(std::size_t index) const noexcept {
        return cumulative_lengths_[index];
    }

    TrackElevation TrackPath::elevation_at(double distance_along_track) const {
        if (elevation_.empty()) {
            return {};
        }
        if (!std::isfinite(distance_along_track)) {
            throw std::invalid_argument("Track elevation distance must be finite");
        }
        double fraction = std::fmod(distance_along_track, length()) / length();
        if (fraction < 0.0) {
            fraction += 1.0;
        }
        if (fraction >= 1.0) {
            fraction = 0.0;
        }
        const auto end = std::ranges::upper_bound(elevation_, fraction, {}, &TrackElevationPoint::lap_fraction);
        const auto start = std::prev(end);
        const double span = end->lap_fraction - start->lap_fraction;
        const double t = (fraction - start->lap_fraction) / span;
        // Ease to zero grade at each crest and valley.
        return {std::lerp(start->height, end->height, t * t * (3.0 - 2.0 * t)),
                (end->height - start->height) * 6.0 * t * (1.0 - t) / (span * length())};
    }

    TrackSample TrackPath::sample_at(double distance_along_track) const {
        if (!std::isfinite(distance_along_track)) {
            throw std::invalid_argument("Track sample distance must be finite");
        }

        const double total_length = length();
        double wrapped_distance = std::fmod(
            distance_along_track,
            total_length
        );

        if (wrapped_distance < 0.0) {
            wrapped_distance += total_length;
        }

        // Adding a tiny negative remainder can round up to total_length.
        if (wrapped_distance >= total_length) {
            wrapped_distance = 0.0;
        }

        const auto segment_end = std::upper_bound(cumulative_lengths_.begin(), cumulative_lengths_.end(),
                                                  wrapped_distance);

        const auto segment_index = static_cast<std::size_t>(
            std::distance(cumulative_lengths_.begin(), segment_end) - 1
        );

        const auto &start = points_[segment_index];
        const auto &end = points_[(segment_index + 1) % points_.size()];

        const double segment_start_distance =
                cumulative_lengths_[segment_index];

        const double segment_distance =
                cumulative_lengths_[segment_index + 1] - segment_start_distance;

        const double fraction =
                (wrapped_distance - segment_start_distance) / segment_distance;

        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double segment_length = std::hypot(dx, dy);

        return TrackSample{
            .position = {std::lerp(start.x, end.x, fraction), std::lerp(start.y, end.y, fraction)},
            .tangent = {dx / segment_length, dy / segment_length}
        };
    }

    double TrackPath::bend_at(double distance_along_track) const {
        constexpr double probe = .58;
        const auto entry = sample_at(distance_along_track - probe).tangent;
        const auto exit = sample_at(distance_along_track + probe).tangent;
        return std::atan2(entry.x * exit.y - entry.y * exit.x,
                          entry.x * exit.x + entry.y * exit.y);
    }

    TrackProjection TrackPath::project(const math::Vec2 &position) const {
        if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
            throw std::invalid_argument("Track projection position must be finite");
        }

        TrackProjection closest{};
        double closest_distance = std::numeric_limits<double>::infinity();

        for (std::size_t index = 0; index < points_.size(); ++index) {
            const auto &start = points_[index];
            const auto &end = points_[(index + 1) % points_.size()];

            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double segment_length = std::hypot(dx, dy);

            const math::Vec2 tangent{
                dx / segment_length,
                dy / segment_length
            };

            const double offset_x = position.x - start.x;
            const double offset_y = position.y - start.y;

            const double along_segment =
                    offset_x * tangent.x + offset_y * tangent.y;

            if (!std::isfinite(along_segment)) {
                throw std::invalid_argument("Projection exceeds the supported numeric range");
            }

            const double fraction = std::clamp(
                along_segment / segment_length, 0.0, 1.0);

            const math::Vec2 candidate{
                std::lerp(start.x, end.x, fraction),
                std::lerp(start.y, end.y, fraction)
            };

            const double distance = std::hypot(position.x - candidate.x, position.y - candidate.y);

            if (distance < closest_distance) {
                closest_distance = distance;
                closest.sample.position = candidate;

                closest.distance_along_track = std::lerp(cumulative_lengths_[index], cumulative_lengths_[index + 1],
                                                         fraction);
            }
        }

        if (!std::isfinite(closest_distance)) {
            throw std::invalid_argument("Projection distance exceeds the supported numeric range");
        }

        if (closest.distance_along_track >= length()) {
            closest.distance_along_track = 0.0;
        }

        closest.sample.tangent =
                sample_at(closest.distance_along_track).tangent;

        const double offset_x = position.x - closest.sample.position.x;
        const double offset_y = position.y - closest.sample.position.y;

        const double side =
                closest.sample.tangent.x * offset_y
                - closest.sample.tangent.y * offset_x;

        closest.lateral_offset = std::copysign(closest_distance, side);

        return closest;
    }
}
