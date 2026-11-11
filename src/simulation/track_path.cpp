#include "simulation/track_path.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>
#include <algorithm>
#include <iterator>
#include <limits>

namespace apex::simulation {
    TrackPath::TrackPath(std::vector<math::Vec2> points)
        : points_{std::move(points)} {
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
