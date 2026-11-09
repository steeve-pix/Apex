#include "simulation/track_boundaries.hpp"

#include <cmath>
#include <span>
#include <stdexcept>
#include <vector>

namespace apex::simulation {
    namespace {
        math::Vec2 segment_direction(const math::Vec2 &start, const math::Vec2 &end) {
            const double dx = end.x - start.x;
            const double dy = end.y - start.y;
            const double length = std::hypot(dx, dy);

            return {dx / length, dy / length};
        }

        void validate_boundary(const std::vector<math::Vec2> &boundary, std::span<const math::Vec2> centerline) {
            for (std::size_t index = 0; index < boundary.size(); ++index) {
                const std::size_t next = (index + 1) % boundary.size();

                const auto direction = segment_direction(
                    centerline[index],
                    centerline[next]
                );

                const double forward_distance =
                        (boundary[next].x - boundary[index].x) * direction.x
                        + (boundary[next].y - boundary[index].y) * direction.y;

                if (!std::isfinite(forward_distance) ||
                    forward_distance <= 0.0) {
                    throw std::invalid_argument("Boundary offset folds a track segment");
                }
            }
        }
    }

    TrackBoundaries make_track_boundaries(
        const TrackPath &path,
        double half_width) {
        if (!std::isfinite(half_width) || half_width <= 0.0) {
            throw std::invalid_argument("Track half-width must be finite and positive");
        }

        const auto points = path.points();

        if (points.size() < 3) {
            throw std::invalid_argument("Track boundaries require a valid closed path");
        }

        TrackBoundaries boundaries;
        boundaries.left.reserve(points.size());
        boundaries.right.reserve(points.size());

        for (std::size_t index = 0; index < points.size(); ++index) {
            const std::size_t previous =
                    (index + points.size() - 1) % points.size();
            const std::size_t next = (index + 1) % points.size();

            const auto incoming = segment_direction(
                points[previous],
                points[index]
            );
            const auto outgoing = segment_direction(
                points[index],
                points[next]
            );

            const math::Vec2 incoming_normal{
                -incoming.y,
                incoming.x
            };
            const math::Vec2 outgoing_normal{
                -outgoing.y,
                outgoing.x
            };

            const math::Vec2 normal_sum{
                incoming_normal.x + outgoing_normal.x,
                incoming_normal.y + outgoing_normal.y
            };

            const double alignment =
                    normal_sum.x * outgoing_normal.x
                    + normal_sum.y * outgoing_normal.y;

            constexpr double minimum_alignment = 0.000001;

            if (alignment <= minimum_alignment) {
                throw std::invalid_argument("Track corner is too sharp for an offset join");
            }

            const double scale = half_width / alignment;

            const math::Vec2 offset{normal_sum.x * scale, normal_sum.y * scale};

            const math::Vec2 left{points[index].x + offset.x, points[index].y + offset.y};
            const math::Vec2 right{points[index].x - offset.x, points[index].y - offset.y};

            if (!std::isfinite(left.x) || !std::isfinite(left.y) ||
                !std::isfinite(right.x) || !std::isfinite(right.y)) {
                throw std::invalid_argument("Track boundary coordinates exceed the numeric range");
            }

            boundaries.left.push_back(left);
            boundaries.right.push_back(right);
        }

        validate_boundary(boundaries.left, points);
        validate_boundary(boundaries.right, points);

        return boundaries;
    }
}
