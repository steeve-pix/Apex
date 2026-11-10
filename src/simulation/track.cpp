#include "simulation/track.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <algorithm>
#include <limits>

#include "simulation/track_boundaries.hpp"

namespace apex::simulation {
    namespace {
        double cross(const math::Vec2 &a, const math::Vec2 &b, const math::Vec2 &point) {
            return (b.x - a.x) * (point.y - a.y)
                   - (b.y - a.y) * (point.x - a.x);
        }

        bool contains_point(const math::Vec2 &point, const math::Vec2 &a, const math::Vec2 &b, const math::Vec2 &c) {
            if (point.x < std::min({a.x, b.x, c.x}) || point.x > std::max({a.x, b.x, c.x}) || point.y < std::min({
                    a.y, b.y, c.y
                }) || point.y > std::max({a.y, b.y, c.y})) {
                return false;
            }

            const double area = cross(a, b, c);

            if (!std::isfinite(area) || area == 0.0) {
                return false;
            }

            const double first = cross(a, b, point);
            const double second = cross(b, c, point);
            const double third = cross(c, a, point);

            const double coordinate_scale = std::max({
                std::abs(a.x), std::abs(a.y),
                std::abs(b.x), std::abs(b.y),
                std::abs(c.x), std::abs(c.y)
            });

            const double edge_scale = std::max({
                std::hypot(b.x - a.x, b.y - a.y),
                std::hypot(c.x - b.x, c.y - b.y),
                std::hypot(a.x - c.x, a.y - c.y)
            });

            constexpr double rounding_margin =
                    32.0 * std::numeric_limits<double>::epsilon();

            const double tolerance =
                    rounding_margin * coordinate_scale * edge_scale;

            const bool has_negative =
                    first < -tolerance
                    || second < -tolerance
                    || third < -tolerance;

            const bool has_positive =
                    first > tolerance || second > tolerance || third > tolerance;

            return !(has_negative && has_positive);
        }
    }

    bool is_on_track(const math::Vec2 &position, const TrackSettings &settings) {
        if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
            return false;
        }

        if (settings.path) {
            const auto boundaries = make_track_boundaries(*settings.path, settings.road_width * 0.5);

            for (std::size_t index = 0; index < boundaries.left.size(); ++index) {
                const std::size_t next =
                        (index + 1) % boundaries.left.size();

                const auto &right = boundaries.right[index];
                const auto &left = boundaries.left[index];
                const auto &next_right = boundaries.right[next];
                const auto &next_left = boundaries.left[next];

                if (contains_point(position, right, left, next_right) || contains_point(
                        position, left, next_right, next_left)) {
                    return true;
                }
            }

            return false;
        }
        const double distance = std::hypot(position.x, position.y);
        const double half_width = settings.road_width * 0.5;

        const double inner_radius =
                settings.centerline_radius - half_width;
        const double outer_radius =
                settings.centerline_radius + half_width;

        return distance >= inner_radius &&
               distance <= outer_radius;
    }

    VehicleSettings vehicle_settings_at(const math::Vec2 &position, const VehicleSettings &base_settings,
                                        const TrackSettings &track) {
        auto settings = base_settings;

        if (!is_on_track(position, track)) {
            settings.motion.linear_drag += track.off_road_drag;
        }

        return settings;
    }

    TrackObservation observe_track(const VehicleState &vehicle, const TrackSettings &track) {
        const auto &body = vehicle.body;

        const double forward_speed =
                body.velocity.x * std::cos(vehicle.heading)
                + body.velocity.y * std::sin(vehicle.heading);
        if (track.path) {
            const auto projection = track.path->project(body.position);

            const double desired_heading = std::atan2(projection.sample.tangent.y, projection.sample.tangent.x);

            return TrackObservation{
                .centerline_error = -projection.lateral_offset,
                .heading_error = std::remainder(desired_heading - vehicle.heading, 2.0 * std::numbers::pi_v<double>),
                .forward_speed = forward_speed
            };
        }

        const double radius =
                std::hypot(body.position.x, body.position.y);

        if (radius == 0.0) {
            throw std::invalid_argument("Track heading is undefined at the origin");
        }
        constexpr double pi = std::numbers::pi_v<double>;

        const double radial_angle =
                std::atan2(body.position.y, body.position.x);

        const double desired_heading = radial_angle + pi / 2.0;
        return TrackObservation{
            .centerline_error = radius - track.centerline_radius,
            .heading_error = std::remainder(desired_heading - vehicle.heading, 2.0 * pi),
            .forward_speed = forward_speed
        };
    };
}
