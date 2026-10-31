#include "simulation/track.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>

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
            .forward_speed = body.velocity.x * std::cos(vehicle.heading) + body.velocity.y * std::sin(
                                 vehicle.heading)
        };
    }
}
