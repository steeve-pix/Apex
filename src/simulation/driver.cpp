#include "simulation/driver.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apex::simulation {
    VehicleControls drive_automatically(const TrackObservation &observation, const VehicleSettings &vehicle,
                                        const TrackSettings &track, const DriverSettings &driver) {
        if (!(track.centerline_radius > 0.0) || !(vehicle.wheelbase > 0.0) || !(vehicle.max_steering_angle > 0.0)) {
            throw std::invalid_argument("Driver requires positive track radius, wheelbase, "
                "and maximum steering angle");
        }

        const double base_angle = std::atan(vehicle.wheelbase / track.centerline_radius);
        const double steering_angle =
                base_angle +
                driver.heading_gain * observation.heading_error +
                driver.centerline_gain * observation.centerline_error;
        const double speed_error =
                driver.target_speed - observation.forward_speed;

        return VehicleControls{
            .throttle = std::clamp(driver.speed_gain * speed_error, 0.0, 1.0),
            .steering = std::clamp(steering_angle / vehicle.max_steering_angle, -1.0, 1.0)
        };
    }

    VehicleControls drive_automatically(const VehicleState &state, const VehicleSettings &vehicle,
                                        const TrackSettings &track, const DriverSettings &driver) {
        if (!track.path) {
            return drive_automatically(observe_track(state, track), vehicle, track, driver);
        }

        if (!std::isfinite(driver.lookahead_distance) || driver.lookahead_distance <= 0.0 || !
            std::isfinite(vehicle.wheelbase) || vehicle.wheelbase <= 0.0 || !std::isfinite(vehicle.max_steering_angle)
            || vehicle.max_steering_angle <= 0.0 || !std::isfinite(driver.target_speed) || driver.target_speed < 0.0 ||
            !std::isfinite(driver.speed_gain) || driver.speed_gain < 0.0) {
            throw std::invalid_argument("Invalid circuit driver settings");
        }

        if (!std::isfinite(state.heading) || !std::isfinite(state.body.velocity.x) || !std::isfinite(
                state.body.velocity.y)) {
            throw std::invalid_argument("Invalid circuit driver state");
        }

        const auto projection = track.path->project(state.body.position);

        const auto target = track.path->sample_at(projection.distance_along_track + driver.lookahead_distance);

        const double dx = target.position.x - state.body.position.x;
        const double dy = target.position.y - state.body.position.y;
        const double distance_squared = dx * dx + dy * dy;

        if (!std::isfinite(distance_squared) || distance_squared <= 0.0) {
            throw std::invalid_argument("Driver target must be a distinct finite point");
        }

        const double sine = std::sin(state.heading);
        const double cosine = std::cos(state.heading);

        const double lateral_target =
                -sine * dx + cosine * dy;

        const double curvature =
                2.0 * lateral_target / distance_squared;

        const double steering_angle =
                std::atan(vehicle.wheelbase * curvature);

        const double forward_speed =
                state.body.velocity.x * cosine + state.body.velocity.y * sine;

        const double speed_error =
                driver.target_speed - forward_speed;

        if (!std::isfinite(steering_angle) ||
            !std::isfinite(speed_error)) {
            throw std::invalid_argument("Circuit driver calculation exceeds the numeric range");
        }

        return VehicleControls{
            .throttle = std::clamp(driver.speed_gain * speed_error, 0.0, 1.0),
            .steering = std::clamp(steering_angle / vehicle.max_steering_angle, -1.0, 1.0)
        };
    }
}
