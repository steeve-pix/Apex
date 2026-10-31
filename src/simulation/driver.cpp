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
}
