#include "simulation/vehicle.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>
#include <stdexcept>

namespace apex::simulation {
    void update_vehicle(VehicleState &vehicle, const VehicleControls &controls, const VehicleSettings &settings,
                        double dt) {
        constexpr double full_turn =
                2.0 * std::numbers::pi_v<double>;

        if (!(settings.wheelbase > 0.0)) {
            throw std::invalid_argument("Wheelbase must be positive");
        }

        const math::Vec2 previous_forward{
            std::cos(vehicle.heading), std::sin(vehicle.heading)
        };

        const double forward_speed =
                vehicle.body.velocity.x * previous_forward.x +
                vehicle.body.velocity.y * previous_forward.y;

        double steering_angle =
                std::clamp(controls.steering, -1.0, 1.0) * settings.max_steering_angle;
        if (settings.lateral_acceleration_limit > 0.0 && forward_speed != 0.0) {
            const double cornering_lock = std::atan(settings.lateral_acceleration_limit * settings.wheelbase
                                                  / (forward_speed * forward_speed));
            steering_angle = std::clamp(steering_angle, -cornering_lock, cornering_lock);
        }
        vehicle.steering_angle = steering_angle;

        const double yaw_rate =
                forward_speed / settings.wheelbase * std::tan(steering_angle);

        vehicle.heading = std::remainder(vehicle.heading + yaw_rate * dt, full_turn);

        const math::Vec2 forward{std::cos(vehicle.heading), std::sin(vehicle.heading)};

        auto motion = settings.motion;
        double throttle = controls.throttle;
        if (throttle < 0.0 && settings.braking_strength > 0.0) {
            if (forward_speed > 0.0 && dt > 0.0) {
                const double speed = std::hypot(vehicle.body.velocity.x, vehicle.body.velocity.y);
                const double stopping_force = std::max(0.0, forward_speed * (1.0 / dt - motion.drag_at_speed(speed)));
                motion.acceleration_strength = std::min(-throttle * settings.braking_strength, stopping_force);
                throttle = -1.0;
            } else {
                constexpr double reverse_speed = 0.65;
                throttle *= 0.35 * std::clamp(1.0 + forward_speed / reverse_speed, 0.0, 1.0);
            }
        }
        const math::Vec2 thrust_direction{forward.x * throttle, forward.y * throttle};

        const auto previous_position = vehicle.body.position;
        update_body(vehicle.body, thrust_direction, motion, dt);
        vehicle.forward_travel += (vehicle.body.position.x - previous_position.x) * forward.x
                                  + (vehicle.body.position.y - previous_position.y) * forward.y;

        const math::Vec2 sideways{-forward.y, forward.x};

        auto &velocity = vehicle.body.velocity;

        const double lateral_speed = velocity.x * sideways.x + velocity.y * sideways.y;
        const double retrained_fraction =
                std::exp(-settings.lateral_grip * dt);

        const double correction =
                lateral_speed * (1.0 - retrained_fraction);

        velocity.x -= sideways.x * correction;
        velocity.y -= sideways.y * correction;
    }
}
