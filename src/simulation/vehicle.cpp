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

        const double steering_angle =
                std::clamp(controls.steering, -1.0, 1.0) * settings.max_steering_angle;

        const double yaw_rate =
                forward_speed / settings.wheelbase * std::tan(steering_angle);

        vehicle.heading = std::remainder(vehicle.heading + yaw_rate * dt, full_turn);

        const math::Vec2 forward{std::cos(vehicle.heading), std::sin(vehicle.heading)};

        const math::Vec2 thrust_direction{
            forward.x * controls.throttle,
            forward.y * controls.throttle
        };

        update_body(vehicle.body, thrust_direction, settings.motion, dt);

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
