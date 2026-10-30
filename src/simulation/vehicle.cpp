#include "simulation/vehicle.hpp"

#include <cmath>
#include <numbers>

namespace apex::simulation {
    void update_vehicle(VehicleState &vehicle, const VehicleControls &controls, const VehicleSettings &settings,
                        double dt) {
        constexpr double full_turn =
                2.0 * std::numbers::pi_v<double>;

        vehicle.heading = std::remainder(vehicle.heading + controls.steering * settings.turn_rate * dt, full_turn);

        const math::Vec2 forward{std::cos(vehicle.heading), std::sin(vehicle.heading)};

        const math::Vec2 thrust_direction{
            forward.x * controls.throttle,
            forward.y * controls.throttle
        };

        update_body(vehicle.body, thrust_direction, settings.motion, dt);
    }
}
