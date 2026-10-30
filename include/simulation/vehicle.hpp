#pragma once
#include "body.hpp"

namespace apex::simulation {
    struct VehicleState {
        Body2D body{};
        double heading{};
    };

    struct VehicleControls {
        double throttle{};
        double steering{};
    };

    struct VehicleSettings {
        MotionSettings motion{};
        double wheelbase{0.2};
        double max_steering_angle{0.45};
        double lateral_grip{8.0};
    };

    void update_vehicle(VehicleState &vehicle, const VehicleControls &controls, const VehicleSettings &settings,
                        double dt);
}
