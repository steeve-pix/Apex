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
        double turn_rate{2.0};
    };

    void update_vehicle(VehicleState &vehicle, const VehicleControls &controls, const VehicleSettings &settings,
                        double dt);

}
