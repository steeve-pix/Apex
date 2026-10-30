#pragma once

#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"

namespace apex::simulation {
    struct WorldSetting {
        VehicleSettings vehicle{};
        TrackSettings track{};
    };

    struct WorldState {
        VehicleState vehicle{};
        double elapsed_time{};
    };

    [[nodiscard]] WorldState make_initial_world(const WorldSetting &settings);

    void step_world(WorldState &world, const VehicleControls &controls, const WorldSetting &settings, double dt);
}
