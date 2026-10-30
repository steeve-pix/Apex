#include "simulation/world.hpp"

#include <numbers>

namespace apex::simulation {
    WorldState make_initial_world(const WorldSetting &settings) {
        WorldState world{};

        world.vehicle.body.position = {
            settings.track.centerline_radius, 0.0
        };

        world.vehicle.heading =
                std::numbers::pi_v<double> / 2.0;

        return world;
    }

    void step_world(WorldState &world, const VehicleControls &controls, const WorldSetting &settings, double dt) {
        const auto step_settings = vehicle_settings_at(world.vehicle.body.position, settings.vehicle, settings.track);

        update_vehicle(world.vehicle, controls, step_settings, dt);
        world.elapsed_time += dt;
    }
}
