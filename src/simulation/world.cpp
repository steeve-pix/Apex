#include "simulation/world.hpp"

#include <numbers>
#include <cmath>

namespace apex::simulation {
    WorldState make_initial_world(const WorldSettings &settings) {
        WorldState world{};

        if (settings.track.path) {
            const auto start = settings.track.path->sample_at(0.0);

            world.vehicle.body.position = start.position;
            world.vehicle.heading = std::atan2(start.tangent.y, start.tangent.x);
        } else {
            world.vehicle.body.position = {
                settings.track.centerline_radius, 0.0
            };
            world.vehicle.heading =
                    std::numbers::pi_v<double> / 2.0;
        }

        return world;
    }

    void step_world(WorldState &world, const VehicleControls &controls, const WorldSettings &settings, double dt) {
        const auto step_settings = vehicle_settings_at(world.vehicle.body.position, settings.vehicle, settings.track);

        update_vehicle(world.vehicle, controls, step_settings, dt);
        world.elapsed_time += dt;
    }
}
