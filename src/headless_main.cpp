#include "simulation/world.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>

int main() {
    try {
        const apex::simulation::WorldSetting settings{};
        auto world = apex::simulation::make_initial_world(settings);

        constexpr int steps_per_second = 120;
        constexpr int duration_seconds = 10;
        constexpr int step_count =
                steps_per_second * duration_seconds;
        constexpr double dt = 1.0 / steps_per_second;

        const apex::simulation::VehicleControls controls{
            .throttle = 1.0
        };

        for (int step = 0; step < step_count; ++step) {
            apex::simulation::step_world(world, controls, settings, dt);
        }

        const auto &body = world.vehicle.body;

        std::cout << "Simulation time: " << world.elapsed_time << " s\n" << "Position: " << body.position.x << ", " <<
                body.position.y << '\n' << "Speed: " << std::hypot(body.velocity.x, body.velocity.y) << " units/s\n";

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
