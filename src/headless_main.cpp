#include "simulation/world.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <iomanip>
#include <limits>

namespace {
    void write_sample(std::ostream &output, const apex::simulation::WorldState &world,
                      const apex::simulation::TrackSettings &track) {
        const auto &body = world.vehicle.body;

        output << world.elapsed_time << ',' << body.position.x << ',' << body.position.y << ',' << body.velocity.x <<
                ',' << body.velocity.y << ',' << world.vehicle.heading << ',' << (
                    apex::simulation::is_on_track(body.position, track) ? 1 : 0) << '\n';
    }
}

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

        std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

        std::cout << "time,x,y,vx,vy,heading,on_track\n";
        write_sample(std::cout, world, settings.track);

        for (int step = 0; step < step_count; ++step) {
            apex::simulation::step_world(world, controls, settings, dt);
            write_sample(std::cout, world, settings.track);
        }

        const auto &body = world.vehicle.body;

        std::cerr << "Simulation time: " << world.elapsed_time << " s\n" << "Position: " << body.position.x << ", " <<
                body.position.y << '\n' << "Speed: " << std::hypot(body.velocity.x, body.velocity.y) << " units/s\n";

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
