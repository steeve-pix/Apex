#include "simulation/world.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
    void write_sample(std::ostream &output, const apex::simulation::WorldState &world,
                      const apex::simulation::TrackSettings &track) {
        const auto &body = world.vehicle.body;

        output << world.elapsed_time << ',' << body.position.x << ',' << body.position.y << ',' << body.velocity.x <<
                ',' << body.velocity.y << ',' << world.vehicle.heading << ',' << (
                    apex::simulation::is_on_track(body.position, track) ? 1 : 0) << '\n';
    }

    double parse_control(const char *argument) {
        const std::string text{argument};
        std::size_t parsed{};
        const double value = std::stod(text, &parsed);

        if (parsed != text.size() ||
            !std::isfinite(value) ||
            value < -1.0 || value > 1.0) {
            throw std::invalid_argument("Controls must be finite numbers between -1 and 1");
        }

        return value;
    }
}

int main(int argc, char *argv[]) {
    try {
        if (argc != 1 && argc != 3) {
            throw std::invalid_argument("Usage: apex_headless [throttle steering]");
        }
        const apex::simulation::WorldSetting settings{};
        auto world = apex::simulation::make_initial_world(settings);

        constexpr int steps_per_second = 120;
        constexpr int duration_seconds = 10;
        constexpr int step_count =
                steps_per_second * duration_seconds;
        constexpr double dt = 1.0 / steps_per_second;

        apex::simulation::VehicleControls controls{
            .throttle = 1.0
        };

        if (argc == 3) {
            controls.throttle = parse_control(argv[1]);
            controls.steering = parse_control(argv[2]);
        }

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
