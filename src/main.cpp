#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <thread>
#include <string_view>
#include <memory>

#include "simulation/circuit_layout.hpp"
#include "graphics/simulation_window.hpp"
#include "simulation/race.hpp"
#include "simulation/sensors.hpp"
#include "simulation/track.hpp"
#include "simulation/world.hpp"

int main(int argc, char **argv) {
    auto mode = apex::simulation::SessionMode::time_trial;
    if (argc == 2 && std::string_view{argv[1]} == "--grid") {
        mode = apex::simulation::SessionMode::full_grid;
    } else if (argc != 1) {
        std::cerr << "Usage: Apex [--grid]\n";
        return EXIT_FAILURE;
    }

    try {
        const auto world_settings =
                apex::simulation::make_race_settings();

        const apex::simulation::SensorSettings sensor_settings{};

        apex::graphics::SimulationWindow window{world_settings.track};
        apex::simulation::Race race{world_settings, mode};

        auto previous_world = race.player();
        bool paused = false;
        bool automatic = false;

        constexpr double physics_step = 1.0 / 120.0;
        constexpr double maximum_frame_time = 0.25;

        double accumulator = 0.0;

        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();

        while (true) {
            const auto current_time = Clock::now();

            double frame_time = std::min(std::chrono::duration<double>(current_time - previous_time).count(),
                                         maximum_frame_time);

            previous_time = current_time;

            const auto input = window.poll_events();

            if (input.close_requested) {
                break;
            }

            if (input.reset_requested) {
                race = apex::simulation::Race{world_settings, mode};
                previous_world = race.player();
                accumulator = 0.0;
                frame_time = 0.0;
            }

            if (input.pause_requested) {
                paused = !paused;
                previous_world = race.player();
                accumulator = 0.0;
                frame_time = 0.0;
            }

            if (input.autopilot_requested) {
                automatic = !automatic;
            }

            if (!paused) {
                accumulator += frame_time;

                while (accumulator >= physics_step) {
                    previous_world = race.player();
                    race.step(input.controls, automatic, physics_step);
                    accumulator -= physics_step;
                }
            }

            auto status = race.status();
            status.paused = paused;
            status.automatic = automatic;
            auto display_world = race.player();
            if (!paused) {
                const double blend = accumulator / physics_step;
                const auto &previous_car = previous_world.vehicle;
                auto &car = display_world.vehicle;
                car.body.position.x = std::lerp(previous_car.body.position.x, car.body.position.x, blend);
                car.body.position.y = std::lerp(previous_car.body.position.y, car.body.position.y, blend);
                const double turn = std::remainder(car.heading - previous_car.heading,
                                                   2.0 * std::numbers::pi_v<double>);
                car.heading = previous_car.heading + blend * turn;
                car.steering_angle = std::lerp(previous_car.steering_angle, car.steering_angle, blend);
                car.forward_travel = std::lerp(previous_car.forward_travel, car.forward_travel, blend);
                display_world.elapsed_time = std::lerp(previous_world.elapsed_time, display_world.elapsed_time, blend);
            }
            window.draw(display_world, sensor_settings, "Sunset Circuit", race.opponents(), &status);

            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
