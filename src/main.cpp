#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include "graphics/simulation_window.hpp"
#include "simulation/driver.hpp"
#include "simulation/sensors.hpp"
#include "simulation/track.hpp"
#include "simulation/world.hpp"

int main() {
    try {
        const apex::simulation::WorldSettings world_settings{};
        const apex::simulation::SensorSettings sensor_settings{};
        const apex::simulation::DriverSettings driver_settings{};

        apex::graphics::SimulationWindow window{world_settings.track};
        auto world = apex::simulation::make_initial_world(world_settings);

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
                world = apex::simulation::make_initial_world(world_settings);
                accumulator = 0.0;
                frame_time = 0.0;
            }

            if (input.pause_requested) {
                paused = !paused;
                accumulator = 0.0;
                frame_time = 0.0;
            }

            if (input.autopilot_requested) {
                automatic = !automatic;
            }

            if (!paused) {
                accumulator += frame_time;

                while (accumulator >= physics_step) {
                    auto controls = input.controls;

                    if (automatic) {
                        const auto observation =
                                apex::simulation::observe_track(
                                    world.vehicle,
                                    world_settings.track
                                );

                        controls = apex::simulation::drive_automatically(
                            observation,
                            world_settings.vehicle,
                            world_settings.track,
                            driver_settings
                        );
                    }

                    apex::simulation::step_world(
                        world,
                        controls,
                        world_settings,
                        physics_step
                    );

                    accumulator -= physics_step;
                }
            }

            const auto &velocity = world.vehicle.body.velocity;
            const double speed = std::hypot(velocity.x, velocity.y);

            const bool on_track = apex::simulation::is_on_track(
                world.vehicle.body.position,
                world_settings.track
            );

            std::ostringstream status;
            status << "Speed " << std::fixed << std::setprecision(2)
                    << speed << " units/s"
                    << (on_track ? " | On track" : " | Off track")
                    << (paused ? " | Paused" : " | Running")
                    << (automatic ? " | Auto" : " | Manual");

            window.draw(world, sensor_settings, status.str());

            std::this_thread::sleep_for(std::chrono::milliseconds{1});
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
