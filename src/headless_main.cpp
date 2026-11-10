#include "simulation/world.hpp"
#include "simulation/driver.hpp"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>

#include "simulation/sensors.hpp"

namespace {
    void write_sample(std::ostream &output, const apex::simulation::WorldState &world,
                      const apex::simulation::TrackSettings &track, const apex::simulation::SensorSettings &sensors,
                      const apex::simulation::VehicleControls &controls) {
        const auto &body = world.vehicle.body;
        const auto readings = apex::simulation::sense_track(
            world.vehicle, track, sensors
        );

        output << world.elapsed_time << ','
                << body.position.x << ','
                << body.position.y << ','
                << body.velocity.x << ','
                << body.velocity.y << ','
                << world.vehicle.heading << ','
                << (apex::simulation::is_on_track(body.position, track) ? 1 : 0);


        output << "," << controls.throttle
                << "," << controls.steering;

        for (const auto &reading: readings) {
            output << ',' << reading.distance
                    << ',' << (reading.hit ? 1 : 0);
        }

        output << '\n';
    }

    double parse_number(const char *argument) {
        const std::string text{argument};
        std::size_t parsed{};
        const double value = std::stod(text, &parsed);

        if (parsed != text.size() || !std::isfinite(value)) {
            throw std::invalid_argument("Expected a finite number");
        }

        return value;
    }

    double parse_control(const char *argument) {
        const double value = parse_number(argument);

        if (value < -1.0 || value > 1.0) {
            throw std::invalid_argument("Controls must be between -1 and 1");
        }

        return value;
    }
}

int main(int argc, char *argv[]) {
    try {
        const bool automatic =
                argc > 1 && std::string{argv[1]} == "--auto";

        const bool valid_arguments = automatic
                                         ? (argc == 2 || argc == 4)
                                         : (argc == 1 || argc == 3);

        if (!valid_arguments) {
            throw std::invalid_argument(
                "Usage: apex_headless [throttle steering] | "
                "--auto [centerline_offset heading_offset]"
            );
        }
        const apex::simulation::WorldSettings settings{};
        auto world = apex::simulation::make_initial_world(settings);

        if (automatic && argc == 4) {
            const double centerline_offset = parse_number(argv[2]);
            const double heading_offset = parse_number(argv[3]);

            if (std::abs(centerline_offset) >
                settings.track.road_width * 0.5) {
                throw std::invalid_argument(
                    "Starting position must be within the road"
                );
            }

            world.vehicle.body.position.x += centerline_offset;
            world.vehicle.heading += heading_offset;
        }

        constexpr int steps_per_second = 120;
        constexpr int duration_seconds = 10;
        constexpr int step_count =
                steps_per_second * duration_seconds;
        constexpr double dt = 1.0 / steps_per_second;

        apex::simulation::VehicleControls controls{
            .throttle = 1.0
        };

        const apex::simulation::DriverSettings driver_settings{};
        const apex::simulation::SensorSettings sensor_settings{};

        if (argc == 3) {
            controls.throttle = parse_control(argv[1]);
            controls.steering = parse_control(argv[2]);
        }

        std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);

        std::cout << "time,x,y,vx,vy,heading,on_track,throttle,steering";

        for (std::size_t i = 0; i < apex::simulation::sensor_count; ++i) {
            std::cout << ",sensor_" << i << "_distance"
                    << ",sensor_" << i << "_hit";
        }

        std::cout << '\n';

        for (int step = 0; step <= step_count; ++step) {
            if (automatic) {
                controls = apex::simulation::drive_automatically(world.vehicle, settings.vehicle, settings.track,
                                                                 driver_settings);
            }
            write_sample(std::cout, world, settings.track, sensor_settings, controls);

            if (step < step_count) {
                apex::simulation::step_world(world, controls, settings, dt);
            }
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
