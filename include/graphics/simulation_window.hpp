#pragma once

#include <memory>
#include <string_view>

#include "simulation/sensors.hpp"
#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"
#include "simulation/world.hpp"

namespace apex::graphics {
    struct WindowInput {
        simulation::VehicleControls controls{};
        bool close_requested{};
        bool reset_requested{};
        bool pause_requested{};
        bool autopilot_requested{};
    };

    class SimulationWindow {
    public:
        explicit SimulationWindow(const simulation::TrackSettings &track);

        ~SimulationWindow();

        SimulationWindow(const SimulationWindow &) = delete;

        SimulationWindow &operator=(const SimulationWindow &) = delete;

        WindowInput poll_events();

        void draw(const simulation::WorldState &world, const simulation::SensorSettings &sensors,
                  std::string_view status);

        void close() noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
