#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "simulation/sensors.hpp"
#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"
#include "simulation/race.hpp"

namespace apex::graphics {
    struct TrainingStanding {
        std::size_t slot;
        double progress;
        std::optional<double> lap_time;
    };

    struct TrainingRecap {
        std::size_t winner_slot;
        double remaining_seconds;
        bool held;
    };

    struct TrainingTelemetry {
        std::size_t generation;
        std::size_t active_cars;
        double best_distance;
        std::span<const TrainingStanding> standings{};
        std::optional<TrainingRecap> recap{};
    };

    struct WindowInput {
        simulation::VehicleControls controls{};
        bool close_requested{};
        bool reset_requested{};
        bool pause_requested{};
        bool continue_requested{};
        bool autopilot_requested{};
        int car_selection_delta{};
        bool follow_leader_requested{};
    };

    class SimulationWindow {
    public:
        explicit SimulationWindow(const simulation::TrackSettings &track);

        ~SimulationWindow();

        SimulationWindow(const SimulationWindow &) = delete;

        SimulationWindow &operator=(const SimulationWindow &) = delete;

        WindowInput poll_events(bool spectate = false);

        // Opponents are in grid order with the focused slot omitted.
        void draw(const simulation::WorldState &world, const simulation::SensorSettings &sensors,
                  std::string_view status,
                  std::span<const simulation::VehicleState> opponents = {},
                  const simulation::RaceStatus *race = nullptr,
                  std::size_t focused_slot = simulation::player_slot, std::string_view focused_driver = {},
                  std::span<const std::string> driver_names = {}, const TrainingTelemetry *training = nullptr,
                  std::span<const std::size_t> eliminated_slots = {});

        void close() noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
