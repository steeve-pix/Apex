#pragma once

#include <array>
#include <optional>
#include <span>
#include <vector>

#include "simulation/world.hpp"

namespace apex::simulation {
    inline constexpr int grid_size = 22;
    inline constexpr int player_slot = 10;
    inline constexpr double start_line_distance = 7.4;
    inline constexpr double runoff_width = 0.15;
    inline constexpr double grid_lane_offset = 0.145;
    // One world unit is 17 metres at the 3.4 m race car wheelbase.
    inline constexpr double race_speed_to_kmh = 17.0 * 3.6;
    inline constexpr double race_top_speed = 350.0 / race_speed_to_kmh;

    [[nodiscard]] double grid_distance(int slot);

    [[nodiscard]] double grid_lane(int slot);

    struct RaceStatus {
        double countdown{};
        bool started{};
        bool automatic{};
        bool paused{};
        bool on_track{};
        int position{player_slot + 1};
        int completed_laps{};
        double speed{};
        double lap_time{};
        std::optional<double> best_lap;
        std::optional<double> last_lap;

        [[nodiscard]] int start_lights() const;
    };

    enum class SessionMode { time_trial, full_grid };

    [[nodiscard]] WorldSettings make_race_settings();

    class Race {
    public:
        explicit Race(
            const WorldSettings &settings, SessionMode mode = SessionMode::time_trial,
            std::optional<int> starting_slot = std::nullopt);

        void step(const VehicleControls &input, bool automatic, double dt);

        [[nodiscard]] const WorldState &player() const { return player_; }

        [[nodiscard]] std::span<const VehicleState> opponents() const {
            return mode_ == SessionMode::full_grid
                       ? std::span<const VehicleState>{opponents_}
                       : std::span<const VehicleState>{};
        }

        [[nodiscard]] const RaceStatus &status() const { return status_; }

        [[nodiscard]] VehicleControls player_controls() const {
            return player_controls_;
        }

    private:
        WorldSettings settings_;
        SessionMode mode_;
        WorldState player_{};
        std::array<VehicleState, grid_size - 1> opponents_{};
        std::array<double, grid_size> progress_{};
        std::vector<double> automatic_speed_limits_;
        RaceStatus status_{};
        VehicleControls player_controls_{};
        double launch_time_{-1.0};
        std::optional<double> lap_start_;
        double next_line_{start_line_distance};
    };
}
