#include "simulation/race.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "simulation/circuit_layout.hpp"


namespace apex::simulation {
    namespace {
        math::Vec2 lane_position(const TrackSample &sample, double lane) {
            return {
                sample.position.x - sample.tangent.y * lane,
                sample.position.y + sample.tangent.x * lane
            };
        }

        double dot(math::Vec2 a, math::Vec2 b) {
            return a.x * b.x + a.y * b.y;
        }

        double ease_input(double current, double target, double dt, double response_time) {
            const double eased = std::lerp(current, target, -std::expm1(-dt / response_time));
            return std::abs(eased - target) < 0.001 ? target : eased;
        }

        std::vector<double> make_speed_limits(const TrackPath &path, const VehicleSettings &vehicle) {
            constexpr double sample_spacing = .20;
            constexpr double curve_probe = .32;
            const auto count = static_cast<std::size_t>(std::ceil(path.length() / sample_spacing));
            const double spacing = path.length() / count;
            const double cornering = vehicle.lateral_acceleration_limit * .57;
            const double braking = vehicle.braking_strength * .59;
            std::vector<double> speeds(count);
            for (std::size_t i = 0; i < count; ++i) {
                const double distance = i * spacing;
                const auto before = path.sample_at(distance - curve_probe).tangent;
                const auto after = path.sample_at(distance + curve_probe).tangent;
                const double curvature = std::abs(std::atan2(before.x*after.y - before.y*after.x, dot(before,after)))
                                         / (2.0 * curve_probe);
                speeds[i] = std::min(race_top_speed * race_top_speed, cornering / std::max(curvature,.001));
            }
            // Work backwards so the car can brake for the next turn, including across the start line.
            for (std::size_t step = count * 2; step > 0; --step) {
                const std::size_t i = (step - 1) % count;
                speeds[i] = std::min(speeds[i], speeds[(i + 1) % count] + 2.0 * braking * spacing);
            }
            return speeds;
        }

        void resolve_contact(VehicleState &a, VehicleState &b) {
            const math::Vec2 delta{
                b.body.position.x - a.body.position.x,
                b.body.position.y - a.body.position.y
            };
            const double distance = std::hypot(delta.x, delta.y);
            if (distance < 1e-7 || distance > 0.31) {
                return;
            }
            const math::Vec2 normal{delta.x / distance, delta.y / distance};
            const auto radius = [normal](double heading) {
                const double alignment = dot(normal, {std::cos(heading), std::sin(heading)});
                return 1.0 / std::sqrt(alignment * alignment / (0.155 * 0.155)
                                       + (1.0 - alignment * alignment) / (0.064 * 0.064));
            };
            const double overlap = radius(a.heading) + radius(b.heading) - distance;
            if (overlap <= 0.0) {
                return;
            }

            a.body.position.x -= normal.x * overlap * 0.5;
            a.body.position.y -= normal.y * overlap * 0.5;
            b.body.position.x += normal.x * overlap * 0.5;
            b.body.position.y += normal.y * overlap * 0.5;

            const math::Vec2 relative{
                a.body.velocity.x - b.body.velocity.x,
                a.body.velocity.y - b.body.velocity.y
            };
            const double impulse = std::max(0.0, dot(relative, normal)) * 0.52;
            a.body.velocity.x -= normal.x * impulse;
            a.body.velocity.y -= normal.y * impulse;
            b.body.velocity.x += normal.x * impulse;
            b.body.velocity.y += normal.y * impulse;
        }
    }

    double grid_distance(int slot) {
        return start_line_distance - 0.45 - (slot / 2) * 0.53 - (slot % 2) * 0.20;
    }

    double grid_lane(int slot) {
        return slot % 2 == 0 ? grid_lane_offset : -grid_lane_offset;
    }

    int RaceStatus::start_lights() const {
        return countdown > 0.0 ? std::clamp(6 - static_cast<int>(std::ceil(countdown * 5.0 / 3.0)), 1, 5) : 0;
    }

    WorldSettings make_race_settings() {
        WorldSettings settings{};

        settings.track.path = std::make_shared<const TrackPath>(make_apex_circuit());
        settings.track.road_width = 0.64;
        constexpr double drive_acceleration = .90;
        constexpr double rolling_drag = .047;
        constexpr double aero_drag = (drive_acceleration - rolling_drag * race_top_speed)
                                     / (race_top_speed * race_top_speed);
        settings.vehicle.motion = {drive_acceleration, rolling_drag, aero_drag};
        settings.vehicle.max_steering_angle = 0.42;
        settings.vehicle.lateral_grip = 28.0;
        settings.vehicle.braking_strength = 2.65;
        settings.vehicle.lateral_acceleration_limit = 2.10;
        return settings;
    }

    Race::Race(const WorldSettings &settings, SessionMode mode, std::optional<int> starting_slot)
        : settings_{settings}, mode_{mode} {
        if (!settings.track.path || settings.track.path->length() <= start_line_distance
            || settings.track.road_width < 0.46) {
            throw std::invalid_argument("The race grid requires a full width circuit");
        }
        const auto &path = *settings_.track.path;
        status_.on_track = true;
        if (mode_ == SessionMode::time_trial) {
            const int slot = starting_slot.value_or(0);
            if (slot < 0 || slot >= grid_size) {
                throw std::invalid_argument("Starting grid slot is out of range");
            }
            const double distance = grid_distance(slot);
            const auto start = path.sample_at(distance);

            player_.vehicle.body.position = starting_slot
                                                ? lane_position(start, grid_lane(slot))
                                                : start.position;
            player_.vehicle.heading = std::atan2(start.tangent.y, start.tangent.x);
            progress_[player_slot] = distance;
            status_.started = true;
            status_.position = 1;
            return;
        }

        for (int slot = 0; slot < grid_size; ++slot) {
            auto &vehicle = slot == player_slot
                                ? player_.vehicle
                                : opponents_[slot - (slot > player_slot)];
            const double distance = grid_distance(slot);
            const auto sample = path.sample_at(distance);
            vehicle.body.position = lane_position(sample, grid_lane(slot));
            vehicle.heading = std::atan2(sample.tangent.y, sample.tangent.x);
            progress_[slot] = distance;
        }
    }

    void Race::step(const VehicleControls &input, bool automatic, double dt) {
        if (!status_.started && (input.throttle > 0.0 || automatic) && launch_time_ < 0.0) {
            launch_time_ = 3.0;
        }
        if (launch_time_ > 0.0) {
            launch_time_ = std::max(0.0, launch_time_ - dt);
            status_.countdown = launch_time_;
            status_.started = launch_time_ == 0.0;
        }
        if (!status_.started) {
            return;
        }

        const auto &path = *settings_.track.path;
        const bool full_grid = mode_ == SessionMode::full_grid;
        const bool race_dynamics = settings_.vehicle.lateral_acceleration_limit > 0.0
                                   && settings_.vehicle.braking_strength > 0.0;
        if (race_dynamics && (automatic || full_grid) && automatic_speed_limits_.empty()) {
            automatic_speed_limits_ = make_speed_limits(path, settings_.vehicle);
        }
        const int first_slot = full_grid ? 0 : player_slot;
        const int end_slot = full_grid ? grid_size : player_slot + 1;
        std::array<VehicleState *, grid_size> cars{};
        std::array<TrackProjection, grid_size> projections{};
        std::array<math::Vec2, grid_size> velocities{};
        const double half_width = settings_.track.road_width * 0.5;
        for (int slot = first_slot; slot < end_slot; ++slot) {
            cars[slot] = slot == player_slot
                             ? &player_.vehicle
                             : &opponents_[slot - (slot > player_slot)];
            auto &body = cars[slot]->body;
            projections[slot] = path.project(body.position);
            const auto &projection = projections[slot];
            constexpr double car_clearance = 0.065;
            const double wall_limit = half_width + runoff_width - car_clearance;
            if (std::abs(projection.lateral_offset) > wall_limit) {
                const double side = std::copysign(1.0, projection.lateral_offset);
                const math::Vec2 outward{-projection.sample.tangent.y * side, projection.sample.tangent.x * side};
                body.position = lane_position(projection.sample, wall_limit * side);
                const double normal_speed = std::max(0.0, dot(body.velocity, outward));
                body.velocity.x = body.velocity.x * 0.82 - 1.05 * normal_speed * outward.x;
                body.velocity.y = body.velocity.y * 0.82 - 1.05 * normal_speed * outward.y;
            }
            velocities[slot] = body.velocity;
        }

        for (int slot = first_slot; slot < end_slot; ++slot) {
            auto &car = *cars[slot];
            const auto &projection = projections[slot];
            auto controls = input;
            if (slot != player_slot || automatic) {
                const double distance = projection.distance_along_track;
                const double lane = slot == player_slot ? 0.0 : grid_lane(slot);
                const math::Vec2 forward{std::cos(car.heading), std::sin(car.heading)};
                const double speed = dot(car.body.velocity, forward);
                const double lookahead = race_dynamics ? .37 + .12 * std::max(0.0,speed) : .42;
                const auto ahead = path.sample_at(distance + lookahead);
                const auto target = lane_position(ahead, lane);
                const math::Vec2 delta{target.x - car.body.position.x, target.y - car.body.position.y};
                const double curvature = 2.0 * dot(delta, {-forward.y, forward.x})
                                         / std::max(dot(delta, delta), 0.001);
                double desired_speed;
                if (race_dynamics) {
                    const auto count = automatic_speed_limits_.size();
                    const double station = distance / path.length() * count;
                    const auto cell = static_cast<std::size_t>(station);
                    const double limit = std::lerp(automatic_speed_limits_[cell % count],
                                                   automatic_speed_limits_[(cell+1) % count], station-cell);
                    desired_speed = std::sqrt(limit) * (0.92 + 0.02 * (slot % 5));
                } else {
                    const auto far = path.sample_at(distance + 0.95);
                    const double bend = std::acos(std::clamp(dot(projection.sample.tangent, far.tangent), -1.0, 1.0));
                    desired_speed = (0.98 + 0.035 * (slot % 5)) / (1.0 + bend * 0.65);
                }
                if (dot(forward, delta) < 0.0) {
                    desired_speed = std::min(desired_speed, 0.40);
                }
                const double safe_gap = race_dynamics ? .38 + .16 * std::max(0.0,speed) : .38;
                for (int other = first_slot; other < end_slot; ++other) {
                    if (other == slot || std::abs(projections[other].lateral_offset - lane) > 0.16) {
                        continue;
                    }
                    const double gap = std::remainder(projections[other].distance_along_track - distance,
                                                      path.length());
                    if (gap > 0.0 && gap < safe_gap + .62) {
                        const double lead_speed = dot(velocities[other], forward);
                        desired_speed = std::min(desired_speed, std::max(0.0, lead_speed + 2.0 * (gap - safe_gap)));
                    }
                }
                controls = {
                    .throttle = std::clamp(5.5 * (desired_speed - speed), -1.0, 1.0),
                    .steering = std::clamp(std::atan(settings_.vehicle.wheelbase * curvature)
                                           / settings_.vehicle.max_steering_angle, -1.0, 1.0)
                };
            }
            if (slot == player_slot) {
                if (!automatic) {
                    const auto velocity = velocities[slot];
                    const double speed = std::hypot(velocity.x, velocity.y);
                    const double high_speed = race_dynamics ? std::clamp((speed - 1.1) / 2.4, 0.0, 1.0)
                                                          : std::clamp((speed - .65) / 1.15, 0.0, 1.0);
                    const double steering_target = std::clamp(input.steering, -1.0, 1.0)
                                                   * std::lerp(1.0, 0.63, high_speed);
                    const double throttle_target = std::clamp(input.throttle, -1.0, 1.0);
                    // Ease into full lock, then center more quickly when the driver releases the key.
                    const double steering_response = steering_target == 0.0 ? 0.072 : 0.135;
                    const double throttle_response = throttle_target <= 0.0 ? 0.080 : 0.14;
                    controls.steering = ease_input(player_controls_.steering, steering_target, dt, steering_response);
                    controls.throttle = ease_input(player_controls_.throttle, throttle_target, dt, throttle_response);
                }
                player_controls_ = controls;
            }
            auto local = settings_.vehicle;
            if (std::abs(projection.lateral_offset) > half_width) {
                local.motion.linear_drag += settings_.track.off_road_drag;
            }
            update_vehicle(car, controls, local, dt);
        }
        player_.elapsed_time += dt;

        for (int slot = first_slot; slot < end_slot; ++slot) {
            for (int other = slot + 1; other < end_slot; ++other) {
                resolve_contact(*cars[slot], *cars[other]);
            }
        }

        for (int slot = first_slot; slot < end_slot; ++slot) {
            progress_[slot] += std::remainder(projections[slot].distance_along_track - progress_[slot], path.length());
        }
        status_.position = 1 + static_cast<int>(std::count_if(progress_.begin() + first_slot,
                                                              progress_.begin() + end_slot,
                                                              [this](double progress) {
                                                                  return progress > progress_[player_slot];
                                                              }));
        status_.on_track = std::abs(projections[player_slot].lateral_offset) <= half_width;
        const auto velocity = player_.vehicle.body.velocity;
        status_.speed = std::hypot(velocity.x, velocity.y);

        if (progress_[player_slot] >= next_line_ && status_.on_track) {
            if (lap_start_) {
                const double lap = player_.elapsed_time - *lap_start_;
                status_.last_lap = lap;
                if (!status_.best_lap || lap < *status_.best_lap) {
                    status_.best_lap = lap;
                }
                ++status_.completed_laps;
            }
            lap_start_ = player_.elapsed_time;
            next_line_ += path.length();
        }
        status_.lap_time = lap_start_ ? player_.elapsed_time - *lap_start_ : 0.0;
    }
}
