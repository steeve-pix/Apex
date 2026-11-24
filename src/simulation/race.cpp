#include "simulation/race.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "simulation/circuit_layout.hpp"
#include "simulation/vehicle_contact.hpp"


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

        double verge_width_at(const RunoffVerge &verge, double lap_fraction) {
            if (verge.width <= 0.0) {
                return 0.0;
            }
            const auto ease = [&](double distance) {
                const double t = std::clamp(distance / verge.taper_fraction, 0.0, 1.0);
                return t * t * (3.0 - 2.0 * t);
            };
            return verge.width * ease(lap_fraction - verge.start_fraction) * ease(verge.end_fraction - lap_fraction);
        }

        RaceTireContacts tire_contacts(const VehicleState &car, const WorldSettings &settings,
                                       double center_distance) {
            const double half_width = settings.track.road_width * .5;
            const double half_wheelbase = settings.vehicle.wheelbase * .5;
            constexpr double half_patch_length = .008;
            const double reach = std::hypot(half_wheelbase, front_tire.lateral_offset)
                                 + rear_tire.half_width + half_patch_length;
            RaceTireContacts contacts{};
            if (center_distance + reach < half_width) {
                return contacts;
            }

            const auto &path = *settings.track.path;
            const math::Vec2 forward{std::cos(car.heading), std::sin(car.heading)};
            const math::Vec2 left{-forward.y, forward.x};
            for (std::size_t i = 0; i < contacts.size(); ++i) {
                const bool front = i < 2;
                const auto &geometry = front ? front_tire : rear_tire;
                const double along = front ? half_wheelbase : -half_wheelbase;
                const double across = i % 2 == 0 ? geometry.lateral_offset : -geometry.lateral_offset;
                const auto &position = car.body.position;
                const auto wheel = path.project({
                    position.x + forward.x * along + left.x * across,
                    position.y + forward.y * along + left.y * across
                });
                const auto &tangent = wheel.sample.tangent;
                const double angle = car.heading + (front ? car.steering_angle : 0.0);
                const math::Vec2 wheel_forward{std::cos(angle), std::sin(angle)};
                // Project the tread footprint onto the road's lateral axis to blend split surfaces.
                const double patch = geometry.half_width * std::abs(dot(tangent, wheel_forward))
                                     + half_patch_length * std::abs(dot(tangent, {-wheel_forward.y, wheel_forward.x}));
                const double offset = std::abs(wheel.lateral_offset);
                const auto coverage = [&](double edge) {
                    return std::clamp((edge - offset + patch) / (2.0 * patch), 0.0, 1.0);
                };
                const double asphalt = coverage(half_width);
                if (asphalt == 1.0) {
                    continue;
                }
                const double bend = path.bend_at(wheel.distance_along_track);
                const bool outside = bend * wheel.lateral_offset < 0.0;
                const bool has_curb = std::abs(bend) >= (outside ? outside_curb_bend : inside_curb_bend);
                const double curb = has_curb ? coverage(half_width + curb_width) : asphalt;
                const double verge = verge_width_at(settings.track.grass_verge,
                                                    wheel.distance_along_track / path.length());
                const double paved = coverage(half_width + runoff_width - verge);
                contacts[i] = {asphalt, curb - asphalt, paved - curb, 1.0 - paved};
            }
            return contacts;
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
                const double curvature = std::abs(std::atan2(before.x * after.y - before.y * after.x,
                                                             dot(before, after)))
                                         / (2.0 * curve_probe);
                speeds[i] = std::min(race_top_speed * race_top_speed, cornering / std::max(curvature, .001));
            }
            // Work backwards so the car can brake for the next turn, including across the start line.
            for (std::size_t step = count * 2; step > 0; --step) {
                const std::size_t i = (step - 1) % count;
                speeds[i] = std::min(speeds[i], speeds[(i + 1) % count] + 2.0 * braking * spacing);
            }
            return speeds;
        }


    }

    RaceTireContacts race_tire_contacts(const VehicleState &car, const WorldSettings &settings) {
        const auto &path = *settings.track.path;
        return tire_contacts(car, settings, std::abs(path.project(car.body.position).lateral_offset));
    }

    bool tires_touch_racing_surface(const RaceTireContacts &contacts) {
        return std::ranges::any_of(contacts, [](const RaceTireContact &tire) {
            return tire.asphalt + tire.curb > 0.001;
        });
    }

    TrainingGrid::TrainingGrid(const WorldSettings &settings, const std::vector<int> &starting_slots) {
        if (starting_slots.empty() || starting_slots.size() > grid_size) {
            throw std::invalid_argument("Training grid needs between 1 and 22 cars");
        }
        auto unique_slots = starting_slots;
        std::sort(unique_slots.begin(), unique_slots.end());
        if (std::adjacent_find(unique_slots.begin(), unique_slots.end()) != unique_slots.end()) {
            throw std::invalid_argument("Training grid starting slots must be unique");
        }
        races_.reserve(starting_slots.size());
        for (int slot : starting_slots) {
            races_.emplace_back(settings, SessionMode::time_trial, slot);
        }
        contacts_.resize(races_.size());
        touching_.resize(races_.size() * races_.size());
    }

    void TrainingGrid::step(const std::vector<VehicleControls> &controls,
                            const std::vector<bool> &active, double dt) {
        if (controls.size() != races_.size() || active.size() != races_.size()
            || !std::isfinite(dt) || dt <= 0.0) {
            throw std::invalid_argument("Training grid requires one control and activity flag per car, and positive dt");
        }
        for (const auto &control : controls) {
            if (!std::isfinite(control.throttle) || !std::isfinite(control.steering)) {
                throw std::invalid_argument("Training controls must be finite");
            }
        }
        // Keep travel below the thinnest wing's contact width to avoid crossing through it.
        double max_speed = 0.0;
        for (std::size_t slot = 0; slot < races_.size(); ++slot) {
            if (active[slot]) {
                const auto velocity = races_[slot].player_.vehicle.body.velocity;
                max_speed = std::max(max_speed, std::hypot(velocity.x, velocity.y));
            }
        }
        const int substeps = static_cast<int>(std::ceil(std::max(1.0, max_speed * dt / .01)));
        const double step_dt = dt / substeps;
        std::vector<bool> hit(touching_.size());
        for (int step = 0; step < substeps; ++step) {
            for (std::size_t slot = 0; slot < races_.size(); ++slot) {
                if (active[slot]) {
                    races_[slot].step(controls[slot], false, step_dt);
                }
            }
            for (int iteration = 0; iteration < 16; ++iteration) {
                bool resolved = false;
                for (std::size_t a = 0; a < races_.size(); ++a) {
                    for (std::size_t b = a + 1; b < races_.size(); ++b) {
                        if (!active[a] || !active[b]) {
                            continue;
                        }
                        const auto impact = resolve_vehicle_contact(races_[a].player_.vehicle,
                                                                    races_[b].player_.vehicle);
                        if (!impact) {
                            continue;
                        }
                        const auto pair = a * races_.size() + b;
                        for (auto car : {a, b}) {
                            if (!touching_[pair] && !hit[pair]) {
                                ++contacts_[car].count;
                            }
                            contacts_[car].severity += *impact * *impact;
                        }
                        hit[pair] = true;
                        resolved = true;
                    }
                }
                if (!resolved) {
                    break;
                }
            }
        }
        for (std::size_t a = 0; a < races_.size(); ++a) {
            for (std::size_t b = a + 1; b < races_.size(); ++b) {
                const auto pair = a * races_.size() + b;
                touching_[pair] = active[a] && active[b]
                                  && (hit[pair] || (touching_[pair] && vehicle_contact(
                                      races_[a].player_.vehicle, races_[b].player_.vehicle, .01).has_value()));
            }
            if (active[a]) {
                auto &race = races_[a];
                const auto velocity = race.player_.vehicle.body.velocity;
                race.status_.speed = std::hypot(velocity.x, velocity.y);
                race.status_.on_track = tires_touch_racing_surface(
                    race_tire_contacts(race.player_.vehicle, race.settings_));
            }
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
        settings.track.grass_verge = {.247, .342, .016, .078};
        settings.vehicle.wheelbase = race_wheelbase;
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
        constexpr double car_clearance = 0.065;
        const double wall_limit = half_width + runoff_width - car_clearance;
        for (int slot = first_slot; slot < end_slot; ++slot) {
            cars[slot] = slot == player_slot
                             ? &player_.vehicle
                             : &opponents_[slot - (slot > player_slot)];
            auto &body = cars[slot]->body;
            projections[slot] = path.project(body.position);
            const auto &projection = projections[slot];
            if (std::abs(projection.lateral_offset) > wall_limit) {
                const double side = std::copysign(1.0, projection.lateral_offset);
                const math::Vec2 outward{-projection.sample.tangent.y * side, projection.sample.tangent.x * side};
                body.position = lane_position(projection.sample, wall_limit * side);
                projections[slot].lateral_offset = wall_limit * side;
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
                const double lookahead = race_dynamics ? .37 + .12 * std::max(0.0, speed) : .42;
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
                                                   automatic_speed_limits_[(cell + 1) % count], station - cell);
                    desired_speed = std::sqrt(limit) * (0.92 + 0.02 * (slot % 5));
                } else {
                    const auto far = path.sample_at(distance + 0.95);
                    const double bend = std::acos(std::clamp(dot(projection.sample.tangent, far.tangent), -1.0, 1.0));
                    desired_speed = (0.98 + 0.035 * (slot % 5)) / (1.0 + bend * 0.65);
                }
                if (dot(forward, delta) < 0.0) {
                    desired_speed = std::min(desired_speed, 0.40);
                }
                const double safe_gap = race_dynamics ? .38 + .16 * std::max(0.0, speed) : .38;
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
                    const double high_speed = race_dynamics
                                                  ? std::clamp((speed - 1.1) / 2.4, 0.0, 1.0)
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
            const auto tires = tire_contacts(car, settings_, std::abs(projection.lateral_offset));
            const auto grip = [](const RaceTireContact &tire) {
                return tire.asphalt + .84 * tire.curb + .94 * tire.runoff + .33 * tire.grass;
            };
            const double front_grip = (grip(tires[0]) + grip(tires[1])) * .5;
            const double rear_grip = (grip(tires[2]) + grip(tires[3])) * .5;
            local.lateral_acceleration_limit *= front_grip;
            local.max_steering_angle *= std::lerp(.70, 1.0, front_grip);
            local.lateral_grip *= std::lerp(front_grip, rear_grip, .60);
            local.motion.acceleration_strength *= rear_grip;
            local.braking_strength *= std::lerp(rear_grip, front_grip, .60);

            const double rumble = .13 + .07 * std::sin(projection.distance_along_track * 100.0);
            const auto drag = [&](const RaceTireContact &tire) {
                return rumble * tire.curb + .045 * tire.runoff + .45 * settings_.track.off_road_drag * tire.grass;
            };
            const double left_drag = (drag(tires[0]) + drag(tires[2])) * .5;
            const double right_drag = (drag(tires[1]) + drag(tires[3])) * .5;
            local.motion.linear_drag += (left_drag + right_drag) * .5;
            const double forward_speed = dot(car.body.velocity, {std::cos(car.heading), std::sin(car.heading)});
            // Unequal rolling resistance pulls the car toward the tires with more drag.
            const double pull = std::clamp(.40 * forward_speed * (left_drag - right_drag), -.55, .55);
            const double grade = path.elevation_at(projection.distance_along_track).grade;
            constexpr double gravity = 9.81 / (race_speed_to_kmh / 3.6);
            const double downhill = -gravity * grade / (1.0 + grade * grade);
            const auto tangent = projection.sample.tangent;
            update_vehicle(car, controls, local, dt, {downhill * tangent.x, downhill * tangent.y}, pull);
        }
        player_.elapsed_time += dt;

        solve_vehicle_contacts(std::span{cars}.subspan(first_slot, end_slot - first_slot));

        for (int slot = first_slot; slot < end_slot; ++slot) {
            progress_[slot] += std::remainder(projections[slot].distance_along_track - progress_[slot], path.length());
        }
        status_.position = 1 + static_cast<int>(std::count_if(progress_.begin() + first_slot,
                                                              progress_.begin() + end_slot,
                                                              [this](double progress) {
                                                                  return progress > progress_[player_slot];
                                                              }));
        const auto &centerline = projections[player_slot].sample.position;
        const auto &position = player_.vehicle.body.position;
        const double center_distance_bound = std::hypot(position.x - centerline.x, position.y - centerline.y);
        status_.on_track = tires_touch_racing_surface(tire_contacts(player_.vehicle, settings_, center_distance_bound));
        const auto velocity = player_.vehicle.body.velocity;
        status_.speed = std::hypot(velocity.x, velocity.y);

        // A small margin keeps the timer running through bounces off the barrier.
        constexpr double wall_margin = .018;
        constexpr double wall_stall_speed = 8.0 / race_speed_to_kmh;
        constexpr double wall_stall_seconds = 2.0;
        const bool near_wall = std::abs(projections[player_slot].lateral_offset) >= wall_limit - wall_margin;
        if (near_wall && status_.speed < wall_stall_speed) {
            slow_at_wall_time_ += dt;
        } else {
            slow_at_wall_time_ = 0.0;
        }
        status_.stuck_on_wall = slow_at_wall_time_ >= wall_stall_seconds;

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
        status_.lap_distance = lap_start_ ? progress_[player_slot] - (next_line_ - path.length()) : 0.0;
    }
}
