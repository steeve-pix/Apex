#include <simulation/body.hpp>
#include <simulation/vehicle.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <numbers>
#include <array>

#include "simulation/driver.hpp"
#include "simulation/race.hpp"
#include "simulation/sensors.hpp"
#include "simulation/track.hpp"
#include "simulation/track_boundaries.hpp"
#include "simulation/world.hpp"
#include "simulation/vehicle_contact.hpp"

namespace {
    void expect_near(double actual, double expected, const char *label) {
        constexpr double tolerance = 1e-9;

        if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
            std::cerr << label << ": expected " << expected << ", got " << actual << '\n';
            throw std::runtime_error("Physics check failed");
        }
    }

    void expect(bool condition, const char *message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    void test_circuit_runoff_boundaries() {
        const auto settings = apex::simulation::make_race_settings();
        constexpr double barrier_thickness = 0.02;
        const auto wall_edge = settings.track.road_width * 0.5
                             + apex::simulation::runoff_width + barrier_thickness;
        const auto boundaries = apex::simulation::make_track_boundaries(*settings.track.path, wall_edge);
        expect(boundaries.left.size() == settings.track.path->points().size(),
               "The circuit must support runoff and barrier offsets");
    }

    void test_sunset_elevation() {
        using namespace apex::simulation;
        const auto settings = make_race_settings();
        const auto &path = *settings.track.path;
        const auto at = [&](double fraction) { return path.elevation_at(fraction * path.length()); };
        expect_near(at(start_line_distance / path.length()).height, 0.0, "Starting grid stays level");
        expect(at(.228).height > .7 && at(.335).height < 0.0, "Turn One crests above the lakeside section");
        expect(at(.18).grade > 0.1 && at(.28).grade < -0.1, "The road climbs into Turn One and descends toward the lake");
        expect_near(at(.228).grade, 0.0, "The grade eases through the crest");
        expect_near(at(-.72).height, at(.28).height, "Elevation wraps around the circuit");

        const auto speed_on_grade = [&](double rise) {
            auto trial = settings;
            const auto points = path.points();
            trial.track.path = std::make_shared<const TrackPath>(
                std::vector<apex::math::Vec2>{points.begin(), points.end()},
                std::vector<TrackElevationPoint>{{0,0}, {.02,0}, {.11,rise}, {.20,0}, {1,0}});
            Race race(trial);
            for (int tick = 0; tick < 120; ++tick) {
                race.step({.throttle = .65}, false, 1.0 / 120.0);
            }
            return race.status().speed;
        };
        const double flat_speed = speed_on_grade(0.0);
        expect(speed_on_grade(.55) < flat_speed - .03 && speed_on_grade(-.55) > flat_speed + .03,
               "At the same throttle, a car gains speed downhill and loses acceleration uphill");
    }

    void test_turn_one_surface_recovery() {
        using namespace apex::simulation;
        const auto settings = make_race_settings();
        auto paved_settings = settings;
        paved_settings.track.grass_verge = {};
        const auto &path = *settings.track.path;
        constexpr double dt = 1.0 / 120.0;

        const auto approach = [&](const WorldSettings &setup, double fraction) {
            Race race(setup);
            for (int tick = 0; tick < 2000; ++tick) {
                if (path.project(race.player().vehicle.body.position).distance_along_track >= fraction * path.length()) {
                    return race;
                }
                race.step({}, true, dt);
            }
            throw std::runtime_error("Autopilot did not reach the Turn One descent");
        };

        auto race = approach(settings, .249);
        auto paved = approach(paved_settings, .249);
        int grass_tick = -1;
        bool clipped_curb = false;
        bool recovered = false;
        double speed_loss = 0.0;
        for (int tick = 0; tick < 330; ++tick) {
            const bool correct = grass_tick >= 0 && tick > grass_tick + 8;
            constexpr VehicleControls drift{.throttle = .30, .steering = .035};
            race.step(drift, correct, dt);
            paved.step(drift, correct, dt);
            expect(race.status().on_track, "Clipping the kerb and touching grass with one tire must allow recovery");
            expect(!race.status().stuck_on_wall, "A recovering driver must not be flagged as stuck on the wall");

            const auto tires = race_tire_contacts(race.player().vehicle, settings);
            clipped_curb |= std::ranges::any_of(tires, [](const auto &tire) { return tire.curb > .04; });
            if (grass_tick < 0 && std::ranges::count_if(tires, [](const auto &tire) { return tire.grass > .15; }) == 1) {
                expect(clipped_curb, "The car should clip a kerb before reaching the downhill grass verge");
                grass_tick = tick;
            }
            if (grass_tick >= 0) {
                speed_loss = std::max(speed_loss, paved.status().speed - race.status().speed);
            }
            if (correct && std::ranges::all_of(tires, [](const auto &tire) { return tire.asphalt > .9975; })) {
                recovered = true;
                break;
            }
        }
        expect(grass_tick >= 0 && recovered, "A car can recover after one tire touches grass on the downhill exit");
        expect(speed_loss > .04, "Grass contact must slow the car more than paved runoff under the same controls");

        auto pinned = approach(settings, .249);
        constexpr VehicleControls into_wall{.throttle = .50, .steering = -.65};
        for (int tick = 0; tick < 120; ++tick) {
            pinned.step(into_wall, false, dt);
            expect(!pinned.status().stuck_on_wall, "The wall rule must give drivers time to recover");
        }
        for (int tick = 0; tick < 360 && !pinned.status().stuck_on_wall; ++tick) {
            pinned.step(into_wall, false, dt);
        }
        expect(pinned.status().on_track && pinned.status().stuck_on_wall,
               "A car creeping against the wall must be flagged even while a tire is still on the track");

        auto excursion = approach(settings, .249);
        int excursion_grass_tick = -1;
        for (int tick = 0; tick < 240 && excursion.status().on_track; ++tick) {
            const bool correct = excursion_grass_tick >= 0 && tick > excursion_grass_tick + 8;
            excursion.step({.throttle = .30, .steering = -.02}, correct, dt);
            if (excursion_grass_tick < 0) {
                const auto tires = race_tire_contacts(excursion.player().vehicle, settings);
                if (std::ranges::any_of(tires, [](const auto &tire) { return tire.grass > .15; })) {
                    excursion_grass_tick = tick;
                }
            }
        }
        expect(!excursion.status().on_track, "A car that takes all four tires off the racing surface must be off track");
    }

    void test_race_car_pace() {
        using namespace apex::simulation;
        const auto settings = make_race_settings();
        constexpr int ticks_per_second = 120;
        constexpr double dt = 1.0 / ticks_per_second;

        VehicleState straight{};
        for (int tick = 0; tick < 60 * ticks_per_second; ++tick) {
            update_vehicle(straight, {.throttle = 1.0}, settings.vehicle, dt);
        }
        const double top_speed_kmh = straight.body.velocity.x * race_speed_to_kmh;
        expect(std::abs(top_speed_kmh - 350.0) < 0.5, "Race car must reach 350 km/h on a long straight");

        for (int tick = 0; tick < 2 * ticks_per_second && straight.body.velocity.x > 1e-9; ++tick) {
            update_vehicle(straight, {.throttle = -1.0}, settings.vehicle, dt);
        }
        expect_near(straight.body.velocity.x, 0.0, "Race brakes must stop the car without overshooting into reverse");

        Race race(settings);
        double peak_speed = 0.0;
        for (int tick = 0; tick < 70 * ticks_per_second && race.status().completed_laps == 0; ++tick) {
            race.step({}, true, dt);
            expect(race.status().on_track, "Autopilot must brake early enough to stay on Sunset Circuit");
            peak_speed = std::max(peak_speed, race.status().speed * race_speed_to_kmh);
        }
        expect(race.status().completed_laps == 1 && peak_speed > 200.0,
               "Autopilot must complete a lap at the new race pace");
    }

    void test_lateral_grip() {
        using apex::math::Vec2;
        using apex::simulation::VehicleControls;
        using apex::simulation::VehicleSettings;
        using apex::simulation::VehicleState;
        using apex::simulation::update_vehicle;

        VehicleSettings settings{};
        settings.motion.acceleration_strength = 0.0;
        settings.motion.linear_drag = 0.0;
        settings.lateral_grip = 8.0;

        VehicleState initial{};
        initial.heading = std::numbers::pi_v<double> / 2.0;
        initial.body.velocity = Vec2{-1.0, 3.0};

        constexpr double dt = 1.0 / 120.0;
        const VehicleControls no_input{};

        auto gripping = initial;

        update_vehicle(gripping, no_input, settings, dt);
        expect(gripping.body.velocity.x > -1.0 && gripping.body.velocity.x < 0.0,
               "Grip must reduce sideways speed without reversing it");
        expect_near(gripping.body.velocity.y, 3.0, "Grip must preserve forward velocity");

        // Disabling grip must preserve both velocity components.
        auto grip_disabled = initial;
        settings.lateral_grip = 0.0;

        update_vehicle(grip_disabled, no_input, settings, dt);
        expect_near(grip_disabled.body.velocity.x, -1.0, "Zero-grip sideways velocity");
        expect_near(grip_disabled.body.velocity.y, 3.0, "Zero-grip forward velocity");
    }

    void test_speed_dependent_steering() {
        using apex::math::Vec2;
        using apex::simulation::VehicleControls;
        using apex::simulation::VehicleSettings;
        using apex::simulation::VehicleState;
        using apex::simulation::update_vehicle;

        VehicleSettings settings{};
        settings.wheelbase = 1.0;
        settings.max_steering_angle =
                std::numbers::pi_v<double> / 4.0;

        settings.motion.linear_drag = 0.0;
        settings.lateral_grip = 0.0;

        struct SteeringCase {
            double speed;
            double steering;
            double expected_heading;
            const char *label;
        };

        constexpr std::array cases{
            SteeringCase{0.0, 1.0, 0.0, "Stationary"},
            SteeringCase{2.0, 1.0, 0.5, "Forward left"},
            SteeringCase{4.0, 1.0, 1.0, "Double speed"},
            SteeringCase{-2.0, 1.0, -0.5, "Reverse left"},
            SteeringCase{2.0, -1.0, -0.5, "Forward right"},
            SteeringCase{-2.0, -1.0, 0.5, "Reverse right"},
            SteeringCase{2.0, 3.0, 0.5, "Clamped steering"}
        };

        for (const auto &test_case: cases) {
            VehicleState vehicle{};
            vehicle.body.velocity = Vec2{test_case.speed, 0.0};

            const VehicleControls controls{
                .steering = test_case.steering
            };

            update_vehicle(vehicle, controls, settings, 0.25);

            expect_near(
                vehicle.heading,
                test_case.expected_heading,
                test_case.label
            );
        }
    }

    void test_track_boundaries() {
        using apex::simulation::is_on_track;
        using apex::simulation::TrackSettings;

        const TrackSettings settings{
            .centerline_radius = 5.0,
            .road_width = 2.0
        };

        expect(is_on_track({5.0, 0.0}, settings), "Centerline must be on track");
        expect(is_on_track({4.0, 0.0}, settings), "Inner edge must be on track");
        expect(is_on_track({6.0, 0.0}, settings), "Outer edge must be on track");
        expect(!is_on_track({3.99, 0.0}, settings), "Inside the inner edge must be off track");
        expect(!is_on_track({6.01, 0.0}, settings), "Outside the outer edge must be off track");
        expect(!is_on_track({0.0, 0.0}, settings), "Track center must be off track");
        expect(is_on_track({-3.0, -4.0}, settings), "Diagonal position must use radial distance");
    }

    void test_world() {
        apex::simulation::WorldSettings settings{};
        settings.track.centerline_radius = 5.0;
        settings.track.road_width = 2.0;
        settings.vehicle.motion.acceleration_strength = 2.0;
        settings.vehicle.motion.linear_drag = 0.0;

        auto world = apex::simulation::make_initial_world(settings);
        expect_near(world.vehicle.body.position.x, 5.0, "World spawn x");
        expect_near(world.vehicle.body.position.y, 0.0, "World spawn y");
        expect_near(world.vehicle.heading, std::numbers::pi_v<double> / 2.0,
                    "World spawn heading");
        expect_near(world.elapsed_time, 0.0, "Initial simulation time");

        const apex::simulation::VehicleControls controls{.throttle = 1.0};
        apex::simulation::step_world(world, controls, settings, 0.25);
        expect_near(world.elapsed_time, 0.25, "World step advances time");
        expect_near(world.vehicle.body.position.y, 0.0625, "World step advances position");
        expect_near(world.vehicle.body.velocity.y, 0.5, "World step advances velocity");

        world = apex::simulation::make_initial_world(settings);
        expect_near(world.vehicle.body.position.x, 5.0, "Reset spawn x");
        expect_near(world.vehicle.body.position.y, 0.0, "Reset spawn y");
        expect_near(world.vehicle.body.velocity.x, 0.0, "Reset velocity x");
        expect_near(world.vehicle.body.velocity.y, 0.0, "Reset velocity y");
        expect_near(world.elapsed_time, 0.0, "Reset simulation time");
    }

    void test_surface_drag() {
        using apex::simulation::TrackSettings;
        using apex::simulation::VehicleSettings;
        using apex::simulation::vehicle_settings_at;

        const TrackSettings track{
            .centerline_radius = 5.0,
            .road_width = 2.0,
            .off_road_drag = 4.0
        };

        VehicleSettings base{};
        base.motion.linear_drag = 1.5;

        const auto road = vehicle_settings_at({5.0, 0.0}, base, track);
        const auto infield = vehicle_settings_at({0.0, 0.0}, base, track);
        const auto outside = vehicle_settings_at({7.0, 0.0}, base, track);
        const auto returned = vehicle_settings_at({5.0, 0.0}, base, track);

        expect_near(road.motion.linear_drag, 1.5,
                    "Road must preserve normal drag");
        expect_near(infield.motion.linear_drag, 5.5,
                    "Infield must add surface drag");
        expect_near(outside.motion.linear_drag, 5.5,
                    "Outside must add surface drag");
        expect_near(returned.motion.linear_drag, 1.5,
                    "Returning to road must restore normal drag");
        expect_near(base.motion.linear_drag, 1.5,
                    "Surface selection must not modify base settings");

        expect_near(outside.motion.acceleration_strength,
                    base.motion.acceleration_strength,
                    "Surface drag must preserve engine acceleration");
    }

    void test_track_observation() {
        using apex::simulation::observe_track;
        using apex::simulation::TrackSettings;
        using apex::simulation::VehicleState;

        constexpr double pi = std::numbers::pi_v<double>;
        const TrackSettings track{.centerline_radius = 5.0};

        VehicleState vehicle{};
        vehicle.heading = pi / 2.0;

        vehicle.body.position = {5.0, 0.0};
        auto observation = observe_track(vehicle, track);

        expect_near(observation.centerline_error, 0.0,
                    "Centerline error at road center");
        expect_near(observation.heading_error, 0.0,
                    "Counterclockwise tangent alignment");

        vehicle.body.position = {6.0, 0.0};
        observation = observe_track(vehicle, track);
        expect_near(observation.centerline_error, 1.0,
                    "Outside error must be positive");

        vehicle.body.position = {4.0, 0.0};
        observation = observe_track(vehicle, track);
        expect_near(observation.centerline_error, -1.0,
                    "Inside error must be negative");

        vehicle.body.position = {5.0, 0.0};
        vehicle.heading = 0.0;
        observation = observe_track(vehicle, track);
        expect_near(observation.heading_error, pi / 2.0,
                    "Desired left turn must be positive");

        vehicle.body.position = {0.0, 5.0};
        vehicle.heading = -pi + 0.1;
        observation = observe_track(vehicle, track);
        expect_near(observation.heading_error, -0.1,
                    "Heading error must wrap across pi");

        vehicle.heading = pi / 2.0;
        vehicle.body.velocity = {9.0, -2.0};
        observation = observe_track(vehicle, track);
        expect_near(observation.forward_speed, -2.0,
                    "Forward speed must exclude sideways velocity");

        vehicle.body.position = {0.0, 0.0};
        bool rejected_origin = false;

        try {
            const auto unused = observe_track(vehicle, track);
            (void) unused;
        } catch (const std::invalid_argument &) {
            rejected_origin = true;
        }

        expect(rejected_origin, "Observation must reject undefined heading at origin");
    }

    void test_automatic_driver() {
        using apex::simulation::drive_automatically;
        using apex::simulation::DriverSettings;
        using apex::simulation::TrackObservation;
        using apex::simulation::TrackSettings;
        using apex::simulation::VehicleSettings;

        const VehicleSettings vehicle{};
        const TrackSettings track{};
        const DriverSettings driver{};

        const auto controls_for = [&](const TrackObservation &observation) {
            return drive_automatically(observation, vehicle, track, driver);
        };

        const auto centered = controls_for(TrackObservation{});

        expect(centered.steering > 0.0 && centered.steering < 1.0,
               "Circular track must require left steering");
        expect_near(centered.throttle, 1.0,
                    "Initial throttle must clamp to full power");

        const auto outside = controls_for(
            TrackObservation{.centerline_error = 0.02}
        );
        const auto inside = controls_for(
            TrackObservation{.centerline_error = -0.02}
        );

        expect(outside.steering > centered.steering,
               "Outside position must increase left steering");
        expect(inside.steering < centered.steering,
               "Inside position must reduce left steering");

        const auto left = controls_for(
            TrackObservation{.heading_error = 10.0}
        );
        const auto right = controls_for(
            TrackObservation{.heading_error = -10.0}
        );

        expect_near(left.steering, 1.0, "Left steering limit");
        expect_near(right.steering, -1.0, "Right steering limit");

        const auto fast = controls_for(
            TrackObservation{.forward_speed = driver.target_speed + 0.1}
        );

        expect_near(fast.throttle, 0.0,
                    "Above target speed must release throttle");
    }

    void test_track_rays() {
        using apex::math::Vec2;
        using apex::simulation::cast_track_ray;
        using apex::simulation::TrackSettings;

        constexpr double pi = std::numbers::pi_v<double>;
        const TrackSettings track{
            .centerline_radius = 5.0,
            .road_width = 2.0
        };

        struct RayCase {
            Vec2 origin;
            double angle{};
            double range{};
            double expected_distance{};
            bool expected_hit{};
            const char *label{};
        };

        const std::array cases{
            RayCase{{5.0, 0.0}, 0.0, 10.0, 1.0, true, "Outward"},
            RayCase{{5.0, 0.0}, pi, 10.0, 1.0, true, "Inward"},
            RayCase{
                {5.0, 0.0}, pi / 2.0, 10.0,
                std::sqrt(11.0), true, "Along road"
            },
            RayCase{{5.0, 0.0}, 0.0, 0.5, 0.5, false, "Beyond range"},
            RayCase{{5.0, 0.0}, 0.0, 1.0, 1.0, true, "At range limit"},
            RayCase{{7.0, 0.0}, 0.0, 10.0, 10.0, false, "Behind ray"},
            RayCase{{-3.0, 4.0}, 0.0, 10.0, 3.0, true, "Tangent"}
        };

        for (const auto &test_case: cases) {
            const auto reading = cast_track_ray(test_case.origin, test_case.angle, track, test_case.range);

            expect(reading.hit == test_case.expected_hit, test_case.label);
            expect_near(reading.distance, test_case.expected_distance, test_case.label);
        }
    }

    void test_sensor_fan() {
        using apex::simulation::sense_track;
        using apex::simulation::SensorSettings;
        using apex::simulation::TrackSettings;
        using apex::simulation::VehicleState;

        constexpr double pi = std::numbers::pi_v<double>;
        const TrackSettings track{
            .centerline_radius = 5.0,
            .road_width = 2.0
        };

        SensorSettings sensors{};
        sensors.max_distance = 10.0;

        VehicleState vehicle{};
        vehicle.body.position = {5.0, 0.0};
        vehicle.heading = pi / 2.0;

        const auto initial = sense_track(vehicle, track, sensors);

        expect(initial[0].hit && initial[3].hit && initial[6].hit,
               "Right, forward, and left sensors must hit");
        expect_near(initial[0].distance, 1.0, "Right sensor");
        expect_near(initial[3].distance, std::sqrt(11.0), "Forward sensor");
        expect_near(initial[6].distance, 1.0, "Left sensor");

        // Rotate both position and heading by a quarter turn.
        vehicle.body.position = {0.0, 5.0};
        vehicle.heading = pi;

        const auto rotated = sense_track(vehicle, track, sensors);

        for (std::size_t i = 0; i < rotated.size(); ++i) {
            expect(rotated[i].hit == initial[i].hit,
                   "Rotation must preserve sensor hit status");
            expect_near(rotated[i].distance, initial[i].distance,
                        "Rotation must preserve sensor distance");
        }

        sensors.max_distance = 0.25;
        const auto limited = sense_track(vehicle, track, sensors);

        for (const auto &reading: limited) {
            expect(!reading.hit, "Short-range sensor must not reach an edge");
            expect_near(reading.distance, 0.25, "Sensor range limit");
        }
    }
    void test_vehicle_footprint() {
        using namespace apex::simulation;
        VehicleState a;
        VehicleState b;
        // Front wing corner against the other car's rear wing corner.
        b.body.position = {.328, .10};
        expect(vehicle_contact(a, b).has_value(), "Wing corners must collide outside the old ellipse");
        a.body.velocity = {1.0, 0.0};
        const auto impact = resolve_vehicle_contact(a, b);
        expect(impact && *impact > 0.0, "Wing impact must slow the closing velocity");
        expect_near(a.body.velocity.x + b.body.velocity.x, 1.0, "Impact conserves linear momentum");
        std::array<VehicleState *, 2> pair{&a, &b};
        solve_vehicle_contacts(pair);
        expect(!vehicle_contact(a, b), "Wing overlap must be separated");

        a = {};
        b = {};
        b.body.position = {0.0, .14};
        expect(!vehicle_contact(a, b), "Side-by-side cars with a visible gap must remain separate");
        b.body.position.y = .12;
        expect(vehicle_contact(a, b).has_value(), "Side-by-side tire overlap must collide");
        solve_vehicle_contacts(pair);
        expect(!vehicle_contact(a, b), "Side contact must be separated");

        // Rotate the same wing-corner impact by 90 degrees.
        a = {};
        b = {};
        a.heading = b.heading = std::numbers::pi / 2.0;
        b.body.position = {-.10, .328};
        const auto rotated = vehicle_contact(a, b);
        expect(rotated && std::abs(rotated->normal.y) > .99, "Contact normal must rotate with the cars");

        std::array<VehicleState, 3> pileup{};
        pileup[1].body.position = {.30, 0.0};
        pileup[2].body.position = {.60, 0.0};
        std::array<VehicleState *, 3> field{&pileup[0], &pileup[1], &pileup[2]};
        solve_vehicle_contacts(field);
        for (std::size_t i = 0; i < field.size(); ++i) {
            for (std::size_t j = i + 1; j < field.size(); ++j) {
                const auto contact = vehicle_contact(*field[i], *field[j]);
                expect(!contact || contact->penetration < 1e-6, "Pileup solver must remove chain overlaps");
            }
        }
    }

    void test_shared_training_grid() {
        using namespace apex::simulation;
        const auto settings = make_race_settings();
        TrainingGrid single(settings, {0});
        Race isolated(settings, SessionMode::time_trial, 0);
        const VehicleControls throttle{.throttle = 0.5};
        constexpr double dt = 1.0 / 120.0;
        for (int tick = 0; tick < 120; ++tick) {
            single.step({throttle}, {true}, dt);
            isolated.step(throttle, false, dt);
        }
        expect_near(single.race(0).player().vehicle.body.position.x,
                    isolated.player().vehicle.body.position.x, "Shared grid single-car x");
        expect_near(single.race(0).player().vehicle.body.position.y,
                    isolated.player().vehicle.body.position.y, "Shared grid single-car y");

        TrainingGrid pair(settings, {0, 2});
        bool rejected = false;
        try {
            pair.step({throttle}, {true, true}, dt);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        expect(rejected, "Shared grid must reject mismatched inputs");
        expect_near(pair.race(0).player().elapsed_time, 0.0, "Rejected tick must be atomic");
        for (int tick = 0; tick < 240; ++tick) {
            pair.step({VehicleControls{}, VehicleControls{.throttle = 1.0}}, {true, true}, dt);
        }
        expect(pair.contacts(0).count > 0 && pair.contacts(1).count > 0,
               "A rear impact must register on both cars");
        expect(pair.contacts(0).count == 1, "Sustained contact must count as one event");
        expect(pair.contacts(0).severity > 0.0, "A moving impact must register severity");
        expect_near(pair.contacts(0).severity, pair.contacts(1).severity,
                    "Contact severity must be shared symmetrically");
        const double elapsed = pair.race(0).player().elapsed_time;
        pair.step({throttle, throttle}, {false, false}, dt);
        expect_near(pair.race(0).player().elapsed_time, elapsed, "Retired cars must stop stepping");
    }
}

int main() {
    try {
        using apex::math::Vec2;
        using apex::simulation::Body2D;
        using apex::simulation::integrate;
        using apex::simulation::MotionSettings;
        using apex::simulation::update_body;
        using apex::simulation::VehicleState;
        using apex::simulation::VehicleControls;
        using apex::simulation::VehicleSettings;
        using apex::simulation::update_vehicle;

        Body2D body{Vec2{1.0, -1.0}, Vec2{2.0, -3.0}};

        integrate(body, Vec2{3.0, 2.0}, 2.0);

        expect_near(body.position.x, 11.0, "Position x");
        expect_near(body.position.y, -3.0, "Position y");
        expect_near(body.velocity.x, 8.0, "Velocity x");
        expect_near(body.velocity.y, 1.0, "Velocity y");

        // One second in one step versus 120 smaller steps.

        Body2D single_step{};
        Body2D multiple_steps{};
        const Vec2 acceleration{1.5, -0.5};

        integrate(single_step, acceleration, 1.0);

        constexpr int step_count = 120;
        constexpr double step_duration = 1.0 / step_count;

        for (int step = 0; step < step_count; ++step) {
            integrate(multiple_steps, acceleration, step_duration);
        }

        expect_near(multiple_steps.position.x, single_step.position.x, "Step consistency: position x");
        expect_near(multiple_steps.position.y, single_step.position.y, "Step consistency: position y");
        expect_near(multiple_steps.velocity.x, single_step.velocity.x, "Step consistency: velocity x");
        expect_near(multiple_steps.velocity.y, single_step.velocity.y, "Step consistency: velocity y");

        // Drag slows a moving body without reversing it.
        Body2D coasting{Vec2{}, Vec2{2.0, -1.0}};

        const MotionSettings drag_settings{.acceleration_strength = 0.0, .linear_drag = 1.5};

        const double initial_speed =
                std::hypot(coasting.velocity.x, coasting.velocity.y);

        double previous_speed = initial_speed;

        for (int step = 0; step < step_count; ++step) {
            update_body(coasting, Vec2{}, drag_settings, step_duration);

            const double speed =
                    std::hypot(coasting.velocity.x, coasting.velocity.y);

            expect(std::isfinite(speed) && speed < previous_speed, "Drag must reduce speed on every step");
            expect(coasting.velocity.x >= 0.0 && coasting.velocity.y <= 0.0, "Drag alone must not reverse velocity");

            previous_speed = speed;
        }

        expect(previous_speed < initial_speed * 0.25, "Expected speed below one quarter after one second");

        Body2D resting{};

        update_body(resting, Vec2{}, drag_settings, step_duration);

        expect_near(resting.position.x, 0.0, "Resting position x");
        expect_near(resting.position.y, 0.0, "Resting position y");
        expect_near(resting.velocity.x, 0.0, "Resting velocity x");
        expect_near(resting.velocity.y, 0.0, "Resting velocity y");

        VehicleSettings vehicle_settings{};
        vehicle_settings.wheelbase = 1.0;
        vehicle_settings.max_steering_angle =
                std::numbers::pi_v<double> / 4.0;
        vehicle_settings.motion.acceleration_strength = 2.0;
        vehicle_settings.motion.linear_drag = 0.0;

        // Two radians per second for a quarter second gives 0.5 radians.
        VehicleState turning{};
        turning.body.velocity = Vec2{2.0, 0.0};
        const VehicleControls turn_left{.steering = 1.0};

        update_vehicle(turning, turn_left, vehicle_settings, 0.25);

        expect_near(turning.heading, 0.5, "Steering over elapsed time");

        // Zero elapsed time must leave the heading unchanged.
        const double previous_heading = turning.heading;

        update_vehicle(turning, turn_left, vehicle_settings, 0.0);

        expect_near(turning.heading, previous_heading, "Zero-time steering");

        // Facing upward must produce upward acceleration.
        VehicleState accelerating{};
        accelerating.heading = std::numbers::pi_v<double> / 2.0;

        const VehicleControls throttle{.throttle = 1.0};

        update_vehicle(accelerating, throttle, vehicle_settings, 0.5);

        expect_near(accelerating.body.velocity.x, 0.0, "Forward velocity x");
        expect_near(accelerating.body.velocity.y, 1.0, "Forward velocity y");
        expect_near(accelerating.body.position.x, 0.0, "Forward position x");
        expect_near(accelerating.body.position.y, 0.25, "Forward position y");

        VehicleState parked{};

        update_vehicle(parked, turn_left, vehicle_settings, 0.25);

        expect_near(parked.heading, 0.0, "Stationary steering must not rotate the vehicle");

        test_lateral_grip();
        test_speed_dependent_steering();
        test_track_boundaries();
        test_circuit_runoff_boundaries();
        test_sunset_elevation();
        test_turn_one_surface_recovery();
        test_race_car_pace();
        test_surface_drag();
        test_world();
        test_track_observation();
        test_automatic_driver();
        test_track_rays();
        test_sensor_fan();
        test_vehicle_footprint();
        test_shared_training_grid();

        std::cout << "Physics checks passed.\n";

        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
