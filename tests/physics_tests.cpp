#include <simulation/body.hpp>
#include <simulation/vehicle.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <numbers>
#include <array>

#include "simulation/track.hpp"
#include "simulation/world.hpp"

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
        apex::simulation::WorldSetting settings{};
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
        test_surface_drag();
        test_world();
        test_track_observation();

        std::cout << "Physics checks passed.\n";

        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
