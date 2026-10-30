#include <simulation/body.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

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
}

int main() {
    try {
        using apex::math::Vec2;
        using apex::simulation::Body2D;
        using apex::simulation::integrate;
        using apex::simulation::MotionSettings;
        using apex::simulation::update_body;

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

        std::cout << "Physics checks passed.\n";

        return EXIT_SUCCESS;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }
}
