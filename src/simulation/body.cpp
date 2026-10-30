#include "simulation/body.hpp"

namespace apex::simulation {
    void integrate(Body2D &body, const math::Vec2 &acceleration, double dt) {
        const double half_dt_squared =
                0.5 * dt * dt;

        body.position.x +=
                body.velocity.x * dt + acceleration.x * half_dt_squared;

        body.position.y +=
                body.velocity.y * dt + acceleration.y * half_dt_squared;

        body.velocity.x += acceleration.x * dt;
        body.velocity.y += acceleration.y * dt;
    }

    void update_body(Body2D &body, const math::Vec2 &direction, const MotionSettings &settings, double dt) {
        const math::Vec2 acceleration{
            direction.x * settings.acceleration_strength - body.velocity.x * settings.linear_drag,
            direction.y * settings.acceleration_strength - body.velocity.y * settings.linear_drag
        };
        integrate(body, acceleration, dt);
    }
}
