#pragma once
#include "math/vec2.hpp"

namespace apex::simulation {
    struct Body2D {
        math::Vec2 position{};
        math::Vec2 velocity{};
    };

    struct MotionSettings {
        double acceleration_strength{1.0};
        double linear_drag{1.5};
        double quadratic_drag{};

        [[nodiscard]] double drag_at_speed(double speed) const { return linear_drag + quadratic_drag * speed; }
    };
    void integrate(Body2D &body, const math::Vec2 &acceleration, double dt);

    void update_body(Body2D& body, const math::Vec2 &direction,const MotionSettings &settings,double dt);

}
