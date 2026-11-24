#include "simulation/vehicle_contact.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include "simulation/race.hpp"

namespace apex::simulation {
    namespace {
        struct Box {
            math::Vec2 center;
            math::Vec2 forward;
            math::Vec2 left;
            double length;
            double width;
        };

        double dot(math::Vec2 a, math::Vec2 b) {
            return a.x * b.x + a.y * b.y;
        }

        std::array<Box, 9> footprint(const VehicleState &car) {
            const math::Vec2 forward{std::cos(car.heading), std::sin(car.heading)};
            const math::Vec2 left{-forward.y, forward.x};
            const auto box = [&](double x, double y, double length, double width, double steering = 0.0) {
                const double angle = car.heading + steering;
                const math::Vec2 axis{std::cos(angle), std::sin(angle)};
                return Box{{car.body.position.x + forward.x * x + left.x * y,
                            car.body.position.y + forward.y * x + left.y * y},
                           axis, {-axis.y, axis.x}, length, width};
            };
            // Half extents follow the authored mesh in car_mesh.cpp.
            return {{box(-.016, 0, .068, .049),
                     box(-.1185, 0, .0345, .026),
                     box(.112, 0, .060, .013),
                     box(.148, 0, .023, .0654),
                     box(-.143, 0, .021, .0481),
                     box(race_wheelbase * .5, front_tire.lateral_offset, .0224,
                         front_tire.half_width * 1.04, car.steering_angle),
                     box(race_wheelbase * .5, -front_tire.lateral_offset, .0224,
                         front_tire.half_width * 1.04, car.steering_angle),
                     box(-race_wheelbase * .5, rear_tire.lateral_offset, .0236, rear_tire.half_width * 1.04),
                     box(-race_wheelbase * .5, -rear_tire.lateral_offset, .0236, rear_tire.half_width * 1.04)}};
        }

        std::optional<VehicleContact> box_contact(const Box &a, const Box &b, double margin) {
            const math::Vec2 delta{b.center.x - a.center.x, b.center.y - a.center.y};
            VehicleContact contact{{}, std::numeric_limits<double>::infinity()};
            for (const auto axis : {a.forward, a.left, b.forward, b.left}) {
                const auto radius = [axis](const Box &box) {
                    return box.length * std::abs(dot(box.forward, axis))
                           + box.width * std::abs(dot(box.left, axis));
                };
                const double separation = dot(delta, axis);
                const double overlap = radius(a) + radius(b) - std::abs(separation);
                if (overlap < -margin) {
                    return std::nullopt;
                }
                if (overlap < contact.penetration) {
                    const double sign = separation < 0.0 ? -1.0 : 1.0;
                    contact = {{axis.x * sign, axis.y * sign}, overlap};
                }
            }
            return contact;
        }
    }

    std::optional<VehicleContact> vehicle_contact(const VehicleState &a, const VehicleState &b, double margin) {
        // .19 encloses every part at any steering angle.
        if (std::hypot(b.body.position.x - a.body.position.x,
                       b.body.position.y - a.body.position.y) > .38 + margin) {
            return std::nullopt;
        }
        const auto a_boxes = footprint(a);
        const auto b_boxes = footprint(b);
        std::optional<VehicleContact> deepest;
        for (const auto &a_box : a_boxes) {
            for (const auto &b_box : b_boxes) {
                const auto contact = box_contact(a_box, b_box, margin);
                if (contact && (!deepest || contact->penetration > deepest->penetration)) {
                    deepest = contact;
                }
            }
        }
        return deepest;
    }

    std::optional<double> resolve_vehicle_contact(VehicleState &a, VehicleState &b) {
        const auto contact = vehicle_contact(a, b);
        if (!contact) {
            return std::nullopt;
        }
        const auto normal = contact->normal;
        const double correction = (contact->penetration + 1e-7) * .5;
        a.body.position.x -= normal.x * correction;
        a.body.position.y -= normal.y * correction;
        b.body.position.x += normal.x * correction;
        b.body.position.y += normal.y * correction;
        const math::Vec2 relative{a.body.velocity.x - b.body.velocity.x,
                                  a.body.velocity.y - b.body.velocity.y};
        const double closing_speed = std::max(0.0, dot(relative, normal));
        const double impulse = closing_speed * .52;
        a.body.velocity.x -= normal.x * impulse;
        a.body.velocity.y -= normal.y * impulse;
        b.body.velocity.x += normal.x * impulse;
        b.body.velocity.y += normal.y * impulse;
        return closing_speed;
    }

    void solve_vehicle_contacts(std::span<VehicleState *> cars) {
        // Revisit the whole field because separating one pair can touch a third car.
        for (int iteration = 0; iteration < 16; ++iteration) {
            bool touching = false;
            for (std::size_t a = 0; a < cars.size(); ++a) {
                for (std::size_t b = a + 1; b < cars.size(); ++b) {
                    touching |= resolve_vehicle_contact(*cars[a], *cars[b]).has_value();
                }
            }
            if (!touching) {
                break;
            }
        }
    }
}
