#include "simulation/circuit_layout.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

namespace apex::simulation {
    namespace {
        double interpolate_coordinate(double previous, double start, double end, double next, double fraction) {
            const double linear = end - previous;
            const double quadratic =
                    2.0 * previous - 5.0 * start + 4.0 * end - next;
            const double cubic =
                    -previous + 3.0 * start - 3.0 * end + next;

            return 0.5 * (2.0 * start + fraction * (linear + fraction * (quadratic + fraction * cubic)));
        }

        math::Vec2 interpolate_section(const math::Vec2 &previous, const math::Vec2 &start, const math::Vec2 &end,
                                       const math::Vec2 &next, double fraction) {
            return {
                interpolate_coordinate(previous.x, start.x, end.x, next.x, fraction),
                interpolate_coordinate(previous.y, start.y, end.y, next.y, fraction)
            };
        }
    }

    TrackPath make_apex_circuit(int samples_per_section) {
        if (samples_per_section < 4 || samples_per_section > 256) {
            throw std::invalid_argument("Circuit samples per section must be between 4 and 256");
        }

        const std::array control_points{
            // Start straight.
            math::Vec2{-1.80, -1.40},
            math::Vec2{-0.60, -1.40},
            math::Vec2{0.80, -1.40},
            math::Vec2{1.80, -1.40},

            // Hairpin.
            math::Vec2{2.30, -1.10},
            math::Vec2{2.35, -0.60},
            math::Vec2{2.00, -0.20},
            math::Vec2{1.25, -0.15},

            // S-bends.
            math::Vec2{0.70, 0.30},
            math::Vec2{1.05, 0.90},
            math::Vec2{1.75, 1.25},
            math::Vec2{1.80, 1.80},

            // Sweeping return.
            math::Vec2{1.10, 2.15},
            math::Vec2{0.20, 2.10},
            math::Vec2{-0.55, 1.80},
            math::Vec2{-0.60, 1.10},
            math::Vec2{-1.20, 0.65},
            math::Vec2{-1.80, 1.00},
            math::Vec2{-2.30, 0.90},
            math::Vec2{-2.45, 0.25},
            math::Vec2{-2.30, -0.60}
        };

        constexpr double layout_scale = 1.5;
        std::vector<math::Vec2> points;
        points.reserve(control_points.size() * static_cast<std::size_t>(samples_per_section));

        for (std::size_t section = 0; section < control_points.size(); ++section) {
            const auto &previous = control_points[(section + control_points.size() - 1) % control_points.size()];
            const auto &start = control_points[section];
            const auto &end = control_points[(section + 1) % control_points.size()];
            const auto &next = control_points[(section + 2) % control_points.size()];

            for (int sample = 0; sample < samples_per_section; ++sample) {
                const double fraction =
                        static_cast<double>(sample) / static_cast<double>(samples_per_section);

                const auto point = interpolate_section(previous, start, end, next, fraction);
                points.push_back({point.x * layout_scale, point.y * layout_scale});
            }
        }

        return TrackPath(std::move(points));
    }
}
