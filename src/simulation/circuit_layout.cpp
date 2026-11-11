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

            // Brake into a tightening left and exit diagonally across the infield.
            math::Vec2{2.66, -1.34},
            math::Vec2{3.08, -1.02},
            math::Vec2{3.04, -0.62},
            math::Vec2{2.62, -0.42},
            math::Vec2{1.88, -0.46},
            math::Vec2{1.11, -0.25},

            // A fast infield exit leads into a right-left chicane on the outer sweep.
            math::Vec2{0.87, 0.16},
            math::Vec2{1.10, 0.51},
            math::Vec2{1.72, 0.59},
            math::Vec2{2.31, 0.75},
            math::Vec2{2.48, 1.06},
            math::Vec2{2.52, 1.38},
            math::Vec2{2.70, 1.64},
            math::Vec2{2.78, 1.96},
            math::Vec2{2.54, 2.27},
            math::Vec2{1.95, 2.36},
            math::Vec2{1.39, 2.37},
            math::Vec2{0.73, 2.23},

            // A late-apex infield hairpin feeds another change of direction.
            math::Vec2{0.32, 1.86},
            math::Vec2{0.26, 1.32},
            math::Vec2{-0.07, 1.01},
            math::Vec2{-0.47, 1.13},
            math::Vec2{-0.63, 1.61},
            math::Vec2{-0.99, 2.13},
            math::Vec2{-1.56, 2.36},
            math::Vec2{-2.20, 2.17},
            math::Vec2{-2.61, 1.70},

            // Offset esses and a slow final corner back onto the grid straight.
            math::Vec2{-2.64, 1.16},
            math::Vec2{-2.25, 0.77},
            math::Vec2{-1.78, 0.65},
            math::Vec2{-1.54, 0.28},
            math::Vec2{-1.78, -0.10},
            math::Vec2{-2.31, -0.23},
            math::Vec2{-2.69, -0.59},
            math::Vec2{-2.55, -1.10},
            math::Vec2{-2.22, -1.36}
        };

        constexpr double layout_scale = 3.85;
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
