#include "simulation/sensors.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace apex::simulation {
    RayReading cast_track_ray(const math::Vec2 &origin, double angle, const TrackSettings &track, double max_distance) {
        if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(angle) ||
            !std::isfinite(max_distance) || max_distance < 0.0) {
            throw std::invalid_argument("Invalid sensor ray");
        }

        const double half_width = track.road_width * 0.5;

        if (!std::isfinite(track.centerline_radius) || !std::isfinite(track.road_width) || half_width <= 0.0 || track.
            centerline_radius <= half_width) {
            throw std::invalid_argument("Invalid track dimensions");
        }

        const math::Vec2 direction{std::cos(angle), std::sin(angle)};

        const double a =
                direction.x * direction.x + direction.y * direction.y;

        const double projection = origin.x * direction.x + origin.y * direction.y;

        const std::array radii{track.centerline_radius - half_width, track.centerline_radius + half_width};

        RayReading reading{max_distance, false};

        for (const double radius: radii) {
            const double c = origin.x * origin.x + origin.y * origin.y - radius * radius;

            const double discriminant =
                    projection * projection - a * c;

            if (discriminant < 0.0) {
                continue;
            }

            const double root = std::sqrt(discriminant);

            const std::array distances{
                (-projection - root) / a,
                (-projection + root) / a
            };

            for (const double distance: distances) {
                if (distance >= 0.0 && distance <= reading.distance) {
                    reading = RayReading{distance, true};
                }
            }
        }

        return reading;
    }
}
