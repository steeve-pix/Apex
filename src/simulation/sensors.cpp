#include "simulation/sensors.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <optional>

#include "simulation/track_boundaries.hpp"

namespace apex::simulation {
    namespace {
        std::optional<double> ray_segment_distance(const math::Vec2 &origin, const math::Vec2 &direction,
                                                   const math::Vec2 &start, const math::Vec2 &end,
                                                   double maximum_distance) {
            const double edge_x = end.x - start.x;
            const double edge_y = end.y - start.y;
            const double edge_length = std::hypot(edge_x, edge_y);

            const double offset_x = start.x - origin.x;
            const double offset_y = start.y - origin.y;

            constexpr double rounding_margin =
                    32.0 * std::numeric_limits<double>::epsilon();

            const double coordinate_scale = std::max({
                std::abs(origin.x), std::abs(origin.y),
                std::abs(start.x), std::abs(start.y),
                std::abs(end.x), std::abs(end.y)
            });

            const double distance_tolerance =
                    rounding_margin * coordinate_scale;

            const double denominator =
                    direction.x * edge_y - direction.y * edge_x;

            if (std::abs(denominator) <= rounding_margin * edge_length) {
                const double perpendicular_distance =
                        offset_x * direction.y - offset_y * direction.x;

                if (std::abs(perpendicular_distance) > distance_tolerance) {
                    return std::nullopt;
                }

                const double start_distance =
                        offset_x * direction.x + offset_y * direction.y;

                const double end_distance =
                        (end.x - origin.x) * direction.x
                        + (end.y - origin.y) * direction.y;

                if (std::max(start_distance, end_distance) < -distance_tolerance) {
                    return std::nullopt;
                }

                const double nearest = std::max(0.0, std::min(start_distance, end_distance));

                if (nearest > maximum_distance + distance_tolerance) {
                    return std::nullopt;
                }

                return std::clamp(nearest, 0.0, maximum_distance);
            }

            const double distance =
                    (offset_x * edge_y - offset_y * edge_x) / denominator;

            const double fraction =
                    (offset_x * direction.y - offset_y * direction.x) / denominator;

            const double fraction_tolerance =
                    distance_tolerance / edge_length;

            if (!std::isfinite(distance) || !std::isfinite(fraction) || distance < -distance_tolerance || distance >
                maximum_distance + distance_tolerance || fraction < -fraction_tolerance || fraction > 1.0 +
                fraction_tolerance) {
                return std::nullopt;
            }

            return std::clamp(distance, 0.0, maximum_distance);
        }

        void validate_ray(const math::Vec2 &origin, double angle, double maximum_distance) {
            if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(angle) || !
                std::isfinite(maximum_distance) || maximum_distance < 0.0) {
                throw std::invalid_argument("Invalid sensor ray");
            }
        }

        RayReading cast_boundary_ray(const math::Vec2 &origin, double angle, const TrackBoundaries &boundaries,
                                     double max_distance) {
            const math::Vec2 direction{
                std::cos(angle),
                std::sin(angle)
            };

            RayReading reading{max_distance, false};

            for (const auto *boundary: {&boundaries.left, &boundaries.right}) {
                for (std::size_t index = 0; index < boundary->size(); ++index) {
                    const std::size_t next = (index + 1) % boundary->size();

                    if (const auto distance = ray_segment_distance(origin, direction, (*boundary)[index],
                                                                   (*boundary)[next], reading.distance)) {
                        reading = RayReading{*distance, true};
                    }
                }
            }

            return reading;
        }
    }

    RayReading cast_track_ray(const math::Vec2 &origin, double angle, const TrackSettings &track, double max_distance) {
        validate_ray(origin, angle, max_distance);

        if (track.path) {
            const auto boundaries = make_track_boundaries(*track.path, track.road_width * 0.5);

            return cast_boundary_ray(origin, angle, boundaries, max_distance);
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

    SensorReadings sense_track(const VehicleState &vehicle, const TrackSettings &track, const SensorSettings &sensors) {
        std::optional<TrackBoundaries> boundaries;

        if (track.path) {
            boundaries = make_track_boundaries(*track.path, track.road_width * 0.5);
        }

        SensorReadings readings{};

        for (std::size_t index = 0; index < readings.size(); ++index) {
            const double angle =
                    vehicle.heading + sensors.angle_offset[index];

            if (boundaries) {
                validate_ray(vehicle.body.position, angle, sensors.max_distance);

                readings[index] = cast_boundary_ray(vehicle.body.position, angle, *boundaries, sensors.max_distance);
            } else {
                readings[index] = cast_track_ray(vehicle.body.position, angle, track, sensors.max_distance);
            }
        }

        return readings;
    }
}
