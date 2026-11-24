#include "graphics/sensor_lines.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x;
            float y;
            float z;
        };

        constexpr std::size_t subdivisions = 12;
        using LineVertices = std::array<Vertex, simulation::sensor_count * subdivisions * 2>;
    }

    SensorLines::SensorLines() {
        vertex_count_ = static_cast<GLsizei>(LineVertices{}.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error("Failed to create sensor line resources");
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(LineVertices{}.size() * sizeof(Vertex)), nullptr,
                     GL_DYNAMIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    SensorLines::~SensorLines() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void SensorLines::update(const simulation::VehicleState &vehicle, const simulation::SensorSettings &sensors,
                             const simulation::SensorReadings &readings, const simulation::TrackPath *path) const {
        LineVertices vertices{};
        const auto &origin = vehicle.body.position;
        const bool elevated = path && path->has_elevation();
        const auto vertex_at = [&](const math::Vec2 &point) -> Vertex {
            const double height = elevated ? path->elevation_at(path->project(point).distance_along_track).height : 0.0;
            return {static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(height)};
        };
        const auto start = vertex_at(origin);
        std::size_t index = 0;
        for (std::size_t i = 0; i < readings.size(); ++i) {
            const double angle = vehicle.heading + sensors.angle_offset[i];
            const double dx = std::cos(angle) * readings[i].distance / subdivisions;
            const double dy = std::sin(angle) * readings[i].distance / subdivisions;
            auto previous = start;
            for (std::size_t step = 1; step <= subdivisions; ++step) {
                const auto next = vertex_at({origin.x + dx * step, origin.y + dy * step});
                vertices[index++] = previous;
                vertices[index++] = next;
                previous = next;
            }
        }

        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)), vertices.data());

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void SensorLines::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_LINES, 0, vertex_count_);
    }
}
