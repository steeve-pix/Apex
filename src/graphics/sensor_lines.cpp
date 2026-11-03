#include "graphics/sensor_lines.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x;
            float y;
        };

        using LineVertices =
        std::array<Vertex, simulation::sensor_count * 2>;
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

        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);
        glEnableVertexAttribArray(0);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    SensorLines::~SensorLines() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void SensorLines::update(const simulation::VehicleState &vehicle, const simulation::SensorSettings &sensors,
                             const simulation::SensorReadings &readings) const {
        LineVertices vertices{};
        const auto &origin = vehicle.body.position;

        for (std::size_t i = 0; i < readings.size(); ++i) {
            const double angle =
                    vehicle.heading + sensors.angle_offset[i];

            vertices[i * 2] = {
                static_cast<float>(origin.x),
                static_cast<float>(origin.y)
            };

            vertices[i * 2 + 1] = {
                static_cast<float>(origin.x + std::cos(angle) * readings[i].distance),
                static_cast<float>(origin.y + std::sin(angle) * readings[i].distance)
            };
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
