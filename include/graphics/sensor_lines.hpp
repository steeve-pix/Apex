#pragma once

#include <glad/gl.h>

#include "simulation/sensors.hpp"

namespace apex::graphics {
    class SensorLines {
    public:
        SensorLines();

        ~SensorLines();

        SensorLines(const SensorLines &) = delete;

        SensorLines &operator=(const SensorLines &) = delete;

        void update(
            const simulation::VehicleState &vehicle, const simulation::SensorSettings &sensors,
            const simulation::SensorReadings &readings) const;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
