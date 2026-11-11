#pragma once

#include <glad/gl.h>

namespace apex::graphics {
    class CarMesh {
    public:
        static constexpr float front_tire_radius = .0224F;
        static constexpr float rear_tire_radius = .0236F;

        CarMesh();

        ~CarMesh();

        CarMesh(const CarMesh &) = delete;

        CarMesh &operator=(const CarMesh &) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}
