#pragma once

#include <glm/vec3.hpp>

namespace apex::graphics {
    struct CameraSettings {
        glm::vec3 position{1.1F, -1.5F, 1.8F};
        glm::vec3 target{0.0F, 0.0F, 0.0F};
        glm::vec3 up{0.0F, 0.0F, 1.0F};

        float vertical_fov_degrees{45.0F};
        float near_clip{0.01F};
        float far_clip{20.0F};

        float minimum_distance{0.3F};
        float maximum_distance{8.0F};
    };

    class OrbitCamera {
    public:
        explicit OrbitCamera(const CameraSettings &settings = {});


        void orbit(float yaw_delta, float pitch_delta);

        void zoom(float amount);

        void reset();

        void set_target(glm::vec3 target);

        [[nodiscard]] const CameraSettings &settings() const noexcept;

    private:
        CameraSettings initial_settings_;
        CameraSettings settings_;
    };
}
