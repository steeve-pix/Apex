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
    };
}
