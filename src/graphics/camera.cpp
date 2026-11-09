#include "graphics/camera.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace apex::graphics {
    namespace {
        constexpr float minimum_distance = 0.3F;
        constexpr float maximum_distance = 8.0F;

        constexpr float minimum_pitch =
                10.0F * std::numbers::pi_v<float> / 180.0F;
        constexpr float maximum_pitch =
                85.0F * std::numbers::pi_v<float> / 180.0F;

        constexpr float zoom_sensitivity = 0.12F;

        bool is_finite(glm::vec3 value) {
            return std::isfinite(value.x) &&
                   std::isfinite(value.y) &&
                   std::isfinite(value.z);
        }
    }

    OrbitCamera::OrbitCamera(const CameraSettings &settings)
        : initial_settings_{settings},
          settings_{settings} {
        if (!is_finite(settings.position) ||
            !is_finite(settings.target) ||
            !is_finite(settings.up)) {
            throw std::invalid_argument(
                "Camera vectors must be finite"
            );
        }

        if (settings.up.x != 0.0F ||
            settings.up.y != 0.0F ||
            settings.up.z <= 0.0F) {
            throw std::invalid_argument(
                "Orbit camera requires positive Z up"
            );
        }

        const auto offset = settings.position - settings.target;
        const float distance =
                std::hypot(offset.x, offset.y, offset.z);

        const float pitch = std::atan2(
            offset.z,
            std::hypot(offset.x, offset.y)
        );

        if (!std::isfinite(distance) ||
            distance < minimum_distance ||
            distance > maximum_distance ||
            pitch < minimum_pitch ||
            pitch > maximum_pitch) {
            throw std::invalid_argument(
                "Initial camera position exceeds orbit limits"
            );
        }
    }

    void OrbitCamera::orbit(float yaw_delta, float pitch_delta) {
        if (!std::isfinite(yaw_delta) ||
            !std::isfinite(pitch_delta)) {
            throw std::invalid_argument(
                "Orbit changes must be finite"
            );
        }

        const auto offset = settings_.position - settings_.target;
        const float distance =
                std::hypot(offset.x, offset.y, offset.z);

        constexpr float pi = std::numbers::pi_v<float>;

        const float yaw = std::remainder(
            std::atan2(offset.y, offset.x) +
            std::remainder(yaw_delta, 2.0F * pi),
            2.0F * pi
        );

        const float current_pitch = std::atan2(
            offset.z,
            std::hypot(offset.x, offset.y)
        );

        const float pitch = std::clamp(
            current_pitch + std::clamp(pitch_delta, -pi, pi),
            minimum_pitch,
            maximum_pitch
        );

        const float horizontal_distance =
                distance * std::cos(pitch);

        settings_.position = settings_.target + glm::vec3{
                                 horizontal_distance * std::cos(yaw),
                                 horizontal_distance * std::sin(yaw),
                                 distance * std::sin(pitch)
                             };
    }

    void OrbitCamera::zoom(float amount) {
        if (!std::isfinite(amount)) {
            throw std::invalid_argument(
                "Zoom amount must be finite"
            );
        }

        const auto offset = settings_.position - settings_.target;
        const float distance =
                std::hypot(offset.x, offset.y, offset.z);

        const float scale = std::exp(
            -zoom_sensitivity * std::clamp(amount, -50.0F, 50.0F)
        );

        const float next_distance = std::clamp(
            distance * scale,
            minimum_distance,
            maximum_distance
        );

        settings_.position =
                settings_.target + offset * (next_distance / distance);
    }

    void OrbitCamera::reset() {
        settings_ = initial_settings_;
    }

    const CameraSettings &OrbitCamera::settings() const noexcept {
        return settings_;
    }
}
