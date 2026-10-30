#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <algorithm>
#include <numbers>

#include "platform/glfw_session.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/track_mesh.hpp"
#include "graphics/triangle.hpp"
#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"

namespace {
    constexpr int window_width = 1200;
    constexpr int window_height = 720;

    void update_viewport(GLFWwindow *, int width, int height) {
        glViewport(0, 0, width, height);
    }

    constexpr char vertex_shader_source[] = R"glsl(
        #version 330 core

        layout(location = 0) in vec2 position;

        uniform vec2 offset;
        uniform vec2 view_scale;
        uniform float heading;

        void main() {
            float c = cos(heading);
            float s = sin(heading);

            vec2 rotated_position = vec2(
                c * position.x - s * position.y,
                s * position.x +c * position.y
            );

            gl_Position = vec4((rotated_position + offset) * view_scale, 0.0, 1.0);
        }
    )glsl";

    constexpr char fragment_shader_source[] = R"glsl(
        #version 330 core

        uniform vec3 color;
        out vec4 fragment_color;

        void main() {
            fragment_color = vec4(color, 1.0);
        }
    )glsl";

    struct WindowDeleter {
        void operator()(GLFWwindow *window) const noexcept {
            glfwDestroyWindow(window);
        }
    };

    using Window = std::unique_ptr<GLFWwindow, WindowDeleter>;

    apex::simulation::VehicleControls read_vehicle_controls(
        GLFWwindow *window
    ) {
        apex::simulation::VehicleControls controls{};

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            controls.throttle += 1.0;
        }

        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            controls.throttle -= 1.0;
        }

        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            controls.steering += 1.0;
        }

        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            controls.steering -= 1.0;
        }

        return controls;
    }
}

int main() {
    try {
        apex::platform::GlfwSession glfw;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

        Window window{
            glfwCreateWindow(window_width, window_height, "Apex", nullptr, nullptr)
        };

        if (!window) {
            throw std::runtime_error("Failed to create the Apex window");
        }

        glfwMakeContextCurrent(window.get());

        if (gladLoadGL(glfwGetProcAddress) == 0) {
            throw std::runtime_error("Failed to load OpenGL functions");
        }

        glfwSwapInterval(1);

        glfwSetFramebufferSizeCallback(window.get(), update_viewport);

        int framebuffer_width{};
        int framebuffer_height{};

        glfwGetFramebufferSize(window.get(), &framebuffer_width, &framebuffer_height);

        update_viewport(window.get(), framebuffer_width, framebuffer_height);

        apex::graphics::Shader vertex_shader{GL_VERTEX_SHADER};
        vertex_shader.compile(vertex_shader_source);

        apex::graphics::Shader fragment_shader{GL_FRAGMENT_SHADER};
        fragment_shader.compile(fragment_shader_source);

        std::cout << "Both shaders compiled successfully.\n";

        apex::graphics::ShaderProgram program;
        program.link(vertex_shader, fragment_shader);
        const GLint offset_location = program.uniform_location("offset");
        const GLint view_scale_location = program.uniform_location("view_scale");
        const GLint heading_location =
                program.uniform_location("heading");
        const GLint color_location = program.uniform_location("color");

        std::cout << "Shader program linked successfully.\n";

        apex::graphics::Triangle triangle;

        const apex::simulation::TrackSettings track_settings{};
        apex::graphics::TrackMesh track_mesh{track_settings};

        glClearColor(0.04f, 0.12f, 0.14f, 1.0f);

        double previous_time = glfwGetTime();
        const apex::simulation::VehicleSettings vehicle_settings{};
        apex::simulation::VehicleState vehicle{};

        vehicle.body.position = {track_settings.centerline_radius, 0.0};
        vehicle.heading = std::numbers::pi_v<double> / 2.0;

        constexpr double physics_step = 1.0 / 120.0;
        constexpr double max_frame_time = 0.25;

        double accumulator{};

        while (glfwWindowShouldClose(window.get()) == GLFW_FALSE) {
            const double current_time = glfwGetTime();
            const double delta_time = current_time - previous_time;

            previous_time = current_time;

            glfwPollEvents();

            if (glfwWindowShouldClose(window.get()) == GLFW_TRUE) {
                break;
            }

            glfwGetFramebufferSize(window.get(), &framebuffer_width, &framebuffer_height);

            if (framebuffer_width <= 0 || framebuffer_height <= 0) {
                glfwWaitEvents();
                previous_time = glfwGetTime();
                continue;
            }

            const auto controls = read_vehicle_controls(window.get());

            accumulator +=
                    std::min(delta_time, max_frame_time);

            while (accumulator >= physics_step) {
                apex::simulation::update_vehicle(vehicle, controls, vehicle_settings, physics_step);

                accumulator -= physics_step;
            }
            if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
            }

            glClear(GL_COLOR_BUFFER_BIT);

            program.use();

            const float aspect =
                    static_cast<float>(framebuffer_width) / static_cast<float>(framebuffer_height);

            if (aspect >= 1.0F) {
                glUniform2f(view_scale_location, 1.0F / aspect, 1.0F);
            } else {
                glUniform2f(view_scale_location, 1.0F, aspect);
            }

            glUniform2f(offset_location, 0.0F, 0.0F);
            glUniform1f(heading_location, 0.0F);
            glUniform3f(color_location, 0.25F, 0.27F, 0.29F);

            track_mesh.draw();


            const bool on_track = apex::simulation::is_on_track(vehicle.body.position, track_settings);

            if (on_track) {
                glUniform3f(color_location, 0.95F, 0.25F, 0.12F);
            } else {
                glUniform3f(color_location, 1.0F, 0.85F, 0.15F);
            }

            glUniform2f(offset_location, static_cast<float>(vehicle.body.position.x),
                        static_cast<float>(vehicle.body.position.y));

            glUniform1f(heading_location, static_cast<float>(vehicle.heading));

            triangle.draw();

            glfwSwapBuffers(window.get());
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
