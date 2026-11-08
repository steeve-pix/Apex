#include "graphics/simulation_window.hpp"

#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>

#include "platform/glfw_session.hpp"
#include "graphics/sensor_lines.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/track_mesh.hpp"
#include "graphics/triangle.hpp"

namespace apex::graphics {
    namespace {
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

        Window create_window() {
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);


#ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GLFW_TRUE);
#endif
            Window window{
                glfwCreateWindow(1200, 720, "Apex", nullptr, nullptr),
            };

            if (!window) {
                throw std::runtime_error{"Failed to create Apex window"};
            }

            glfwMakeContextCurrent(window.get());

            if (gladLoadGL(glfwGetProcAddress) == 0) {
                throw std::runtime_error{"Failed to load OpenGL functions"};
            }

            glfwSwapInterval(1);

            return window;
        }
    }

    struct SimulationWindow::Impl {
        platform::GlfwSession glfw;
        Window window{create_window()};
        simulation::TrackSettings track_settings;

        ShaderProgram program;
        TrackMesh track_mesh;
        Triangle vehicle_mesh;
        SensorLines sensor_lines;

        GLint offset_location{};
        GLint view_scale_location{};
        GLint heading_location{};
        GLint color_location{};

        bool reset_was_pressed{};
        bool pause_was_pressed{};
        bool autopilot_was_pressed{};

        explicit Impl(const simulation::TrackSettings &track)
            : track_settings{track}, track_mesh{track} {
            Shader vertex_shader{GL_VERTEX_SHADER};
            vertex_shader.compile(vertex_shader_source);

            Shader fragment_shader{GL_FRAGMENT_SHADER};
            fragment_shader.compile(fragment_shader_source);

            program.link(vertex_shader, fragment_shader);

            offset_location = program.uniform_location("offset");
            view_scale_location = program.uniform_location("view_scale");
            heading_location = program.uniform_location("heading");
            color_location = program.uniform_location("color");

            glClearColor(0.04F, 0.12F, 0.14F, 1.0F);
        }
    };

    SimulationWindow::SimulationWindow(const simulation::TrackSettings &track) : impl_(std::make_unique<Impl>(track)) {
    }

    SimulationWindow::~SimulationWindow() {
        close();
    }

    WindowInput SimulationWindow::poll_events() {
        WindowInput input{};

        if (!impl_) {
            input.close_requested = true;
            return input;
        }

        glfwPollEvents();

        auto &view = *impl_;
        auto *window = view.window.get();

        const auto key_down = [window](int key) {
            return glfwGetKey(window, key) == GLFW_PRESS;
        };

        input.close_requested =
                glfwWindowShouldClose(window) == GLFW_TRUE ||
                key_down(GLFW_KEY_ESCAPE);

        if (input.close_requested) {
            return input;
        }

        const auto pressed_once =
                [&key_down](int key, bool &was_pressed) {
            const bool pressed = key_down(key);
            const bool triggered = pressed && !was_pressed;
            was_pressed = pressed;
            return triggered;
        };

        input.reset_requested =
                pressed_once(GLFW_KEY_R, view.reset_was_pressed);

        input.pause_requested =
                pressed_once(GLFW_KEY_SPACE, view.pause_was_pressed);

        input.autopilot_requested =
                pressed_once(GLFW_KEY_P, view.autopilot_was_pressed);

        if (key_down(GLFW_KEY_W) || key_down(GLFW_KEY_UP)) {
            input.controls.throttle += 1.0;
        }

        if (key_down(GLFW_KEY_S) || key_down(GLFW_KEY_DOWN)) {
            input.controls.throttle -= 1.0;
        }

        if (key_down(GLFW_KEY_A) || key_down(GLFW_KEY_LEFT)) {
            input.controls.steering += 1.0;
        }

        if (key_down(GLFW_KEY_D) || key_down(GLFW_KEY_RIGHT)) {
            input.controls.steering -= 1.0;
        }

        return input;
    }

    void SimulationWindow::draw(const simulation::WorldState &world, const simulation::SensorSettings &sensors,
                                std::string_view status) {
        if (!impl_) {
            throw std::runtime_error{"Cannot draw a closed window"};
        }

        auto &view = *impl_;
        auto *window = view.window.get();

        glfwMakeContextCurrent(window);

        const std::string title = "Apex | " + std::string{status};
        glfwSetWindowTitle(window, title.c_str());

        int width{};
        int height{};
        glfwGetFramebufferSize(window, &width, &height);

        if (width <= 0 || height <= 0) {
            return;
        }

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT);

        view.program.use();

        const float aspect =
                static_cast<float>(width) / static_cast<float>(height);

        if (aspect >= 1.0F) {
            glUniform2f(view.view_scale_location, 1.0F / aspect, 1.0F);
        } else {
            glUniform2f(view.view_scale_location, 1.0F, aspect);
        }

        glUniform2f(view.offset_location, 0.0F, 0.0F);
        glUniform1f(view.heading_location, 0.0F);
        glUniform3f(view.color_location, 0.25F, 0.27F, 0.29F);
        view.track_mesh.draw();

        const auto readings = simulation::sense_track(world.vehicle, view.track_settings, sensors);

        view.sensor_lines.update(world.vehicle, sensors, readings);
        glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        view.sensor_lines.draw();

        const auto &position = world.vehicle.body.position;
        const bool on_track =
                simulation::is_on_track(position, view.track_settings);


        if (on_track) {
            glUniform3f(view.color_location, 0.95F, 0.25F, 0.12F);
        } else {
            glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        }

        glUniform2f(view.offset_location, static_cast<float>(position.x), static_cast<float>(position.y));
        glUniform1f(view.heading_location, static_cast<float>(world.vehicle.heading));

        view.vehicle_mesh.draw();
        glfwSwapBuffers(window);
    }

    void SimulationWindow::close() noexcept {
        impl_.reset();
    }
}
