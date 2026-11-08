#include "graphics/simulation_window.hpp"

#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "graphics/barrier_mesh.hpp"
#include "graphics/camera.hpp"
#include "platform/glfw_session.hpp"
#include "graphics/sensor_lines.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/track_mesh.hpp"
#include "graphics/triangle.hpp"
#include "graphics/ground_mesh.hpp"

namespace apex::graphics {
    namespace {
        constexpr char vertex_shader_source[] = R"glsl(
            #version 330 core

            layout(location = 0) in vec3 position;

            uniform vec2 offset;
            uniform float heading;
            uniform float elevation;
            uniform mat4 view_projection;

            out vec3 surface_position;

            void main() {
                float c = cos(heading);
                float s = sin(heading);

                vec2 rotated_position = vec2(
                    c * position.x - s * position.y,
                    s * position.x +c * position.y
                );

                vec3 world_position = vec3(
                    rotated_position + offset,
                    position.z + elevation
                );

                surface_position = world_position;

                gl_Position = view_projection * vec4(world_position, 1.0);
            }
        )glsl";

        constexpr char fragment_shader_source[] = R"glsl(
            #version 330 core

            in vec3 surface_position;

            uniform vec3 color;
            uniform bool curb_pattern;
            uniform float track_radius;
            uniform float road_half_width;
            uniform float curb_segments;

            uniform bool lighting_enabled;
            uniform vec3 direction_to_light;

            out vec4 fragment_color;

            void main() {
                float brightness = 1.0;

                if (lighting_enabled) {
                    vec3 normal = normalize(cross(
                        dFdx(surface_position),
                        dFdy(surface_position)
                    ));

                    vec3 light = normalize(direction_to_light);

                    float diffuse = max(dot(normal, light), 0.0);

                    const float ambient = 0.35;
                    brightness = ambient + (1.0 - ambient) * diffuse;
                }

                vec3 base_color = color;

                if (curb_pattern) {
                    vec2 world_xy = surface_position.xy;

                    float edge_distance =
                        abs(length(world_xy) - track_radius);

                    if (edge_distance <= road_half_width) {
                        discard;
                    }

                    const float pi = 3.14159265359;

                    float angle = atan(world_xy.y, world_xy.x) + pi;
                    float segment = floor(angle * curb_segments / (2.0 * pi));

                    float stripe = mod(segment, 2.0);

                    base_color = mix(vec3(0.80, 0.07, 0.05),vec3(0.93, 0.91, 0.85),stripe);
                }

                fragment_color = vec4(base_color * brightness, 1.0);
            }
        )glsl";

        simulation::TrackSettings curb_geometry_settings(const simulation::TrackSettings &track) {
            constexpr double desired_curb_width = 0.025;

            const double inner_radius =
                    track.centerline_radius - track.road_width * 0.5;

            const double curb_width =
                    std::min(desired_curb_width, inner_radius * 0.5);

            auto geometry = track;
            geometry.road_width += curb_width * 2.0;

            return geometry;
        }

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
        TrackMesh curb_mesh;
        Triangle vehicle_mesh;
        GroundMesh ground_mesh;
        BarrierMesh barrier_mesh;

        SensorLines sensor_lines;

        GLint offset_location{};
        GLint view_projection_location{};
        GLint elevation_location{};
        GLint heading_location{};
        GLint color_location{};
        GLint curb_pattern_location{};
        GLint track_radius_location{};
        GLint road_half_width_location{};
        GLint curb_segments_location{};
        GLint lighting_enabled_location{};
        GLint direction_to_light_location{};

        bool reset_was_pressed{};
        bool pause_was_pressed{};
        bool autopilot_was_pressed{};

        CameraSettings camera{};

        explicit Impl(const simulation::TrackSettings &track)
            : track_settings{track}, track_mesh{track},
              curb_mesh{curb_geometry_settings(track)}, barrier_mesh{track} {
            Shader vertex_shader{GL_VERTEX_SHADER};
            vertex_shader.compile(vertex_shader_source);

            Shader fragment_shader{GL_FRAGMENT_SHADER};
            fragment_shader.compile(fragment_shader_source);

            program.link(vertex_shader, fragment_shader);

            offset_location = program.uniform_location("offset");
            view_projection_location = program.uniform_location("view_projection");
            elevation_location = program.uniform_location("elevation");
            heading_location = program.uniform_location("heading");
            color_location = program.uniform_location("color");

            curb_pattern_location = program.uniform_location("curb_pattern");
            track_radius_location = program.uniform_location("track_radius");
            road_half_width_location = program.uniform_location("road_half_width");
            curb_segments_location = program.uniform_location("curb_segments");

            lighting_enabled_location = program.uniform_location("lighting_enabled");
            direction_to_light_location = program.uniform_location("direction_to_light");

            glClearColor(0.04F, 0.12F, 0.14F, 1.0F);
            glEnable(GL_DEPTH_TEST);
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
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        view.program.use();

        glUniform1i(view.lighting_enabled_location, GL_TRUE);
        glUniform3f(view.direction_to_light_location, -0.5F, -0.4F, 1.0F);

        const float aspect =
                static_cast<float>(width) / static_cast<float>(height);

        const auto &camera =
                view.camera;

        const glm::mat4 projection = glm::perspective(glm::radians(camera.vertical_fov_degrees), aspect,
                                                      camera.near_clip, camera.far_clip);

        const glm::mat4 camera_view =
                glm::lookAt(
                    camera.position, camera.target, camera.up);

        const glm::mat4 view_projection = projection * camera_view;

        glUniformMatrix4fv(view.view_projection_location, 1,GL_FALSE, glm::value_ptr(view_projection));

        constexpr float road_elevation = 0.0F;
        constexpr float sensor_elevation = 0.002F;
        constexpr float vehicle_elevation = 0.004F;

        glUniform2f(view.offset_location, 0.0F, 0.0F);
        glUniform1f(view.heading_location, 0.0F);

        constexpr float ground_elevation = -0.005F;

        glUniform1i(view.curb_pattern_location,GL_FALSE);
        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.68F, 0.65F, 0.53F);

        view.ground_mesh.draw();

        glUniform1f(view.elevation_location, road_elevation);
        glUniform3f(view.color_location, 0.25F, 0.27F, 0.29F);
        glUniform1i(view.curb_pattern_location,GL_FALSE);
        view.track_mesh.draw();

        constexpr float curb_elevation = 0.001F;
        constexpr float curb_segments = 80.0F;

        glUniform1i(view.curb_pattern_location,GL_TRUE);
        glUniform1f(view.track_radius_location, static_cast<float>(view.track_settings.centerline_radius));
        glUniform1f(view.road_half_width_location, static_cast<float>(view.track_settings.road_width * 0.5));
        glUniform1f(view.curb_segments_location, curb_segments);
        glUniform1f(view.elevation_location, curb_elevation);

        view.curb_mesh.draw();

        glUniform1i(view.curb_pattern_location, GL_FALSE);

        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.16F, 0.17F, 0.18F);
        view.barrier_mesh.draw();

        const auto readings = simulation::sense_track(world.vehicle, view.track_settings, sensors);

        view.sensor_lines.update(world.vehicle, sensors, readings);

        glUniform1f(view.elevation_location, sensor_elevation);
        glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        glUniform1i(view.lighting_enabled_location,GL_FALSE);
        view.sensor_lines.draw();
        glUniform1i(view.lighting_enabled_location,GL_TRUE);

        const auto &position =
                world.vehicle.body.position;
        const bool on_track =
                simulation::is_on_track(position, view.track_settings);

        if (on_track) {
            glUniform3f(view.color_location, 0.95F, 0.25F, 0.12F);
        } else {
            glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        }

        glUniform2f(view.offset_location, static_cast<float>(position.x), static_cast<float>(position.y));
        glUniform1f(view.heading_location, static_cast<float>(world.vehicle.heading));
        glUniform1f(view.elevation_location, vehicle_elevation);

        view.vehicle_mesh.draw();

        glfwSwapBuffers(window);
    }

    void SimulationWindow::close() noexcept {
        impl_.reset();
    }
}
