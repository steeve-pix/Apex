#include "graphics/simulation_window.hpp"

#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <numbers>
#include <vector>

#include "graphics/tree_mesh.hpp"
#include "graphics/barrier_mesh.hpp"
#include "graphics/camera.hpp"
#include "graphics/car_mesh.hpp"
#include "platform/glfw_session.hpp"
#include "graphics/sensor_lines.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/track_mesh.hpp"
#include "graphics/ground_mesh.hpp"
#include "graphics/shadow_map.hpp"

namespace apex::graphics {
    namespace {
        constexpr char shadow_vertex_shader_source[] = R"glsl(
            #version 330 core

            layout(location = 0) in vec3 position;

            uniform vec2 offset;
            uniform float heading;
            uniform float elevation;
            uniform mat4 light_view_projection;

            void main() {
                float c = cos(heading);
                float s = sin(heading);

                vec2 rotated_position = vec2(
                    c * position.x - s * position.y,
                    s * position.x + c * position.y
                );

                vec3 world_position = vec3(
                    rotated_position + offset,
                    position.z + elevation
                );

                gl_Position =
                    light_view_projection * vec4(world_position, 1.0);
            }
        )glsl";

        constexpr char shadow_fragment_shader_source[] = R"glsl(
            #version 330 core

            void main() {
            }
        )glsl";
        constexpr char vertex_shader_source[] = R"glsl(
            #version 330 core

            layout(location = 0) in vec3 position;
            layout(location = 1) in vec3 vertex_color;

            uniform vec2 offset;
            uniform float heading;
            uniform float elevation;
            uniform mat4 view_projection;

            out vec3 surface_position;
            out vec3 surface_color;

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
                surface_color = vertex_color;

                gl_Position = view_projection * vec4(world_position, 1.0);
            }
        )glsl";

        constexpr char fragment_shader_source[] = R"glsl(
            #version 330 core

            in vec3 surface_position;

            uniform vec3 color;
            uniform bool curb_pattern;
            uniform bool start_line_enabled;
            uniform float track_radius;
            uniform float road_half_width;
            uniform float curb_segments;

            uniform bool lighting_enabled;
            uniform vec3 direction_to_light;

            out vec4 fragment_color;

            in vec3 surface_color;
            uniform bool use_vertex_color;

            uniform sampler2D shadow_depth;
            uniform mat4 light_view_projection;

            float shadow_amount(vec3 normal, vec3 light) {
                vec4 light_position =
                    light_view_projection * vec4(surface_position, 1.0);

                vec3 projected = light_position.xyz / light_position.w;
                projected = projected * 0.5 + 0.5;

                if (any(lessThan(projected, vec3(0.0))) ||
                    any(greaterThan(projected, vec3(1.0)))) {
                    return 0.0;
                }

                float bias = max(
                    0.0003 * (1.0 - max(dot(normal, light), 0.0)),
                    0.00005
                );

                vec2 texel_size =
                    1.0 / vec2(textureSize(shadow_depth, 0));

                float shadow = 0.0;

                for (int y = -1; y <= 1; ++y) {
                    for (int x = -1; x <= 1; ++x) {
                        vec2 sample_position =
                            projected.xy + vec2(x, y) * texel_size;

                        float closest_depth =
                            textureLod(shadow_depth, sample_position, 0.0).r;

                        if (projected.z - bias > closest_depth) {
                            shadow += 1.0;
                        }
                    }
                }

                return shadow / 9.0;
            }

            void main() {
                float brightness = 1.0;

                if (lighting_enabled) {
                    vec3 normal = normalize(cross(
                        dFdx(surface_position),
                        dFdy(surface_position)
                    ));

                    vec3 light = normalize(direction_to_light);

                    float diffuse = max(dot(normal, light), 0.0);

                    float shadow = shadow_amount(normal, light);

                    const float ambient = 0.35;
                    brightness =
                        ambient + (1.0 - ambient) * diffuse * (1.0 - shadow);
                }

                vec3 base_color = color;

                if (use_vertex_color) {
                    base_color = surface_color * color;
                }

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
                if (start_line_enabled) {
                    const float column_count = 10.0;

                    float cell_size =
                        (road_half_width * 2.0) / column_count;

                    float angle =
                        atan(surface_position.y, surface_position.x);

                    float distance_along_track = angle * track_radius;

                    if (abs(distance_along_track) < cell_size) {
                        float distance_across_track =
                            length(surface_position.xy)
                            - (track_radius - road_half_width);

                        float column = floor(distance_across_track / cell_size);
                        float row = floor(
                            (distance_along_track + cell_size) / cell_size
                        );

                        float checker = mod(column + row, 2.0);

                        base_color = mix(vec3(0.06),vec3(0.94),checker   );
                    }
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

        ShaderProgram shadow_program;
        ShadowMap shadow_map;

        GLint shadow_offset_location{};
        GLint shadow_heading_location{};
        GLint shadow_elevation_location{};
        GLint light_view_projection_location{};

        TrackMesh track_mesh;
        TrackMesh curb_mesh;
        CarMesh vehicle_mesh;
        GroundMesh ground_mesh;
        BarrierMesh barrier_mesh;
        TreeSettings tree_settings{};
        TreeMesh tree_mesh{tree_settings};
        std::vector<glm::vec2> tree_positions;

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
        GLint use_vertex_color_location{};
        GLint shadow_depth_location{};
        GLint scene_light_view_projection_location{};
        GLint start_line_enabled_location{};

        bool reset_was_pressed{};
        bool pause_was_pressed{};
        bool autopilot_was_pressed{};
        bool camera_reset_was_pressed{};
        bool orbit_dragging{};

        bool follow_vehicle{};
        bool follow_was_pressed{};

        double previous_cursor_x{};
        double previous_cursor_y{};
        double scroll_delta{};

        OrbitCamera camera{};

        explicit Impl(const simulation::TrackSettings &track)
            : track_settings{track}, track_mesh{track},
              curb_mesh{curb_geometry_settings(track)}, barrier_mesh{track} {
            constexpr int tree_count = 16;
            constexpr float tree_clearance = 0.10F;
            constexpr float radial_variation = 0.08F;

            const BarrierSettings barrier_settings{};

            const float base_radius =
                    static_cast<float>(
                        track.centerline_radius + track.road_width * 0.5
                    )
                    + barrier_settings.clearance
                    + barrier_settings.thickness
                    + tree_settings.crown_radius
                    + tree_clearance;

            tree_positions.reserve(tree_count);

            for (int index = 0; index < tree_count; ++index) {
                const float angle =
                        2.0F * std::numbers::pi_v<float>
                        * (static_cast<float>(index) + 0.5F)
                        / static_cast<float>(tree_count);

                const float radius =
                        base_radius
                        + radial_variation * 0.5F
                        * (1.0F + std::sin(angle * 3.0F));

                tree_positions.emplace_back(
                    radius * std::cos(angle),
                    radius * std::sin(angle)
                );
            }
            Shader vertex_shader{GL_VERTEX_SHADER};
            vertex_shader.compile(vertex_shader_source);

            Shader fragment_shader{GL_FRAGMENT_SHADER};
            fragment_shader.compile(fragment_shader_source);

            program.link(vertex_shader, fragment_shader);

            Shader shadow_vertex_shader{GL_VERTEX_SHADER};
            shadow_vertex_shader.compile(shadow_vertex_shader_source);

            Shader shadow_fragment_shader{GL_FRAGMENT_SHADER};
            shadow_fragment_shader.compile(shadow_fragment_shader_source);

            shadow_program.link(shadow_vertex_shader, shadow_fragment_shader);

            shadow_offset_location = shadow_program.uniform_location("offset");
            shadow_heading_location = shadow_program.uniform_location("heading");
            shadow_elevation_location = shadow_program.uniform_location("elevation");
            light_view_projection_location = shadow_program.uniform_location("light_view_projection");

            offset_location = program.uniform_location("offset");
            view_projection_location = program.uniform_location("view_projection");
            elevation_location = program.uniform_location("elevation");
            heading_location = program.uniform_location("heading");
            color_location = program.uniform_location("color");

            curb_pattern_location = program.uniform_location("curb_pattern");
            start_line_enabled_location = program.uniform_location("start_line_enabled");
            track_radius_location = program.uniform_location("track_radius");
            road_half_width_location = program.uniform_location("road_half_width");
            curb_segments_location = program.uniform_location("curb_segments");

            lighting_enabled_location = program.uniform_location("lighting_enabled");
            direction_to_light_location = program.uniform_location("direction_to_light");

            use_vertex_color_location = program.uniform_location("use_vertex_color");

            shadow_depth_location = program.uniform_location("shadow_depth");
            scene_light_view_projection_location = program.uniform_location("light_view_projection");

            glClearColor(0.04F, 0.12F, 0.14F, 1.0F);
            glEnable(GL_DEPTH_TEST);

            glfwSetWindowUserPointer(window.get(), this);

            glfwSetScrollCallback(
                window.get(),
                [](GLFWwindow *window, double, double vertical_offset) {
                    auto *view = static_cast<Impl *>(glfwGetWindowUserPointer(window));

                    view->scroll_delta += vertical_offset;
                }
            );
        }

        void draw_trees(GLint offset_uniform, GLint heading_uniform, GLint elevation_uniform, float elevation) const {
            glUniform1f(heading_uniform, 0.0F);
            glUniform1f(elevation_uniform, elevation);

            for (const auto &position: tree_positions) {
                glUniform2f(offset_uniform, position.x, position.y);
                tree_mesh.draw();
            }
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

        const bool focused =
                glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE;

        if (focused) {
            if (pressed_once(GLFW_KEY_F, view.follow_was_pressed)) {
                view.follow_vehicle = !view.follow_vehicle;

                if (!view.follow_vehicle) {
                    view.camera.reset();
                }
            }

            if (pressed_once(GLFW_KEY_C, view.camera_reset_was_pressed)) {
                view.camera.reset();
                view.orbit_dragging = false;
                view.scroll_delta = 0.0;
                view.follow_vehicle = false;
            }

            double cursor_x{};
            double cursor_y{};
            glfwGetCursorPos(window, &cursor_x, &cursor_y);

            const bool dragging =
                    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ||
                    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

            if (dragging && view.orbit_dragging) {
                constexpr double orbit_sensitivity = 0.005;

                const double horizontal_change =
                        cursor_x - view.previous_cursor_x;

                const double vertical_change =
                        cursor_y - view.previous_cursor_y;

                view.camera.orbit(
                    static_cast<float>(-horizontal_change * orbit_sensitivity),
                    static_cast<float>(vertical_change * orbit_sensitivity)
                );
            }

            view.previous_cursor_x = cursor_x;
            view.previous_cursor_y = cursor_y;
            view.orbit_dragging = dragging;

            if (view.scroll_delta != 0.0) {
                const float amount = static_cast<float>(std::clamp(view.scroll_delta, -50.0, 50.0));

                view.camera.zoom(amount);
                view.scroll_delta = 0.0;
            }
        } else {
            view.orbit_dragging = false;
            view.camera_reset_was_pressed = false;
            view.scroll_delta = 0.0;
            view.follow_was_pressed = false;
        }

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

        constexpr float road_elevation = 0.0F;
        constexpr float ground_elevation = -0.005F;
        constexpr float sensor_elevation = 0.002F;
        constexpr float vehicle_elevation = 0.004F;

        const glm::vec3 direction_to_light =
                glm::normalize(glm::vec3{-0.5F, -0.4F, 1.0F});

        float shadow_extent = std::max(
            1.0F, static_cast<float>(view.track_settings.centerline_radius + view.track_settings.road_width * 0.5) +
                  0.3F);

        const float tree_height = view.tree_settings.trunk_height + view.tree_settings.crown_radius * 1.5F;

        for (const auto &position: view.tree_positions) {
            shadow_extent = std::max(shadow_extent,
                                     glm::length(position) + view.tree_settings.crown_radius + tree_height);
        }

        const glm::mat4 light_projection = glm::ortho(-shadow_extent, shadow_extent, -shadow_extent, shadow_extent,
                                                      0.1F, shadow_extent * 6.0F);

        const glm::mat4 light_view = glm::lookAt(direction_to_light * (shadow_extent * 3.0F), glm::vec3{0.0F},
                                                 glm::vec3{0.0F, 0.0F, 1.0F});

        const glm::mat4 light_view_projection =
                light_projection * light_view;

        view.shadow_map.bind_for_writing();
        view.shadow_program.use();

        glUniformMatrix4fv(view.light_view_projection_location, 1, GL_FALSE, glm::value_ptr(light_view_projection));

        glUniform2f(view.shadow_offset_location, 0.0F, 0.0F);
        glUniform1f(view.shadow_heading_location, 0.0F);
        glUniform1f(view.shadow_elevation_location, ground_elevation);

        view.barrier_mesh.draw();

        view.draw_trees(view.shadow_offset_location, view.shadow_heading_location, view.shadow_elevation_location,
                        ground_elevation);

        const auto &vehicle_position = world.vehicle.body.position;

        glUniform2f(view.shadow_offset_location, static_cast<float>(vehicle_position.x),
                    static_cast<float>(vehicle_position.y));
        glUniform1f(view.shadow_heading_location, static_cast<float>(world.vehicle.heading));
        glUniform1f(view.shadow_elevation_location, vehicle_elevation);

        view.vehicle_mesh.draw();

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        view.program.use();

        glUniform1i(view.start_line_enabled_location, GL_FALSE);

        glUniform1f(view.track_radius_location, static_cast<float>(view.track_settings.centerline_radius));
        glUniform1f(view.road_half_width_location, static_cast<float>(view.track_settings.road_width * 0.5));

        view.shadow_map.bind_texture(0);
        glUniform1i(view.shadow_depth_location, 0);
        glUniformMatrix4fv(
            view.scene_light_view_projection_location,
            1,
            GL_FALSE,
            glm::value_ptr(light_view_projection)
        );

        glUniform1i(view.use_vertex_color_location, GL_FALSE);

        glUniform1i(view.lighting_enabled_location, GL_TRUE);
        glUniform3fv(view.direction_to_light_location, 1, glm::value_ptr(direction_to_light));

        const float aspect =
                static_cast<float>(width) / static_cast<float>(height);

        if (view.follow_vehicle) {
            const auto &position = world.vehicle.body.position;
            constexpr float follow_height = 0.03F;

            view.camera.set_target(glm::vec3{
                static_cast<float>(position.x),
                static_cast<float>(position.y),
                follow_height
            });
        }

        const auto &camera = view.camera.settings();

        const glm::mat4 projection = glm::perspective(glm::radians(camera.vertical_fov_degrees), aspect,
                                                      camera.near_clip, camera.far_clip);

        const glm::mat4 camera_view =
                glm::lookAt(
                    camera.position, camera.target, camera.up);

        const glm::mat4 view_projection = projection * camera_view;

        glUniformMatrix4fv(view.view_projection_location, 1, GL_FALSE, glm::value_ptr(view_projection));


        glUniform2f(view.offset_location, 0.0F, 0.0F);
        glUniform1f(view.heading_location, 0.0F);


        glUniform1i(view.curb_pattern_location, GL_FALSE);
        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.68F, 0.65F, 0.53F);

        view.ground_mesh.draw();

        glUniform1f(view.elevation_location, road_elevation);
        glUniform3f(view.color_location, 0.25F, 0.27F, 0.29F);
        glUniform1i(view.curb_pattern_location, GL_FALSE);
        glUniform1i(view.start_line_enabled_location, GL_TRUE);
        view.track_mesh.draw();
        glUniform1i(view.start_line_enabled_location, GL_FALSE);

        constexpr float curb_elevation = 0.001F;
        constexpr float curb_segments = 80.0F;

        glUniform1i(view.curb_pattern_location, GL_TRUE);
        glUniform1f(view.curb_segments_location, curb_segments);
        glUniform1f(view.elevation_location, curb_elevation);

        view.curb_mesh.draw();

        glUniform1i(view.curb_pattern_location, GL_FALSE);

        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.16F, 0.17F, 0.18F);
        view.barrier_mesh.draw();

        glUniform1i(view.use_vertex_color_location, GL_TRUE);
        glUniform3f(view.color_location, 1.0F, 1.0F, 1.0F);

        view.draw_trees(view.offset_location, view.heading_location, view.elevation_location, ground_elevation);

        glUniform1i(view.use_vertex_color_location, GL_FALSE);
        glUniform2f(view.offset_location, 0.0F, 0.0F);
        glUniform1f(view.heading_location, 0.0F);

        const auto readings = simulation::sense_track(world.vehicle, view.track_settings, sensors);

        view.sensor_lines.update(world.vehicle, sensors, readings);

        glUniform1f(view.elevation_location, sensor_elevation);
        glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        glUniform1i(view.lighting_enabled_location, GL_FALSE);
        view.sensor_lines.draw();
        glUniform1i(view.lighting_enabled_location, GL_TRUE);

        const auto &position =
                world.vehicle.body.position;
        const bool on_track =
                simulation::is_on_track(position, view.track_settings);

        if (on_track) {
            glUniform3f(view.color_location, 1.0F, 1.0F, 1.0F);
        } else {
            glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
        }

        glUniform2f(view.offset_location, static_cast<float>(position.x), static_cast<float>(position.y));
        glUniform1f(view.heading_location, static_cast<float>(world.vehicle.heading));
        glUniform1f(view.elevation_location, vehicle_elevation);

        glUniform1i(view.use_vertex_color_location, GL_TRUE);
        view.vehicle_mesh.draw();
        glUniform1i(view.use_vertex_color_location, GL_FALSE);

        glfwSwapBuffers(window);
    }

    void SimulationWindow::close() noexcept {
        impl_.reset();
    }
}
