#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <cmath>
#include "platform/glfw_session.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/triangle.hpp"
#include "math/vec2.hpp"

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

        void main() {
            gl_Position = vec4((position + offset) * view_scale, 0.0, 1.0);
        }
    )glsl";

    constexpr char fragment_shader_source[] = R"glsl(
        #version 330 core

        out vec4 fragment_color;

        void main() {
            fragment_color = vec4(0.95, 0.25, 0.12, 1.0);
        }
    )glsl";

    struct WindowDeleter {
        void operator()(GLFWwindow *window) const noexcept {
            glfwDestroyWindow(window);
        }
    };

    using Window = std::unique_ptr<GLFWwindow, WindowDeleter>;

    apex::math::Vec2 read_movement_direction(GLFWwindow *window) {
        apex::math::Vec2 direction{};

        if (glfwGetKey(window,GLFW_KEY_RIGHT) == GLFW_PRESS ||
            glfwGetKey(window,GLFW_KEY_D) == GLFW_PRESS) {
            direction.x += 1.0;
        }
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
            direction.x -= 1.0;
        }

        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
            direction.y += 1.0;
        }

        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS ||
            glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
            direction.y -= 1.0;
        }

        const double length = std::hypot(direction.x, direction.y);

        if (length > 0.0) {
            direction.x /= length;
            direction.y /= length;
        }

        return direction;
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

        std::cout << "Shader program linked successfully.\n";

        apex::graphics::Triangle triangle;

        glClearColor(0.04f, 0.12f, 0.14f, 1.0f);

        double previous_time = glfwGetTime();
        constexpr double movement_speed = 0.75;
        apex::math::Vec2 position{};

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

            const auto direction = read_movement_direction(window.get());
            if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
            }

            position.x +=
                    direction.x * movement_speed * delta_time;

            position.y +=
                    direction.y * movement_speed * delta_time;

            glClear(GL_COLOR_BUFFER_BIT);

            report_elapsed += delta_time;

            program.use();

            const float aspect =
                    static_cast<float>(framebuffer_width) / static_cast<float>(framebuffer_height);

            if (aspect >= 1.0F) {
                glUniform2f(view_scale_location, 1.0F / aspect, 1.0F);
            } else {
                glUniform2f(view_scale_location, 1.0F, aspect);
            }
            glUniform2f(offset_location, static_cast<float>(position.x), static_cast<float>(position.y));

            triangle.draw();

            glfwSwapBuffers(window.get());
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
