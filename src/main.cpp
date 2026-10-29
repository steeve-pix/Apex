#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <memory>

namespace {
    constexpr int window_width = 1200;
    constexpr int window_height = 720;

    void report_glfw_error(int code, const char *description) {
        std::cerr << "GLFW error " << code << ": " << description << '\n';
    }

    void update_viewport(GLFWwindow *, int width, int height) {
        glViewport(0, 0, width, height);
    }

    class GlfwSession {
    public:
        GlfwSession() {
            glfwSetErrorCallback(report_glfw_error);

            if (glfwInit() != GLFW_TRUE) {
                throw std::runtime_error("Failed to initialize GLFW");
            }
        }

        ~GlfwSession() {
            glfwTerminate();
        }

        GlfwSession(const GlfwSession &) = delete;

        GlfwSession &operator=(const GlfwSession &) = delete;
    };

    struct WindowDeleter {
        void operator()(GLFWwindow *window) const noexcept {
            glfwDestroyWindow(window);
        }
    };

    using Window = std::unique_ptr<GLFWwindow, WindowDeleter>;
}

int main() {
    try {
        GlfwSession glfw;
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

        Window window{
            glfwCreateWindow(window_width, window_height, "Apex", nullptr, nullptr)
        };

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


        if (!window) {
            throw std::runtime_error("Failed to create the Apex window");
        }

        glClearColor(0.04f, 0.12f, 0.14f, 1.0f);

        while (glfwWindowShouldClose(window.get()) == GLFW_FALSE) {
            glfwPollEvents();

            if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
            }

            glClear(GL_COLOR_BUFFER_BIT);

            glfwSwapBuffers(window.get());
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
