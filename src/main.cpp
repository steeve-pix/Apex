#define GLFW_INCLUDE_NONE
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

    class GlfwSession {
    public:
        GlfwSession() {
            glfwSetErrorCallback(report_glfw_error);

            if (glfwInit() != GLFW_TRUE){
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
        glfwWindowHint(GLFW_CLIENT_API,GLFW_NO_API);

        Window window{
            glfwCreateWindow(window_width, window_height, "Apex", nullptr, nullptr)
        };

        if (!window) {
            throw std::runtime_error("Failed to create the Apex window");
        }

        while (glfwWindowShouldClose(window.get()) == GLFW_FALSE) {
            glfwWaitEvents();
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
