#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>

namespace {
    constexpr int window_width = 1200;
    constexpr int window_height = 720;

    void report_glfw_error(int code, const char *description) {
        std::cerr << "GLFW error " << code << ": " << description << '\n';
    }
}

int main() {
    glfwSetErrorCallback(report_glfw_error);

    if (glfwInit() != GLFW_TRUE) return EXIT_FAILURE;

    glfwWindowHint(GLFW_CLIENT_API,GLFW_NO_API);

    GLFWwindow *window = glfwCreateWindow(window_width, window_height, "Apex", nullptr, nullptr);

    if (window == nullptr) {
        glfwTerminate();
        return EXIT_FAILURE;
    }

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        glfwWaitEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;

}
