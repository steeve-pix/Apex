#include "platform/glfw_session.hpp"
#define GLFW_INCLUDE_NONE

#include <GLFW/glfw3.h>

#include <iostream>
#include <stdexcept>

namespace apex::platform {
    namespace {
        void report_glfw_error(int code, const char *description) {
            std::cerr << "GLFW error " << code << ": " << description << '\n';
        }
    }

    GlfwSession::GlfwSession() {
        glfwSetErrorCallback(report_glfw_error);

        if (glfwInit() != GLFW_TRUE) {
            throw std::runtime_error("Failed to initialize GLFW");
        }
    }

    GlfwSession::~GlfwSession() {
        glfwTerminate();
    }
}
