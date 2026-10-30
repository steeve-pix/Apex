#pragma once

namespace apex::platform {
    class GlfwSession {
    public:
        GlfwSession();

        ~GlfwSession();

        GlfwSession(const GlfwSession &) = delete;

        GlfwSession &operator=(const GlfwSession &) = delete;
    };
}
