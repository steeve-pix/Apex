#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <memory>
#include <string>
#include <array>

namespace {
    constexpr int window_width = 1200;
    constexpr int window_height = 720;

    void report_glfw_error(int code, const char *description) {
        std::cerr << "GLFW error " << code << ": " << description << '\n';
    }

    void update_viewport(GLFWwindow *, int width, int height) {
        glViewport(0, 0, width, height);
    }

    constexpr char vertex_shader_source[] = R"glsl(
        #version 330 core

        layout(location = 0) in vec2 position;

        void main() {
            gl_Position = vec4(position, 0.0, 1.0);
        }
    )glsl";

    constexpr char fragment_shader_source[] = R"glsl(
        #version 330 core

        out vec4 fragment_color;

        void main() {
            fragment_color = vec4(0.95, 0.25, 0.12, 1.0);
        }
    )glsl";

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

    class Shader {
    public:
        explicit Shader(GLenum type) : id_{glCreateShader(type)} {
            if (id_ == 0) {
                throw std::runtime_error("Failed to create shader");
            }
        }

        ~Shader() {
            glDeleteShader(id_);
        }

        Shader(const Shader &) = delete;

        Shader &operator=(const Shader &) = delete;

        void compile(const char *source) const {
            glShaderSource(id_, 1, &source, nullptr);
            glCompileShader(id_);

            GLint compiled{};
            glGetShaderiv(id_,GL_COMPILE_STATUS, &compiled);

            if (compiled == GL_TRUE) {
                return;
            }

            GLint log_length{};
            glGetShaderiv(id_,GL_INFO_LOG_LENGTH, &log_length);

            std::string log;

            if (log_length > 0) {
                log.resize(static_cast<std::string::size_type>(log_length));

                glGetShaderInfoLog(id_, log_length, nullptr, log.data());
            }

            throw std::runtime_error("Shader compilation failed:\n" + log);
        }

        [[nodiscard]] GLuint get() const noexcept {
            return id_;
        }

    private:
        GLuint id_;
    };

    class ShaderProgram {
    public:
        ShaderProgram()
            : id_{glCreateProgram()} {
            if (id_ == 0) {
                throw std::runtime_error("Failed to create shader program");
            }
        }

        ~ShaderProgram() {
            glDeleteProgram(id_);
        }

        ShaderProgram(const ShaderProgram &) = delete;

        ShaderProgram &operator=(const ShaderProgram &) = delete;

        void link(const Shader &vertex, const Shader &fragment) const {
            glAttachShader(id_, vertex.get());
            glAttachShader(id_, fragment.get());

            glLinkProgram(id_);

            glDetachShader(id_, vertex.get());
            glDetachShader(id_, fragment.get());

            GLint linked{};
            glGetProgramiv(id_, GL_LINK_STATUS, &linked);

            if (linked == GL_TRUE) {
                return;
            }

            GLint log_length{};
            glGetProgramiv(id_, GL_INFO_LOG_LENGTH, &log_length);

            std::string log;

            if (log_length > 0) {
                log.resize(static_cast<std::string::size_type>(log_length));

                glGetProgramInfoLog(id_, log_length, nullptr, log.data());
            }

            throw std::runtime_error("Shader program linking failed:\n" + log);
        }

        void use() const {
            glUseProgram(id_);
        }

    private:
        GLuint id_;
    };

    struct Vertex {
        float x;
        float y;
    };

    class Triangle {
    public:
        Triangle() {
            constexpr std::array vertices{
                Vertex{-0.5F, -0.5F},
                Vertex{0.5F, -0.5F},
                Vertex{0.5F, 0.5F},
            };

            vertex_count_ = static_cast<GLsizei>(vertices.size());

            glGenVertexArrays(1, &vertex_array_);
            glGenBuffers(1, &vertex_buffer_);

            if (vertex_array_ == 0 || vertex_buffer_ == 0) {
                glDeleteBuffers(1, &vertex_buffer_);
                glDeleteVertexArrays(1, &vertex_array_);

                throw std::runtime_error("Failed to create Triangle resources");
            }

            glBindVertexArray(vertex_array_);
            glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

            glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                         vertices.data(),
                         GL_STATIC_DRAW
            );

            glVertexAttribPointer(0, 2,GL_FLOAT,GL_FALSE, static_cast<GLsizei>(sizeof(Vertex)), nullptr);

            glEnableVertexAttribArray(0);

            glBindVertexArray(0);

            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        ~Triangle() {
            glDeleteBuffers(1, &vertex_buffer_);

            glDeleteVertexArrays(1, &vertex_array_);
        }

        Triangle(const Triangle &) = delete;

        Triangle &operator=(const Triangle &) = delete;

        void draw() const {
            glBindVertexArray(vertex_array_);
            glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
        }

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
};


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

        Shader vertex_shader{GL_VERTEX_SHADER};
        vertex_shader.compile(vertex_shader_source);

        Shader fragment_shader{GL_FRAGMENT_SHADER};
        fragment_shader.compile(fragment_shader_source);

        std::cout << "Both shaders compiled successfully.\n";

        ShaderProgram program;
        program.link(vertex_shader, fragment_shader);

        std::cout << "Shader program linked successfully.\n";

        Triangle triangle;

        glClearColor(0.04f, 0.12f, 0.14f, 1.0f);

        while (glfwWindowShouldClose(window.get()) == GLFW_FALSE) {
            glfwPollEvents();

            if (glfwGetKey(window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
                glfwSetWindowShouldClose(window.get(), GLFW_TRUE);
            }

            glClear(GL_COLOR_BUFFER_BIT);

            program.use();
            triangle.draw();

            glfwSwapBuffers(window.get());
        }

        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Apex: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
