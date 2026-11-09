#pragma once

#include <glad/gl.h>

namespace apex::graphics {
    struct TreeSettings {
        float trunk_height{0.10F};
        float trunk_radius{0.012F};
        float crown_radius{0.07F};
        int segment_count{8};
    };

    class TreeMesh {
    public:
        explicit TreeMesh(const TreeSettings& settings = {});
        ~TreeMesh();

        TreeMesh(const TreeMesh&) = delete;
        TreeMesh& operator=(const TreeMesh&) = delete;

        void draw() const;

    private:
        GLuint vertex_array_{};
        GLuint vertex_buffer_{};
        GLsizei vertex_count_{};
    };
}