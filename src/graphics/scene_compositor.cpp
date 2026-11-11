#include "graphics/scene_compositor.hpp"

#include <algorithm>
#include <stdexcept>

#include "graphics/shader.hpp"

namespace apex::graphics {
    SceneCompositor::SceneCompositor() {
        Shader vertex{GL_VERTEX_SHADER};
        vertex.compile(R"glsl(
            #version 330 core

            out vec2 uv;

            void main() {
                vec2 position = gl_VertexID == 0 ? vec2(-1,-1) : (gl_VertexID == 1 ? vec2(3,-1) : vec2(-1,3));
                uv = position * .5 + .5;
                gl_Position = vec4(position,0,1);
            }
        )glsl");
        Shader fragment{GL_FRAGMENT_SHADER};
        fragment.compile(R"glsl(
            #version 330 core

            in vec2 uv;
            uniform sampler2D scene_image;
            uniform bool grade;
            uniform bool overview;
            out vec4 output_color;

            vec3 bloom_sample(vec2 position) {
                vec3 source = texture(scene_image,position).rgb;
                float peak = max(source.r,max(source.g,source.b));
                return source * (smoothstep(.75,1.85,peak) / max(1.0,peak));
            }

            void main() {
                vec3 color = max(texture(scene_image,uv).rgb,vec3(0.0));
                if (grade) {
                    vec2 size = vec2(textureSize(scene_image,0));
                    vec2 step_size = vec2(size.y/size.x,1.0) * .0065;
                    vec3 glow = bloom_sample(uv) * .18;
                    glow += .085 * (bloom_sample(uv + vec2(step_size.x,0))
                                   + bloom_sample(uv - vec2(step_size.x,0))
                                   + bloom_sample(uv + vec2(0,step_size.y))
                                   + bloom_sample(uv - vec2(0,step_size.y)));
                    glow += .12 * (bloom_sample(uv + step_size)
                                 + bloom_sample(uv - step_size)
                                 + bloom_sample(uv + vec2(step_size.x,-step_size.y))
                                 + bloom_sample(uv + vec2(-step_size.x,step_size.y)));
                    color += glow * .15;
                    float brightness = dot(color,vec3(.213,.715,.072));
                    vec3 tint = mix(vec3(.92,1.025,1.13),vec3(1.08,1.005,.93),
                                    smoothstep(.024,.55,brightness));
                    color = pow(color * tint,vec3(.94)) * 1.06;
                    float edge = smoothstep(.26,1.18,length((uv - .5)*2.0));
                    color *= 1.0 - edge * (overview ? .095 : .17);

                    float peak = max(color.r,max(color.g,color.b));
                    if (peak > .71) {
                        float shoulder = .71 + .29 * (1.0 - exp(-(peak-.71)/.29));
                        color *= shoulder / peak;
                    }
                }
                output_color = vec4(pow(color,vec3(1.0/2.2)),1.0);
            }
        )glsl");
        program_.link(vertex, fragment);
        program_.use();
        glUniform1i(program_.uniform_location("scene_image"), 0);
        grade_ = program_.uniform_location("grade");
        overview_ = program_.uniform_location("overview");

        GLint maximum_samples{};
        glGetIntegerv(GL_MAX_SAMPLES, &maximum_samples);
        samples_ = std::min(4, maximum_samples);
        glGenVertexArrays(1, &array_);
        glGenFramebuffers(1, &scene_);
        glGenFramebuffers(1, &resolve_);
        glGenRenderbuffers(1, &color_);
        glGenRenderbuffers(1, &depth_);
        glGenTextures(1, &image_);
    }

    SceneCompositor::~SceneCompositor() {
        glDeleteTextures(1, &image_);
        glDeleteRenderbuffers(1, &depth_);
        glDeleteRenderbuffers(1, &color_);
        glDeleteFramebuffers(1, &resolve_);
        glDeleteFramebuffers(1, &scene_);
        glDeleteVertexArrays(1, &array_);
    }

    void SceneCompositor::begin(int width, int height) {
        glBindFramebuffer(GL_FRAMEBUFFER, scene_);
        if (width != width_ || height != height_) {
            glBindRenderbuffer(GL_RENDERBUFFER, color_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_RGBA16F, width, height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color_);
            glBindRenderbuffer(GL_RENDERBUFFER, depth_);
            glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples_, GL_DEPTH_COMPONENT24, width, height);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
            glBindRenderbuffer(GL_RENDERBUFFER, 0);
            const auto scene_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);

            glBindFramebuffer(GL_FRAMEBUFFER, resolve_);
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, image_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_HALF_FLOAT, nullptr);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image_, 0);
            if (scene_status != GL_FRAMEBUFFER_COMPLETE ||
                glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
                glBindFramebuffer(GL_FRAMEBUFFER, 0);
                throw std::runtime_error("Apex could not allocate the scene framebuffer");
            }
            width_ = width;
            height_ = height;
            glBindFramebuffer(GL_FRAMEBUFFER, scene_);
        }
        glViewport(0, 0, width_, height_);
        glClearColor(.001F,.009F,.013F,1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    }

    void SceneCompositor::present(bool grade, bool overview) const {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, scene_);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolve_);
        glBlitFramebuffer(0, 0, width_, height_, 0, 0, width_, height_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDisable(GL_DEPTH_TEST);

        program_.use();
        glUniform1i(grade_, grade);
        glUniform1i(overview_, overview);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, image_);
        glBindVertexArray(array_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glEnable(GL_DEPTH_TEST);
    }
}
