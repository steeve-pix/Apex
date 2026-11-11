#include "graphics/race_hud.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <numbers>
#include <numeric>
#include <optional>
#include <sstream>
#include <string_view>
#include <vector>

#include <glm/gtc/type_ptr.hpp>

#include "graphics/shader.hpp"
#include "graphics/simulation_window.hpp"
#include "hud_font.hpp"

namespace apex::graphics {
    namespace {
        struct Vertex {
            float x, y;
            glm::vec4 color;
            glm::vec2 uv{-1.0F};
        };

        constexpr glm::vec4 cream{.98F,.95F,.88F,1};
        constexpr glm::vec4 muted{.73F,.75F,.75F,1};
        constexpr glm::vec4 red{1.0F,.36F,.28F,1};
        constexpr glm::vec4 green{.47F,.83F,.68F,1};
        constexpr glm::vec4 panel_color{.045F,.057F,.073F,.94F};
        constexpr glm::vec4 rule{.44F,.49F,.54F,.30F};

        struct Bounds {
            float x, y, width, height;

            bool overlaps(Bounds other, float gap = 3.0F) const {
                return x < other.x + other.width + gap && x + width > other.x - gap
                    && y < other.y + other.height + gap && y + height > other.y - gap;
            }
        };

        struct Canvas {
            float width;
            float height;
            std::vector<Vertex> vertices;

            void rect(float x, float y, float w, float h, glm::vec4 color) {
                const float l = x * 2 / width - 1;
                const float r = (x + w) * 2 / width - 1;
                const float t = 1 - y * 2 / height;
                const float b = 1 - (y + h) * 2 / height;
                vertices.insert(vertices.end(), {{l,t,color}, {l,b,color}, {r,b,color},
                                                 {l,t,color}, {r,b,color}, {r,t,color}});
            }

            void line(float x0, float y0, float x1, float y1, float thickness, glm::vec4 color) {
                const float dx = x1 - x0;
                const float dy = y1 - y0;
                const float length = std::hypot(dx, dy);
                if (length == 0.0F) return;
                const float nx = dy / length * thickness * 0.5F;
                const float ny = -dx / length * thickness * 0.5F;
                const auto point = [&](float x, float y) {
                    return Vertex{x * 2 / width - 1, 1 - y * 2 / height, color};
                };
                const auto a = point(x0 + nx, y0 + ny), b = point(x0 - nx, y0 - ny);
                const auto c = point(x1 - nx, y1 - ny), d = point(x1 + nx, y1 + ny);
                vertices.insert(vertices.end(), {a,b,c,a,c,d});
            }

            void cross(glm::vec2 p, float radius, float thickness, glm::vec4 color) {
                line(p.x-radius,p.y-radius,p.x+radius,p.y+radius,thickness,color);
                line(p.x-radius,p.y+radius,p.x+radius,p.y-radius,thickness,color);
            }

            float text_width(std::string_view label, float size, float tracking = 0.0F) const {
                float width = 0.0F;
                for (char c : label) {
                    const auto index = std::string_view{hud_font::characters}.find(c);
                    width += index == std::string_view::npos ? size * .29F :
                        hud_font::glyphs[index].advance * size / (64.0F * hud_font::design_size) + tracking;
                }
                return width;
            }

            void text(float x, float y, std::string_view label, float size, glm::vec4 color, float tracking = 0.0F) {
                const float scale = size / hud_font::design_size;
                for (char c : label) {
                    const auto index = std::string_view{hud_font::characters}.find(c);
                    if (index == std::string_view::npos) {
                        x += size * .29F;
                        continue;
                    }
                    const auto &g = hud_font::glyphs[index];
                    const float l = (x + g.left * scale) * 2 / width - 1;
                    const float r = l + g.width * scale * 2 / width;
                    const float t = 1 - (y + g.top * scale) * 2 / height;
                    const float b = t - g.height * scale * 2 / height;
                    const float u0 = static_cast<float>(g.x) / hud_font::width;
                    const float u1 = static_cast<float>(g.x + g.width) / hud_font::width;
                    const float v0 = static_cast<float>(g.y) / hud_font::height;
                    const float v1 = static_cast<float>(g.y + g.height) / hud_font::height;
                    vertices.insert(vertices.end(), {{l,t,color,{u0,v0}}, {l,b,color,{u0,v1}}, {r,b,color,{u1,v1}},
                                                     {l,t,color,{u0,v0}}, {r,b,color,{u1,v1}}, {r,t,color,{u1,v0}}});
                    x += g.advance * scale / 64.0F + tracking;
                }
            }

            void rounded_rect(Bounds bounds, float radius, glm::vec4 color) {
                const auto [x, y, w, h] = bounds;
                const auto point = [&](float px, float py) {
                    return Vertex{px * 2 / width - 1, 1 - py * 2 / height, color};
                };
                const std::array centers{
                    glm::vec2{x+w-radius,y+radius}, glm::vec2{x+w-radius,y+h-radius},
                    glm::vec2{x+radius,y+h-radius}, glm::vec2{x+radius,y+radius}
                };
                const auto center = point(x + w * .5F, y + h * .5F);
                auto previous = point(x + radius, y);
                for (std::size_t corner = 0; corner < centers.size(); ++corner) {
                    for (int step = 0; step <= 4; ++step) {
                        const float angle = (static_cast<float>(corner) - 1.0F + step * .25F)
                                            * (std::numbers::pi_v<float> * .5F);
                        const auto next = point(centers[corner].x + radius * std::cos(angle),
                                                centers[corner].y + radius * std::sin(angle));
                        vertices.insert(vertices.end(), {center, previous, next});
                        previous = next;
                    }
                }
            }

            void panel(Bounds bounds) {
                rounded_rect(bounds, 8.0F, panel_color);
            }

            void dot(float x, float y, float radius, glm::vec4 color) {
                const auto p = [&](float dx, float dy) {
                    return Vertex{(x + dx) * 2 / width - 1, 1 - (y + dy) * 2 / height, color};
                };
                for (int i = 0; i < 12; ++i) {
                    const float a = i * 0.5235988F, b = (i + 1) * 0.5235988F;
                    vertices.insert(vertices.end(), {p(0,0), p(radius*std::cos(a),radius*std::sin(a)),
                                                            p(radius*std::cos(b),radius*std::sin(b))});
                }
            }
        };

        std::string lap_time(std::optional<double> value) {
            if (!value) return "--:--.---";
            const auto ms = std::llround(*value * 1000.0);
            std::ostringstream result;
            result << std::setfill('0') << std::setw(2) << ms / 60000 << ':'
                   << std::setw(2) << ms / 1000 % 60 << '.' << std::setw(3) << ms % 1000;
            return result.str();
        }

        std::string decimal(double value) {
            std::ostringstream result;
            result << std::fixed << std::setprecision(2) << value;
            return result.str();
        }

        std::string car_number(std::size_t slot) {
            return (slot < 9 ? "0" : "") + std::to_string(slot + 1);
        }

        struct CircuitMap {
            math::Vec2 center;
            glm::vec2 screen_center;
            double scale;

            CircuitMap(const simulation::TrackPath &path, Bounds area) {
                auto lo = path.points().front();
                auto hi = lo;
                for (const auto &p : path.points()) {
                    lo.x = std::min(lo.x,p.x);
                    lo.y = std::min(lo.y,p.y);
                    hi.x = std::max(hi.x,p.x);
                    hi.y = std::max(hi.y,p.y);
                }
                center = {std::midpoint(lo.x,hi.x),std::midpoint(lo.y,hi.y)};
                screen_center = {area.x + area.width*.5F,area.y + area.height*.5F};
                scale = 1.0 / std::max((hi.x-lo.x)/area.width,(hi.y-lo.y)/area.height);
            }

            glm::vec2 operator()(math::Vec2 p) const {
                return {screen_center.x + (p.x-center.x)*scale,screen_center.y - (p.y-center.y)*scale};
            }

            void draw(Canvas &ui, const simulation::TrackPath &path, float thickness, glm::vec4 color) const {
                const auto points = path.points();
                for (std::size_t i = 0; i < points.size(); ++i) {
                    const auto a = (*this)(points[i]);
                    const auto b = (*this)(points[(i + 1) % points.size()]);
                    ui.line(a.x,a.y,b.x,b.y,thickness,color);
                }
                const auto start = (*this)(path.sample_at(simulation::start_line_distance).position);
                ui.rect(start.x-1,start.y-4,2,8,cream);
            }
        };

        void draw_driver_marker(Canvas &ui, glm::vec2 marker, const BroadcastDriver &driver) {
            if (driver.eliminated) {
                ui.cross(marker,3,1.65F,red);
            } else if (driver.lap_time) {
                ui.line(marker.x-3,marker.y,marker.x-1,marker.y+2,1.6F,green);
                ui.line(marker.x-1,marker.y+2,marker.x+4,marker.y-3,1.6F,green);
            } else {
                ui.rounded_rect({marker.x-3.5F,marker.y-3.5F,7,7},2,{driver.livery,1});
            }
        }

        Bounds draw_circuit_standings(Canvas &ui, std::span<const BroadcastDriver> standings,
                                      std::size_t focused_slot, const TrainingTelemetry &training) {
            std::array<std::size_t,4> ranks{0,1,2,0};
            const auto leading_count = std::min(standings.size(),std::size_t{3});
            auto visible_count = leading_count;
            const auto focused = std::ranges::find(standings,focused_slot,&BroadcastDriver::slot);
            const auto focused_rank = static_cast<std::size_t>(focused - standings.begin());
            if (focused != standings.end() && focused_rank >= leading_count) {
                ranks[visible_count++] = focused_rank;
            }

            constexpr float tile_width = 151;
            const float width = 12 + visible_count * (tile_width + 4);
            const Bounds panel{ui.width - width - 24,24,width,53};
            ui.panel(panel);
            ui.text(panel.x+10,panel.y+6,"GENERATION " + std::to_string(training.generation),12,cream,.65F);
            const auto running = std::to_string(training.active_cars) + " RUNNING";
            ui.text(panel.x+width-10-ui.text_width(running,11,.5F),panel.y+7,running,11,muted,.5F);

            for (std::size_t i = 0; i < visible_count; ++i) {
                const auto rank = ranks[i];
                const auto &driver = standings[rank];
                const bool selected = driver.slot == focused_slot;
                const float x = panel.x + 8 + i * (tile_width + 4);
                const float y = panel.y + 24;
                const auto background = selected ? glm::vec4{.30F,.14F,.125F,1} : glm::vec4{.14F,.17F,.21F,1};
                const auto ink = driver.eliminated && !selected ? muted : cream;
                ui.rounded_rect({x,y,tile_width,22},4,background);
                if (selected) ui.rect(x,y+4,2,14,red);
                ui.text(x+8,y+6,std::to_string(rank+1),11,ink,.35F);
                draw_driver_marker(ui,{x+27,y+11},driver);
                ui.text(x+37,y+5,driver.name,12,ink,.25F);
                const auto value = driver.lap_time ? lap_time(driver.lap_time) : decimal(driver.progress);
                ui.text(x+tile_width-7-ui.text_width(value,10,.1F),y+6,value,10,ink,.1F);
            }
            return panel;
        }

        void draw_standings(Canvas &ui, Bounds panel, std::span<const BroadcastDriver> standings,
                            std::size_t focused_slot, const TrainingTelemetry &training) {
            const float x = panel.x + 12;
            const float right = panel.x + panel.width - 12;
            const bool recap = training.recap.has_value();
            const float row_height = recap ? 23.0F : 19.0F;
            ui.panel(panel);
            ui.text(x,panel.y+12,recap ? "FINAL STANDINGS" : "GENERATION " + std::to_string(training.generation),
                    17,cream,.55F);
            if (!recap) {
                const auto running = std::to_string(training.active_cars) + " RUNNING";
                ui.text(right-ui.text_width(running,11,.45F),panel.y+17,running,11,muted,.45F);
            }
            ui.rect(x,panel.y+38,panel.width-24,1,rule);
            ui.text(x,panel.y+45,"POS",10,muted,.65F);
            ui.text(x+43,panel.y+45,"DRIVER",10,muted,.65F);
            const std::string_view metric = "DIST / LAP";
            ui.text(right-ui.text_width(metric,10,.55F),panel.y+45,metric,10,muted,.55F);

            for (std::size_t rank = 0; rank < standings.size(); ++rank) {
                const auto &driver = standings[rank];
                const bool focused = driver.slot == focused_slot;
                const bool winner = recap && driver.slot == training.recap->winner_slot;
                const float y = panel.y + 63 + rank * row_height;
                if (focused || winner) {
                    const glm::vec4 background = focused ? glm::vec4{.30F,.14F,.125F,1}
                                                         : glm::vec4{.20F,.19F,.15F,1};
                    ui.rounded_rect({panel.x+5,y,panel.width-10,row_height-1},4,background);
                    ui.rect(panel.x+5,y+3,2,row_height-7,focused ? red : cream);
                } else if (rank % 2 == 0) {
                    ui.rect(panel.x+5,y,panel.width-10,row_height-1,{.13F,.16F,.19F,.47F});
                }
                const float middle = y + (row_height-1)*.5F;
                ui.text(x,middle-6,std::to_string(rank + 1),12,focused ? cream : muted,.3F);
                draw_driver_marker(ui,{x+31,middle},driver);
                const auto ink = driver.eliminated && !focused && !winner ? muted : cream;
                ui.text(x+43,middle-6.5F,driver.name,13,ink,.4F);
                const auto value = driver.lap_time ? lap_time(driver.lap_time) : decimal(driver.progress);
                ui.text(right-ui.text_width(value,12,.3F),middle-6,value,12,ink,.3F);
            }
        }

        void draw_generation_recap(Canvas &ui, const simulation::TrackPath &path, const TrainingTelemetry &training,
                                   std::span<const BroadcastDriver> standings, std::size_t focused_slot) {
            const auto &recap = *training.recap;
            const auto winner = std::ranges::find(standings,recap.winner_slot,&BroadcastDriver::slot);
            const float x = (ui.width - 1032)*.5F;
            const float y = (ui.height - 628)*.5F;
            ui.rect(0,0,ui.width,ui.height,{.018F,.025F,.035F,.77F});
            ui.rounded_rect({x,y,1032,628},10,{.049F,.064F,.083F,1});
            ui.rect(x+20,y+20,4,18,red);
            ui.text(x+34,y+20,"APEX  /  GENERATION " + std::to_string(training.generation) + " RESULTS",15,cream,1.0F);

            const Bounds winner_panel{x+20,y+52,652,86};
            ui.rounded_rect(winner_panel,6,{.15F,.11F,.11F,1});
            ui.text(x+35,y+63,"WINNER",12,red,1.25F);
            ui.text(x+35,y+83,winner->name,34,cream,.8F);
            const auto result = winner->lap_time ? lap_time(winner->lap_time) : decimal(winner->progress);
            const std::string_view result_label = winner->lap_time ? "LAP TIME" : "DISTANCE";
            const float right = winner_panel.x + winner_panel.width - 15;
            ui.text(right-ui.text_width(result_label,11,.7F),y+67,result_label,11,muted,.7F);
            ui.text(right-ui.text_width(result,24,.4F),y+86,result,24,cream,.4F);

            const Bounds area{x+20,y+152,652,408};
            ui.rounded_rect(area,6,{.030F,.043F,.060F,1});
            ui.text(area.x+15,area.y+14,"ELIMINATION MAP",14,cream,.95F);
            const auto total = std::ranges::count_if(standings, &BroadcastDriver::eliminated);
            const auto out = std::to_string(total) + " OUT";
            ui.text(right-ui.text_width(out,12,.5F),area.y+16,out,12,red,.5F);
            ui.rect(area.x+15,area.y+40,area.width-30,1,rule);

            const Bounds labels{area.x+10,area.y+52,area.width-20,area.height-65};
            const CircuitMap map{path,{labels.x+45,labels.y+31,labels.width-90,labels.height-62}};
            map.draw(ui,path,6.0F,{.17F,.22F,.28F,1});
            map.draw(ui,path,2.0F,{.55F,.61F,.66F,1});

            struct Marker {
                const BroadcastDriver *driver;
                glm::vec2 point;
                std::optional<Bounds> label;
            };
            std::vector<Marker> markers;
            for (const auto &driver : standings) {
                if (driver.eliminated) markers.push_back({&driver,map(driver.position),{}});
            }
            for (auto &marker : markers) {
                // Numbered callouts fan out from the original positions when several cars stop together.
                for (float radius = 19.0F; radius <= 155.0F && !marker.label; radius += 14.0F) {
                    constexpr int directions = 16;
                    for (int i = 0; i < directions; ++i) {
                        const float angle = i * 2.0F * std::numbers::pi_v<float> / directions;
                        const glm::vec2 center = marker.point + radius * glm::vec2{std::cos(angle),std::sin(angle)};
                        const Bounds box{center.x-13,center.y-8,26,16};
                        if (box.x < labels.x || box.x+box.width > labels.x+labels.width ||
                            box.y < labels.y || box.y+box.height > labels.y+labels.height ||
                            std::ranges::any_of(markers, [&](const Marker &other) {
                                const Bounds dot{other.point.x-6,other.point.y-6,12,12};
                                return box.overlaps(dot,1) || (other.label && box.overlaps(*other.label,2));
                            })) continue;
                        marker.label = box;
                        break;
                    }
                }
            }
            for (const auto &marker : markers) {
                if (marker.label) {
                    const auto &box = *marker.label;
                    const bool focused = marker.driver->slot == focused_slot;
                    ui.line(marker.point.x,marker.point.y,box.x+box.width*.5F,box.y+box.height*.5F,
                            focused ? 1.6F : .85F, focused ? cream : glm::vec4{.72F,.38F,.33F,.55F});
                }
            }
            for (const auto &marker : markers) {
                ui.cross(marker.point,3.9F,2.0F,red);
                if (marker.label) {
                    const auto &box = *marker.label;
                    const bool focused = marker.driver->slot == focused_slot;
                    ui.rounded_rect(box,4,focused ? cream : glm::vec4{.24F,.125F,.12F,1});
                    const auto id = car_number(marker.driver->slot);
                    ui.text(box.x+(box.width-ui.text_width(id,11,.4F))*.5F,box.y+3,id,11,
                            focused ? panel_color : cream,.4F);
                }
            }
            const auto focused = std::ranges::find(standings,focused_slot,&BroadcastDriver::slot);
            if (focused != standings.end() && focused->eliminated) {
                const auto point = map(focused->position);
                ui.dot(point.x,point.y,7.4F,cream);
                ui.dot(point.x,point.y,5.9F,{.17F,.07F,.07F,1});
                ui.cross(point,3.6F,1.8F,red);
            }

            draw_standings(ui,{x+688,y+16,324,579},standings,focused_slot,training);
            const auto countdown = recap.held ? std::string{"RECAP HELD"}
                : "NEXT GENERATION IN " + std::to_string(static_cast<int>(std::ceil(recap.remaining_seconds))) + "S";
            ui.text(x+20,y+578,countdown,13,cream,.6F);
            ui.text(x+20,y+602,"ARROWS  INSPECT DRIVER    SPACE  HOLD / RESUME    ENTER  CONTINUE",11,muted,.6F);
        }
    }

    RaceHud::RaceHud() {
        Shader vertex{GL_VERTEX_SHADER};
        vertex.compile(R"glsl(
            #version 330 core
            layout(location = 0) in vec2 position;
            layout(location = 1) in vec4 color;
            layout(location = 2) in vec2 uv;
            uniform bool sky;
            out vec2 clip;
            out vec4 tint;
            out vec2 glyph_uv;
            void main() {
                vec2 p = position;
                if (sky) {
                    p = gl_VertexID == 0 ? vec2(-1,-1) : (gl_VertexID == 1 ? vec2(3,-1) : vec2(-1,3));
                }
                clip = p;
                tint = color;
                glyph_uv = uv;
                gl_Position = vec4(p, 0.0, 1.0);
            }
        )glsl");
        Shader fragment{GL_FRAGMENT_SHADER};
        fragment.compile(R"glsl(
            #version 330 core
            uniform bool sky;
            uniform mat4 inverse_view_projection;
            uniform vec3 eye;
            uniform vec3 sun;
            uniform bool blue_hour;
            in vec2 clip;
            in vec4 tint;
            in vec2 glyph_uv;
            uniform sampler2D font_atlas;
            out vec4 output_color;

            float cloud_hash(vec2 p) {
                vec3 q = fract(vec3(p.xyx)*.1031);
                q += dot(q,q.yzx+33.33);
                return fract((q.x+q.y)*q.z);
            }

            float cloud_noise(vec2 p) {
                vec2 cell = floor(p);
                vec2 u = fract(p);
                u = u*u*(3.0-2.0*u);
                return mix(mix(cloud_hash(cell),cloud_hash(cell+vec2(1,0)),u.x),
                           mix(cloud_hash(cell+vec2(0,1)),cloud_hash(cell+vec2(1,1)),u.x),u.y);
            }

            void main() {
                if (!sky) {
                    float coverage = glyph_uv.x >= 0.0 ? texture(font_atlas, glyph_uv).r : 1.0;
                    output_color = vec4(tint.rgb, tint.a * coverage);
                    return;
                }
                vec4 world = inverse_view_projection * vec4(clip, 1, 1);
                vec3 ray = normalize(world.xyz / world.w - eye);
                float altitude = max(ray.z, 0.0);
                float daylight = 1.0 - exp(-altitude * 4.8);
                float toward_sun = max(dot(ray, sun), 0.0);
                vec3 horizon = mix(vec3(.68,.52,.56), vec3(.99,.65,.38), pow(toward_sun,4.0));
                vec3 color = mix(horizon,vec3(.19,.33,.53),daylight);
                color += vec3(.34,.14,.035) * pow(toward_sun, 26.0);
                color = mix(color, vec3(1.65,1.34,.92), smoothstep(.99967,.99985,toward_sun));
                if (blue_hour) {
                    horizon = mix(vec3(.27,.30,.50),vec3(.86,.47,.41),pow(toward_sun,4.0));
                    color = mix(horizon,vec3(.073,.14,.31),1.0-exp(-altitude*6.4));
                }
                vec2 sky_plane = ray.xy / (.17+altitude);
                vec2 cloud = vec2(dot(sky_plane,vec2(.82,.57)),dot(sky_plane,vec2(-.57,.82))) * vec2(1.8,5.5);
                float wisps = .59*cloud_noise(cloud) + .27*cloud_noise(cloud*2.1+8.2)
                            + .14*cloud_noise(cloud*4.8-2.6);
                float cloud_cover = smoothstep(.47,.79,wisps)*smoothstep(.018,.085,altitude);
                vec3 clouds = mix(vec3(.49,.45,.62),vec3(.99,.74,.53),pow(toward_sun,3.0));
                if (blue_hour) {
                    clouds = mix(vec3(.18,.21,.36),vec3(.71,.39,.43),pow(toward_sun,3.0));
                }
                color = mix(color, clouds, cloud_cover * .64);
                float azimuth = atan(ray.y, ray.x);
                float ridge = .086 + .025 * sin(azimuth*5.0 + .9) + .021 * sin(azimuth*11.0)
                    + .009 * sin(azimuth*29.0) + .003 * sin(azimuth*67.0);
                float foothill = .041 + .019 * sin(azimuth*6.0 - .7)
                    + .008 * sin(azimuth*21.0) + .004 * sin(azimuth*48.0);
                vec3 distant = mix(vec3(.35,.37,.52),horizon,.31 + .24 * pow(toward_sun,4.0));
                distant *= .94 + .08 * sin(azimuth*17.0 + ray.z*65.0);
                vec3 near = mix(vec3(.25,.29,.39), distant, .30 + .25 * sin(azimuth*13.0-ray.z*70.0));
                if (blue_hour) {
                    distant = mix(vec3(.17,.20,.33),horizon,.23);
                    near = mix(vec3(.081,.11,.22),distant,.28);
                }
                color = mix(color, distant, 1.0-smoothstep(ridge-.002,ridge,ray.z));
                color = mix(color, near, 1.0-smoothstep(foothill-.002,foothill,ray.z));
                output_color = vec4(pow(max(color,vec3(0.0)),vec3(2.2)), 1.0);
            }
        )glsl");
        program_.link(vertex, fragment);
        sky_ = program_.uniform_location("sky");
        inverse_view_projection_ = program_.uniform_location("inverse_view_projection");
        eye_ = program_.uniform_location("eye");
        sun_ = program_.uniform_location("sun");
        blue_hour_ = program_.uniform_location("blue_hour");

        std::array<unsigned char, hud_font::width * hud_font::height> atlas{};
        auto pixel = atlas.begin();
        for (const auto run : hud_font::runs) {
            pixel = std::fill_n(pixel, (run & 15) + 1, static_cast<unsigned char>((run >> 4) * 17));
        }
        glGenTextures(1, &font_texture_);
        glBindTexture(GL_TEXTURE_2D, font_texture_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, hud_font::width, hud_font::height, 0, GL_RED,
                     GL_UNSIGNED_BYTE, atlas.data());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);

        glGenVertexArrays(1, &sky_array_);
        glGenVertexArrays(1, &array_);
        glGenBuffers(1, &buffer_);
        glBindVertexArray(array_);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, color)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, uv)));
        glEnableVertexAttribArray(2);
        glBindVertexArray(0);
    }

    RaceHud::~RaceHud() {
        glDeleteTextures(1, &font_texture_);
        glDeleteBuffers(1, &buffer_);
        glDeleteVertexArrays(1, &array_);
        glDeleteVertexArrays(1, &sky_array_);
    }

    void RaceHud::draw_sky(const glm::mat4 &inverse_view_projection, glm::vec3 eye, glm::vec3 sun,
                           bool blue_hour) const {
        glDisable(GL_DEPTH_TEST);
        program_.use();
        glUniform1i(sky_, GL_TRUE);
        glUniformMatrix4fv(inverse_view_projection_, 1, GL_FALSE, glm::value_ptr(inverse_view_projection));
        glUniform3fv(eye_, 1, glm::value_ptr(eye));
        glUniform3fv(sun_, 1, glm::value_ptr(sun));
        glUniform1i(blue_hour_, blue_hour);
        glBindVertexArray(sky_array_);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glEnable(GL_DEPTH_TEST);
    }

    static void draw_live_hud(Canvas &ui, const simulation::RaceStatus &status,
                       const simulation::TrackPath &path, const simulation::VehicleState &player,
                       std::span<const simulation::VehicleState> opponents, bool circuit_overview,
                       std::string_view focused_driver, std::span<const DriverNameplate> nameplates,
                       const TrainingTelemetry *training, std::string_view trackside_name,
                       std::span<const BroadcastDriver> standings, std::size_t focused_slot) {
        const bool training_view = training || !focused_driver.empty();
        const bool leaderboard = !standings.empty();
        const auto controls = training_view
                            ? std::string_view{"ARROWS  SELECT CAR    TAB  FOLLOW LEADER    T  TRACKSIDE    C  CIRCUIT    F  CHASE    ESC  CLOSE"}
                            : std::string_view{"T  TRACKSIDE    C  CIRCUIT    F  CHASE    P  AUTO    R  RESET    SPACE  PAUSE"};
        const Bounds brand{24, 24, 208, 65};
        const float timing_width = leaderboard ? 237.0F : circuit_overview ? 180.0F : 220.0F;
        const float timing_height = leaderboard ? 71.0F + 19.0F * standings.size() : training ? 132.0F : 94.0F;
        const Bounds timing{ui.width - timing_width - 24, 24, timing_width, timing_height};
        const Bounds speed{24, ui.height - 99, 147, 75};
        const Bounds circuit{leaderboard ? 24 : ui.width - 208, leaderboard ? 103 : ui.height - 225, 184, 159};
        const float controls_width = ui.text_width(controls, 12, .65F);
        const Bounds shortcuts{ui.width - controls_width - 48, ui.height - 52, controls_width + 24, 28};
        const float station_width = ui.text_width(trackside_name, 13, .85F) + 36;
        const Bounds station{(ui.width - station_width) * .5F, 24, station_width, 28};

        std::vector<Bounds> occupied{brand, speed, shortcuts};
        if (leaderboard && circuit_overview) {
            occupied.push_back(draw_circuit_standings(ui,standings,focused_slot,*training));
        } else {
            occupied.push_back(timing);
        }
        if (!trackside_name.empty()) occupied.push_back(station);
        if (!circuit_overview) {
            occupied.push_back(circuit);
        }
        if (status.paused || !status.started) {
            occupied.push_back({ui.width * .5F - 161, ui.height * .21F, 322, 67});
        }

        for (const auto &plate : nameplates) {
            const float x = (plate.anchor.x + 1) * ui.width * .5F;
            const float y = (1 - plate.anchor.y) * ui.height * .5F;
            if (plate.name.empty() || x < 0 || x > ui.width || y < 0 || y > ui.height) {
                continue;
            }

            const float size = plate.focused ? 14.0F : 12.0F;
            const auto split = plate.name.find_last_of(' ');
            const auto name = plate.name.substr(0, split);
            const auto number = split == std::string_view::npos ? std::string_view{} : plate.name.substr(split + 1);
            const float number_width = number.empty() ? 0.0F : ui.text_width(number, size, .35F) + 9.0F;
            const float plate_width = ui.text_width(name, size, .55F) + number_width + 23.0F;
            const float plate_height = size + 8.0F;
            const float left = std::clamp(x - plate_width * .5F, 6.0F,
                                          std::max(6.0F, ui.width - plate_width - 6.0F));

            for (const float lift : {0.0F, 25.0F, 50.0F, 75.0F}) {
                const float bottom = y - 11.0F - lift;
                const Bounds label{left, bottom - plate_height, plate_width, plate_height};
                if (label.y < 6 || label.width > ui.width - 12 ||
                    std::ranges::any_of(occupied, [&](Bounds other) { return label.overlaps(other); })) {
                    continue;
                }

                const auto accent = glm::mix(plate.livery, glm::vec3(cream), .26F);
                const glm::vec4 leader_line{cream.r, cream.g, cream.b, plate.focused ? .75F : .38F};
                ui.line(std::clamp(x, left + 5, left + plate_width - 5), bottom, x, y - 3,
                        1.0F, leader_line);
                ui.dot(x, y - 2, 1.7F, leader_line);
                ui.rounded_rect(label, 5.0F, plate.focused ? cream : panel_color);
                const float marker_x = left + 9.5F;
                const float marker_y = label.y + plate_height * .5F;
                if (plate.eliminated) {
                    const auto ink = plate.focused ? glm::vec4{.67F,.15F,.11F,1} : red;
                    ui.cross({marker_x,marker_y},3,1.9F,ink);
                } else {
                    ui.rounded_rect({marker_x - 3.5F, marker_y - 3.5F, 7, 7}, 2, {accent, 1.0F});
                }
                ui.text(left + 18, label.y + 4, name, size, plate.focused ? panel_color : cream, .55F);
                if (!number.empty()) {
                    const float divider = left + plate_width - number_width - 5;
                    ui.rect(divider, label.y + 5, 1, plate_height - 10, rule);
                    ui.text(divider + 6, label.y + 4, number, size, plate.focused ? panel_color : muted, .35F);
                }
                occupied.push_back(label);
                break;
            }
        }

        ui.panel(brand);
        ui.rounded_rect({36,36,4,19}, 2, red);
        ui.text(49,33,"APEX",27,cream,1.1F);
        const auto session = training_view ? "TRAINING" : (opponents.empty() ? "TIME TRIAL" : "RACE DAY");
        ui.text(126,41,session,12,muted,.75F);
        ui.text(37,66,"SUNSET CIRCUIT",13,cream,1.35F);

        if (!trackside_name.empty()) {
            ui.panel(station);
            ui.dot(station.x + 13, station.y + 14, 2.6F, red);
            ui.text(station.x + 23, station.y + 7, trackside_name, 13, cream, .85F);
        }

        const float x = timing.x + 14;
        const auto current_lap = status.lap_time > 0.0 ? std::optional{status.lap_time} : std::nullopt;
        if (leaderboard) {
            if (!circuit_overview) draw_standings(ui,timing,standings,focused_slot,*training);
        } else if (training) {
            ui.panel(timing);
            const float running_x = timing.x + timing.width * .57F;
            ui.text(x,36,focused_driver,24,cream,.9F);
            ui.rect(x,64,timing.width - 28,1,rule);
            ui.text(x,73,"GENERATION",11,muted,.8F);
            ui.text(running_x,73,"RUNNING",11,muted,.8F);
            ui.text(x,89,std::to_string(training->generation),21,cream);
            const auto active = std::to_string(training->active_cars) + "/" + std::to_string(opponents.size()+1);
            ui.text(running_x,89,active,21,cream);
            ui.rect(x,117,timing.width - 28,1,rule);
            ui.text(x,130,"BEST DISTANCE",11,muted,.65F);
            const auto distance = decimal(training->best_distance);
            ui.text(timing.x + timing.width - 14 - ui.text_width(distance, 16),127,distance,16,cream);
        } else {
            ui.panel(timing);
            const auto lap_label = "LAP " + std::to_string(status.completed_laps + 1);
            ui.text(x,36,focused_driver.empty() ? std::string_view{lap_label} : focused_driver,13,muted,1.0F);
            if (!training_view && !opponents.empty()) {
                const auto position = "P" + std::to_string(status.position) + "/" + std::to_string(opponents.size()+1);
                ui.text(timing.x + timing.width - 14 - ui.text_width(position, 13),36,position,13,cream);
            }
            ui.text(x,54,lap_time(current_lap),28,cream);
            ui.text(x,89,"BEST  " + lap_time(status.best_lap),13,muted,.65F);
        }

        ui.panel(speed);
        const auto driving_status = status.automatic ? "AUTOPILOT" : (status.on_track ? "ON TRACK" : "OFF TRACK");
        ui.dot(speed.x + 16, speed.y + 17, 2.7F, status.on_track ? cream : red);
        ui.text(speed.x+25,speed.y+12,driving_status,11,muted,.85F);
        ui.text(speed.x+13,speed.y+30,std::to_string(std::lround(status.speed * simulation::race_speed_to_kmh)),32,cream);
        ui.text(speed.x+91,speed.y+44,"KM/H",10,muted,.55F);
        ui.rounded_rect({speed.x+13,speed.y+64,121,3},1.5F,rule);
        const float fill = static_cast<float>(std::clamp(status.speed / simulation::race_top_speed, 0.0, 1.0));
        if (fill > 0) {
            const float fill_width = 121 * fill;
            ui.rounded_rect({speed.x+13,speed.y+64,fill_width,3},std::min(1.5F,fill_width * .5F),red);
        }

        ui.rounded_rect(shortcuts, 6, panel_color);
        ui.text(shortcuts.x+12,shortcuts.y+8,controls,12,cream,.65F);

        if (!circuit_overview) {
            const CircuitMap map{path,{circuit.x+15,circuit.y+32.5F,154,111}};
            ui.panel(circuit);
            ui.text(circuit.x+14,circuit.y+12,"CIRCUIT",12,cream,1.25F);
            ui.rect(circuit.x+14,circuit.y+30,circuit.width-28,1,rule);
            map.draw(ui,path,2.5F,{.53F,.57F,.61F,1});
            for (const auto &opponent : opponents) {
                const auto dot = map(opponent.body.position);
                ui.dot(dot.x,dot.y,1.7F,cream);
            }
            const auto dot = map(player.body.position);
            ui.dot(dot.x,dot.y,4.8F,panel_color);
            ui.dot(dot.x,dot.y,3.3F,red);
        }

        if (status.paused || !status.started) {
            const Bounds modal{ui.width * .5F - 161, ui.height * .21F, 322, 67};
            ui.panel(modal);
            const auto centered = [&](float top, std::string_view label, float size, glm::vec4 color) {
                ui.text(modal.x + (modal.width - ui.text_width(label,size,1.0F)) * .5F,
                        modal.y + top, label, size, color, 1.0F);
            };
            if (status.countdown > 0.0 && !status.paused) {
                for (int light = 0; light < 5; ++light) {
                    ui.dot(modal.x+89+light*36,modal.y+21,9,light < status.start_lights() ? red : muted);
                }
                centered(44,"HOLD LINE",14,cream);
            } else {
                centered(12,status.paused ? "PAUSED" : "STARTING GRID",24,cream);
                centered(44,status.paused ? "SPACE RESUME" : "W THROTTLE  /  P AUTOPILOT",14,muted);
            }
        }
    }

    void RaceHud::draw(int width, int height, const simulation::RaceStatus &status,
                       const simulation::TrackPath &path, const simulation::VehicleState &player,
                       std::span<const simulation::VehicleState> opponents, bool circuit_overview,
                       std::string_view focused_driver, std::span<const DriverNameplate> nameplates,
                       const TrainingTelemetry *training, std::string_view trackside_name,
                       std::span<const BroadcastDriver> standings, std::size_t focused_slot) const {
        const float scale = std::min(width / 1120.0F, height / 680.0F);
        Canvas ui{width / scale, height / scale, {}};
        if (training && training->recap) {
            draw_generation_recap(ui,path,*training,standings,focused_slot);
        } else {
            draw_live_hud(ui,status,path,player,opponents,circuit_overview,focused_driver,
                          nameplates,training,trackside_name,standings,focused_slot);
        }
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        program_.use();
        glUniform1i(sky_, GL_FALSE);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, font_texture_);
        glBindVertexArray(array_);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(ui.vertices.size() * sizeof(Vertex)),
                     ui.vertices.data(), GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(ui.vertices.size()));
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }
}
