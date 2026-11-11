#include "graphics/race_scenery.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <numbers>
#include <random>
#include <string>
#include <vector>

#include <glm/common.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "graphics/ground_mesh.hpp"
#include "simulation/race.hpp"

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 color;
            float material{};
        };

        enum class Surface { painted, glass, concrete };

        glm::vec3 track_position(const simulation::TrackSample &sample, glm::vec3 point) {
            return {sample.position.x + point.x*sample.tangent.x - point.y*sample.tangent.y,
                    sample.position.y + point.x*sample.tangent.y + point.y*sample.tangent.x, point.z};
        }

        struct Builder {
            std::vector<Vertex> vertices;

            void triangle(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 color) {
                vertices.insert(vertices.end(), {{a,color},{b,color},{c,color}});
            }

            void quad(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 color,
                      Surface surface = Surface::painted) {
                const float material = static_cast<float>(surface);
                vertices.insert(vertices.end(), {{a,color,material},{b,color,material},{c,color,material},
                                                 {a,color,material},{c,color,material},{d,color,material}});
            }

            void box(glm::vec3 lo, glm::vec3 hi, glm::vec3 color, Surface surface = Surface::painted) {
                const glm::vec3 a{lo.x, lo.y, lo.z}, b{hi.x, lo.y, lo.z};
                const glm::vec3 c{hi.x, hi.y, lo.z}, d{lo.x, hi.y, lo.z};
                const glm::vec3 e{lo.x, lo.y, hi.z}, f{hi.x, lo.y, hi.z};
                const glm::vec3 g{hi.x, hi.y, hi.z}, h{lo.x, hi.y, hi.z};
                quad(a,b,f,e,color,surface);
                quad(b,c,g,f,color,surface);
                quad(c,d,h,g,color,surface);
                quad(d,a,e,h,color,surface);
                quad(e,f,g,h,color,surface);
                quad(d,c,b,a,color,surface);
            }

            void beam(float x, float width, glm::vec2 start, glm::vec2 end, float thickness, glm::vec3 color) {
                const auto offset = glm::vec2{start.y - end.y, end.x - start.x}
                                  * (thickness * .5F / std::hypot(end.x - start.x, end.y - start.y));
                const auto a = start + offset, b = end + offset;
                const auto c = end - offset, d = start - offset;
                const auto left = [x, width](glm::vec2 p) { return glm::vec3{x-width*.5F,p.x,p.y}; };
                const auto right = [x, width](glm::vec2 p) { return glm::vec3{x+width*.5F,p.x,p.y}; };
                quad(left(a), left(b), left(c), left(d), color);
                quad(right(d), right(c), right(b), right(a), color);
                quad(left(a), right(a), right(b), left(b), color);
                quad(left(b), right(b), right(c), left(c), color);
                quad(left(c), right(c), right(d), left(d), color);
                quad(left(d), right(d), right(a), left(a), color);
            }
        };

        void add_braking_boards(Builder &mesh, const simulation::TrackSettings &track, const GroundMesh &ground,
                                double apex, float outside, glm::vec3 accent) {
            const float board_offset = static_cast<float>(track.road_width*.5 + simulation::runoff_width) + .20F;
            constexpr double metres_per_unit = simulation::race_speed_to_kmh / 3.6;
            constexpr glm::vec3 housing{.125F,.15F,.18F};
            constexpr glm::vec3 enamel{.93F,.88F,.76F};
            for (const int metres : {150,100,50}) {
                const auto sample = track.path->sample_at(apex - metres / metres_per_unit);
                const float side = -outside * board_offset;
                const auto center = track_position(sample,{0,side,0});
                if (std::abs(track.path->project({center.x,center.y}).lateral_offset) < board_offset - .10F) continue;
                const float base = ground.height_at(center.x,center.y);
                const auto first_vertex = mesh.vertices.size();
                mesh.box({-.016F,side-.014F,base-.015F},{.016F,side+.014F,base+.267F},housing);
                mesh.box({-.017F,side-.159F,base+.152F},{.018F,side+.159F,base+.339F},housing);
                const auto face = [&](float depth, float left, float bottom, float right, float top, glm::vec3 ink) {
                    mesh.quad({depth,side-left,base+bottom},{depth,side-right,base+bottom},
                              {depth,side-right,base+top},{depth,side-left,base+top},ink);
                };
                face(-.018F,-.151F,.160F,.151F,.331F,enamel);
                face(-.019F,-.151F,.311F,.151F,.331F,accent);

                constexpr std::array segments{
                    glm::vec4{.010F,.100F,.047F,.112F}, glm::vec4{.043F,.059F,.055F,.099F},
                    glm::vec4{.043F,.013F,.055F,.053F}, glm::vec4{.010F,.000F,.047F,.012F},
                    glm::vec4{.002F,.013F,.014F,.053F}, glm::vec4{.002F,.059F,.014F,.099F},
                    glm::vec4{.010F,.050F,.047F,.062F},
                };
                const auto number = std::to_string(metres);
                constexpr float step = .069F;
                float x = (step - .057F - number.size()*step)*.5F;
                for (const char digit : number) {
                    constexpr int masks[]{63,6,91,79,102,109,125,7,127,111};
                    const float shift = digit == '1' ? -.021F : 0.0F;
                    for (std::size_t i = 0; i < segments.size(); ++i) {
                        if (!(masks[digit-'0'] & (1 << i))) continue;
                        const auto &s = segments[i];
                        face(-.020F,x+shift+s.x,.182F+s.y,x+shift+s.z,.182F+s.w,housing);
                    }
                    x += step;
                }
                for (auto i = first_vertex; i < mesh.vertices.size(); ++i) {
                    auto &point = mesh.vertices[i].position;
                    point.y = side + (point.y-side)*.84F;
                    point.z = base + (point.z-base)*.78F;
                    point = track_position(sample,point);
                }
            }
        }

        void add_landscape_details(Builder &mesh, const simulation::TrackSettings &track, const GroundMesh &ground) {
            std::mt19937 random{72491};
            const auto unit = [&] { return static_cast<float>(random() >> 8) / 16777216.0F; };
            constexpr float turn = 2.0F * std::numbers::pi_v<float>;
            const auto radial = [](float angle) { return glm::vec2{std::cos(angle),std::sin(angle)}; };
            const auto point = [&](glm::vec2 p) { return glm::vec3{p,ground.height_at(p.x,p.y)}; };
            const auto paddock = track.path->sample_at(simulation::start_line_distance);
            const float barrier_edge = static_cast<float>(track.road_width * .5 + simulation::runoff_width);
            const auto open_ground = [&](glm::vec2 p, float radius, float margin = .74F) {
                const double dx = p.x - paddock.position.x;
                const double dy = p.y - paddock.position.y;
                const double along = dx*paddock.tangent.x + dy*paddock.tangent.y;
                const double across = dy*paddock.tangent.x - dx*paddock.tangent.y;
                if (along > -5.4-radius && along < 3.6+radius && std::abs(across) < 2.1+radius) return false;
                return std::abs(track.path->project({p.x,p.y}).lateral_offset) > barrier_edge + margin + radius;
            };
            const auto boulder = [&](glm::vec2 center, float radius, float height) {
                constexpr int sides = 7;
                std::array<glm::vec3, sides> base, shoulder, ridge;
                const float heading = unit() * turn;
                const auto crown = point(center) + glm::vec3{radius*.23F,radius*.09F,height};
                for (int i = 0; i < sides; ++i) {
                    const auto rim = radial(heading + turn * i / sides) * radius * std::lerp(.76F,1.0F,unit());
                    base[i] = point(center + rim) - glm::vec3{0,0,.025F};
                    shoulder[i] = point(center + rim*.84F) + glm::vec3{0,0,height*.51F};
                    ridge[i] = crown + glm::vec3{rim*.49F,(unit()-.65F)*height*.40F};
                }
                const glm::vec3 stone = glm::mix(glm::vec3{.35F,.32F,.26F},glm::vec3{.52F,.45F,.36F},unit());
                for (int i = 0; i < sides; ++i) {
                    const int next = (i+1) % sides;
                    mesh.quad(base[i],base[next],shoulder[next],shoulder[i],stone*.77F);
                    mesh.quad(shoulder[i],shoulder[next],ridge[next],ridge[i],stone*std::lerp(.86F,1.06F,unit()));
                    mesh.triangle(ridge[i],ridge[next],crown,stone*1.05F);
                }
            };
            const auto tuft = [&](glm::vec2 center, float height, bool reed) {
                const glm::vec3 green = reed ? glm::vec3{.29F,.36F,.17F} : glm::vec3{.34F,.38F,.21F};
                const glm::vec3 straw = reed ? glm::vec3{.48F,.43F,.23F} : glm::vec3{.56F,.48F,.28F};
                for (int blade = 0; blade < 10; ++blade) {
                    const auto direction = radial(unit()*turn);
                    const auto base = point(center + direction * height * .13F) - glm::vec3{0,0,.003F};
                    const glm::vec3 width{glm::vec2{-direction.y,direction.x} * height * .025F,0};
                    const glm::vec2 lean = direction * height * std::lerp(.26F,.67F,unit());
                    const float rise = height * std::lerp(.50F,1.0F,unit());
                    const glm::vec3 bend = base + glm::vec3{lean*.29F,rise*.62F};
                    const glm::vec3 tip = base + glm::vec3{lean,rise};
                    const auto color = glm::mix(green,straw,unit());
                    mesh.quad(base-width,base+width,bend+width*.61F,bend-width*.61F,color*.84F);
                    mesh.triangle(bend-width*.61F,bend+width*.61F,tip,color);
                }
            };

            // Broken rock bands follow the hills, with sparse scrub between exposed faces.
            struct Outcrop { glm::vec2 center; float angle, length; };
            constexpr Outcrop outcrops[]{
                {{-12.7F, 2.3F},1.18F,2.6F}, {{-11.8F, 7.4F},.93F,2.4F},
                {{ -6.2F,11.1F},.27F,2.8F}, {{  1.0F, 9.9F},.61F,2.0F},
                {{ 12.4F, 5.6F},1.18F,3.6F}, {{ 13.3F,-2.4F},1.45F,2.5F},
                {{ -6.6F,-7.5F},.12F,2.8F}, {{ -6.7F, 3.4F},.36F,1.9F},
                {{  6.5F, 4.6F},.38F,1.7F}, {{  2.7F,-3.7F},.32F,1.7F},
            };
            for (const auto &outcrop : outcrops) {
                const auto along = radial(outcrop.angle);
                const glm::vec2 across{-along.y,along.x};
                for (int i = 0; i < 17; ++i) {
                    const auto p = outcrop.center + along * ((unit()-.5F)*outcrop.length)
                                                   + across * ((unit()-.5F)*.72F);
                    const float radius = std::lerp(.065F,.41F,unit()*unit());
                    if (open_ground(p,radius) && ground.height_at(p.x,p.y) > -.04F) {
                        boulder(p,radius,radius*std::lerp(.39F,.82F,unit()));
                    }
                }
            }

            // Leave open grass beside the straight and scatter tufts in loose patches at the turns.
            const double length = track.path->length();
            for (const float fraction : {.14F,.20F,.27F,.38F,.43F,.52F,.59F,.66F,.74F,.81F,.90F}) {
                const auto sample = track.path->sample_at(length * fraction);
                const auto before = track.path->sample_at(length*fraction - .52).tangent;
                const auto after = track.path->sample_at(length*fraction + .52).tangent;
                const bool straight = std::abs(before.x*after.y - before.y*after.x) < .30;
                for (float side : {-1.0F,1.0F}) {
                    for (int i = 0; i < 22; ++i) {
                        const auto p = glm::vec2{track_position(sample,{(unit()-.5F)*1.95F,
                                                                      side*std::lerp(straight ? .65F : 1.04F,2.15F,unit()),0})};
                        if (open_ground(p,.035F,straight ? .16F : .74F) && ground.height_at(p.x,p.y) > -.09F) {
                            tuft(p,std::lerp(.028F,.080F,unit()),false);
                        }
                    }
                }
            }

            // Taller grass follows uneven margins beyond the mown verge and gravel traps.
            for (double distance = .17*length; distance < .96*length; distance += .18) {
                const auto sample = track.path->sample_at(distance);
                const auto before = track.path->sample_at(distance-.38).tangent;
                const auto after = track.path->sample_at(distance+.38).tangent;
                const double bend = before.x*after.y - before.y*after.x;
                for (float side : {-1.0F,1.0F}) {
                    const float density = static_cast<float>(.49 + .26*std::sin(distance*1.33+side)
                                                                  + .15*std::sin(distance*3.46));
                    if (unit() > density) continue;
                    const float margin = bend*side < -.29 ? .70F : .14F;
                    for (int patch = 0; patch < 3; ++patch) {
                        const auto p = glm::vec2{track_position(sample,{(unit()-.5F)*.31F,
                                side*(barrier_edge+margin+std::lerp(.05F,.54F,unit())),0})};
                        if (open_ground(p,.025F,margin) && point(p).z > -.04F) {
                            tuft(p,std::lerp(.026F,.062F,unit()),false);
                        }
                    }
                }
            }

            // Find the waterline first so the reeds grow in bands rather than isolated tufts.
            const glm::vec2 basin{-.75F,-.30F};
            if (point(basin).z >= GroundMesh::reservoir_level) return;
            for (int patch = 0; patch < 24; ++patch) {
                const auto direction = radial(turn * (patch + unit()) / 24.0F);
                const glm::vec2 tangent{-direction.y,direction.x};
                float edge = 0.5F;
                while (edge < 5.4F && point(basin + direction*edge).z < GroundMesh::reservoir_level) {
                    edge += .038F;
                }
                const auto shore = basin + direction*edge;
                for (int i = 0; i < 15; ++i) {
                    const auto p = shore + tangent*((unit()-.5F)*.78F) + direction*(unit()*.24F);
                    const float height = point(p).z - GroundMesh::reservoir_level;
                    if (height < -.015F || height > .085F || !open_ground(p,.045F)
                        || (p.y < -.65F && std::abs(p.x + 1.7F) < .32F)) {
                        continue;
                    }
                    if (patch % 5 == 2 && i % 4 == 0) {
                        boulder(p,std::lerp(.045F,.11F,unit()),std::lerp(.035F,.072F,unit()));
                    } else {
                        tuft(p,std::lerp(.085F,.185F,unit()),true);
                    }
                }
            }
        }
    }

    RaceScenery::RaceScenery(const simulation::TrackSettings &track, const GroundMesh &ground) {
        if (!track.path || track.road_width < 0.46) {
            return;
        }
        Builder mesh;
        const auto origin = track.path->sample_at(simulation::start_line_distance);
        const float half = static_cast<float>(track.road_width / 2.0);
        const float base = half + 0.12F;
        const glm::vec3 concrete{0.64F, 0.60F, 0.53F};
        const glm::vec3 graphite{0.13F, 0.16F, 0.19F};
        const glm::vec3 slate{0.29F, 0.36F, 0.40F};
        const glm::vec3 vermilion{0.67F, 0.20F, 0.16F};
        const glm::vec3 aluminum{.66F,.68F,.66F};
        const glm::vec3 ivory{.91F,.85F,.72F};
        // The scenery shader reserves this vertex color for warm practical lights.
        const glm::vec3 lamp{1.0F,.74F,.12F};
        const glm::vec3 blue_light{.08F,.68F,1.0F};

        // Pit apron and garage row on the outside of the main straight.
        mesh.box({-4.6F, -1.55F, 0.0F}, {2.4F, -base, 0.001F}, {0.23F, 0.24F, 0.24F});
        const float pit_line = -base - .084F;
        mesh.box({-4.56F,pit_line-.010F,.001F},{2.35F,pit_line+.010F,.004F},{.77F,.59F,.28F});
        for (int bay = 0; bay < 11; ++bay) {
            const float x = -4.50F + bay * 0.56F;
            mesh.box({x,-1.53F,0}, {x+.53F,-1.03F,.40F}, concrete, Surface::concrete);
            mesh.box({x + 0.06F, -1.04F, 0.01F}, {x + 0.47F, -1.025F, 0.27F}, graphite);
            for (int slat = 0; slat < 9; ++slat) {
                const float z = .021F + slat*.027F;
                mesh.box({x+.061F,-1.026F,z}, {x+.469F,-1.023F,z+.002F}, {.25F,.28F,.28F});
            }
            mesh.box({x+.047F,-1.018F,.009F}, {x+.056F,-1.005F,.280F}, aluminum);
            mesh.box({x+.474F,-1.018F,.009F}, {x+.483F,-1.005F,.280F}, aluminum);
            mesh.box({x+.045F,-1.014F,.272F}, {x+.485F,-.999F,.285F}, aluminum);
            mesh.box({x+.058F,-1.0F,.295F}, {x+.47F,-.979F,.304F}, {.79F,.74F,.54F});
            mesh.box({x+.079F,-.982F,.297F}, {x+.452F,-.976F,.301F}, lamp);
            mesh.box({x+.065F,-.998F,.359F}, {x+.465F,-.986F,.400F}, {.21F,.32F,.38F}, Surface::glass);
            for (float mullion : {.066F,.197F,.330F,.463F}) {
                mesh.box({x+mullion,-.985F,.357F},{x+mullion+.003F,-.980F,.403F},graphite);
            }
            mesh.box({x+.068F,-.94F,.001F}, {x+.075F,-.66F,.002F}, aluminum);
            mesh.box({x+.08F,-.77F,.001F}, {x+.44F,-.763F,.002F}, ivory);
            mesh.box({x + 0.03F, -1.05F, 0.30F}, {x + 0.50F, -1.02F, 0.35F},
                     bay % 3 == 0 ? vermilion : slate);
        }
        mesh.box({-4.55F, -1.59F, 0.40F}, {1.69F, -0.93F, 0.435F}, graphite);
        mesh.box({-4.56F, -.932F, .412F}, {1.70F, -.923F, .431F}, aluminum);
        mesh.box({-4.54F,-1.016F,.315F},{1.68F,-.905F,.327F},graphite);
        mesh.box({-4.54F,-.905F,.318F},{1.68F,-.901F,.330F},slate);
        for (int bay = 0; bay < 11; ++bay) {
            const float x = -4.46F + bay * .56F;
            for (float seam : {0.0F,.14F,.28F,.42F}) {
                mesh.box({x+seam,-1.56F,.435F},{x+seam+.003F,-.945F,.440F},slate);
            }
            mesh.box({x, -1.43F, .435F}, {x + .34F, -1.32F, .447F}, {.33F,.39F,.41F});
            mesh.box({x + .038F, -1.25F, .435F}, {x + .29F, -1.14F, .459F}, {.48F,.49F,.45F});
        }
        mesh.box({-4.53F, -1.08F, .435F}, {1.66F, -1.065F, .439F}, aluminum);

        // Set the glazed gallery back from the pit roof so the garage canopy remains distinct.
        constexpr float gallery_begin = -3.78F;
        constexpr float gallery_end = .91F;
        mesh.box({gallery_begin,-1.53F,.435F},{gallery_end,-1.15F,.483F},concrete,Surface::concrete);
        mesh.box({gallery_begin,-1.53F,.483F},{gallery_end,-1.49F,.657F},graphite);
        mesh.box({gallery_begin-.045F,-1.57F,.646F},{gallery_end+.045F,-1.105F,.677F},graphite);
        mesh.box({gallery_begin-.045F,-1.111F,.651F},{gallery_end+.045F,-1.101F,.671F},aluminum);
        for (int section = 0; section < 14; ++section) {
            const float x = gallery_begin + section * (gallery_end-gallery_begin)/14;
            const float right = x + (gallery_end-gallery_begin)/14;
            const glm::vec3 glass = section % 5 == 1 ? glm::vec3{.53F,.37F,.25F}
                                                      : glm::vec3{.14F,.24F,.32F};
            mesh.box({x+.006F,-1.163F,.484F},{right-.005F,-1.152F,.646F},glass,Surface::glass);
            mesh.box({x,-1.164F,.481F},{x+.009F,-1.144F,.648F},graphite);
        }
        for (float x : {gallery_begin,gallery_end-.016F}) {
            mesh.box({x,-1.504F,.482F},{x+.016F,-1.152F,.649F},slate);
        }
        mesh.box({gallery_begin,-1.148F,.489F},{gallery_end,-1.142F,.497F},aluminum);

        // The roof pitches up toward the track, leaving the sightline open.
        const float stand = base + 0.25F;
        for (int row = 0; row < 6; ++row) {
            const float y = stand + row * 0.11F;
            const float z = 0.05F + row * 0.066F;
            mesh.box({-3.6F, y, 0.0F}, {-0.4F, y + 0.115F, z}, concrete);
            for (int seat = 0; seat < 32; ++seat) {
                if (seat % 8 == 0) continue;
                const float x = -3.58F + seat * .10F;
                const auto color = (seat / 8 + row / 2) % 3 == 0 ? vermilion : slate;
                mesh.box({x, y + .028F, z}, {x + .085F, y + .096F, z + .018F}, color);
                mesh.box({x, y + .082F, z + .018F}, {x + .085F, y + .096F, z + .044F}, color);
            }
        }
        const glm::vec2 roof_front{stand - .13F, .83F};
        const glm::vec2 roof_back{stand + .76F, .69F};
        for (float x : {-3.60F, -2.54F, -1.48F, -.42F}) {
            mesh.beam(x, .048F, {stand + .74F, 0}, {stand + .61F, .71F}, .047F, graphite);
            mesh.beam(x, .032F, {stand + .67F, .48F}, {stand + .11F, .78F}, .032F, aluminum);
            mesh.beam(x, .038F, roof_front, roof_back, .057F, slate);
        }
        mesh.beam(-2.0F, 3.48F, roof_front, roof_back, .034F, graphite);
        mesh.beam(-2.0F, 3.49F, roof_front, roof_front + glm::vec2{.057F,-.009F}, .045F, ivory);
        for (int rib = 0; rib < 17; ++rib) {
            const float x = -3.67F + rib * .205F;
            const glm::vec2 raise{0.0F, .02F};
            mesh.beam(x, .011F, roof_front + raise, roof_back + raise, .009F, slate);
        }
        // Catch fencing between the front of the grandstand and the barrier.
        const float fence_y = base + .095F;
        constexpr float fence_bottom = .067F;
        constexpr float fence_top = .258F;
        constexpr float fence_start = -3.72F;
        constexpr float fence_end = -.32F;
        for (int post = 0; post <= 10; ++post) {
            const float x = std::lerp(fence_start, fence_end, post / 10.0F);
            mesh.beam(x, .015F, {fence_y,0}, {fence_y,fence_top}, .017F, graphite);
            mesh.beam(x, .012F, {fence_y,fence_top}, {fence_y-.042F,fence_top+.038F}, .014F, slate);
        }
        for (float z : {fence_bottom, fence_top}) {
            mesh.box({fence_start,fence_y-.004F,z-.003F},
                     {fence_end,fence_y+.004F,z+.003F}, aluminum);
        }
        const glm::vec3 wire{.43F,.46F,.46F};
        constexpr int cells = 64;
        for (int cell = 0; cell < cells; ++cell) {
            const float a = std::lerp(fence_start, fence_end, cell / static_cast<float>(cells));
            const float b = std::lerp(fence_start, fence_end, (cell+1) / static_cast<float>(cells));
            for (int row = 0; row < 4; ++row) {
                const float low = std::lerp(fence_bottom, fence_top, row / 4.0F);
                const float high = std::lerp(fence_bottom, fence_top, (row+1) / 4.0F);
                mesh.quad({a,fence_y,low-.0009F},{b,fence_y,high-.0009F},
                          {b,fence_y,high+.0009F},{a,fence_y,low+.0009F},wire);
                mesh.quad({b,fence_y,low-.0009F},{a,fence_y,high-.0009F},
                          {a,fence_y,high+.0009F},{b,fence_y,low+.0009F},wire);
            }
        }
        mesh.box({0.62F,stand,0}, {1.35F,stand+.56F,.58F}, concrete, Surface::concrete);
        mesh.box({0.58F, stand - 0.035F, 0.58F}, {1.39F, stand + 0.60F, 0.78F}, {0.12F, 0.30F, 0.35F});
        mesh.box({0.55F, stand - 0.075F, 0.78F}, {1.42F, stand + 0.63F, 0.81F}, graphite);
        for (int pane = 0; pane < 7; ++pane) {
            const float x = .60F + pane * .111F;
            const glm::vec3 glass{.23F + pane*.013F,.40F + pane*.008F,.46F + pane*.006F};
            mesh.box({x,stand-.037F,.608F}, {x+.098F,stand-.034F,.749F}, glass, Surface::glass);
            mesh.box({x-.003F,stand-.040F,.603F}, {x+.002F,stand-.036F,.757F}, graphite);
            mesh.box({x,stand-.040F,.695F}, {x+.098F,stand-.037F,.699F}, aluminum);
        }
        for (float x : {.64F,1.31F}) {
            mesh.box({x,stand-.012F,.06F}, {x+.027F,stand+.003F,.54F}, aluminum);
        }
        mesh.box({.68F,stand-.004F,.038F}, {1.29F,stand+.003F,.19F}, graphite);
        mesh.box({.70F,stand-.009F,.170F}, {1.27F,stand-.004F,.179F}, vermilion);

        // The infield service court connects the stand and control room.
        mesh.box({-3.85F, 1.34F, 0}, {2.82F, 1.68F, .001F}, {.29F,.30F,.28F});
        mesh.box({1.53F, stand, 0}, {2.82F, 1.34F, .001F}, {.26F,.28F,.27F});
        for (int bay = 0; bay < 5; ++bay) {
            const float x = 1.62F + bay * .22F;
            mesh.box({x, stand + .05F, .001F}, {x + .009F, stand + .42F, .002F}, aluminum);
        }
        mesh.box({1.62F, stand + .42F, .001F}, {2.51F, stand + .429F, .002F}, aluminum);

        // Start gantry and five light housings.
        for (float y : {-base, base}) {
            mesh.box({-0.036F, y - 0.023F, 0.0F}, {0.036F, y + 0.023F, 0.47F}, graphite);
            mesh.box({-.039F,y-.024F,.071F}, {-.035F,y+.024F,.423F}, aluminum);
            mesh.box({-.053F,y-.037F,0}, {.053F,y+.037F,.058F}, concrete);
        }
        mesh.box({-.075F,-base-.048F,.44F}, {.07F,base+.048F,.60F}, graphite);
        mesh.box({-.085F,-base-.048F,.594F}, {.07F,base+.048F,.608F}, vermilion);
        mesh.box({-.085F,-base-.048F,.445F}, {-.075F,base+.048F,.453F}, aluminum);
        // Chamfered enamel face with a forward-slanted wordmark in the icon's colors.
        const auto sign_face = [&](float depth, glm::vec3 color, std::initializer_list<glm::vec2> outline) {
            const auto front = [depth](glm::vec2 p) { return glm::vec3{depth,-p.x,p.y}; };
            for (auto edge = outline.begin() + 1; edge + 1 != outline.end(); ++edge) {
                mesh.triangle(front(*outline.begin()),front(*edge),front(*(edge+1)),color);
            }
        };
        sign_face(-.079F, slate, {{-.349F,.503F},{.330F,.503F},{.349F,.520F},{.349F,.583F},
                                 {.335F,.593F},{-.330F,.593F},{-.349F,.576F}});
        sign_face(-.080F, {.047F,.137F,.160F}, {{-.345F,.506F},{.328F,.506F},{.345F,.521F},{.345F,.580F},
                                                {.333F,.590F},{-.328F,.590F},{-.345F,.575F}});
        sign_face(-.081F, vermilion, {{-.327F,.514F},{-.295F,.514F},{-.259F,.582F},{-.291F,.582F}});
        sign_face(-.081F, ivory, {{-.288F,.514F},{-.277F,.514F},{-.241F,.582F},{-.252F,.582F}});
        sign_face(-.081F, vermilion, {{.258F,.514F},{.270F,.514F},{.306F,.582F},{.294F,.582F}});
        sign_face(-.081F, vermilion, {{.280F,.514F},{.313F,.514F},{.337F,.559F},{.337F,.582F},{.316F,.582F}});

        const auto letter_piece = [&](float start, std::initializer_list<glm::vec2> outline) {
            const auto front = [start](glm::vec2 p) {
                return glm::vec3{-.083F,-.0092F*(start+p.x+p.y*.25F-23.6F),.515F+p.y*.00665F};
            };
            for (auto edge = outline.begin() + 1; edge + 1 != outline.end(); ++edge) {
                mesh.triangle(front(*outline.begin()),front(*edge),front(*(edge+1)),ivory);
            }
        };
        // A
        letter_piece(0, {{0,0},{2.5F,0},{6.25F,10},{4.15F,10}});
        letter_piece(0, {{7.8F,0},{10.5F,0},{6.35F,10},{4.2F,10}});
        letter_piece(0, {{2.25F,2.35F},{8.15F,2.35F},{7.35F,4.25F},{3.05F,4.25F}});
        // P
        letter_piece(12.2F, {{0,0},{2.5F,0},{2.5F,10},{0,10}});
        letter_piece(12.2F, {{2.5F,7.85F},{9.7F,7.85F},{9.7F,8.75F},{8.45F,10},{2.5F,10}});
        letter_piece(12.2F, {{7.25F,5.8F},{9.7F,5.8F},{9.7F,7.85F},{7.25F,7.85F}});
        letter_piece(12.2F, {{2.5F,3.7F},{8.45F,3.7F},{9.7F,4.95F},{9.7F,5.8F},{2.5F,5.8F}});
        // E
        letter_piece(23.7F, {{0,0},{2.5F,0},{2.5F,10},{0,10}});
        letter_piece(23.7F, {{2.5F,7.85F},{8.8F,7.85F},{9.65F,10},{2.5F,10}});
        letter_piece(23.7F, {{2.5F,4.0F},{7.75F,4.0F},{8.6F,6.05F},{2.5F,6.05F}});
        letter_piece(23.7F, {{2.5F,0},{8.8F,0},{9.65F,2.15F},{2.5F,2.15F}});
        // X
        letter_piece(35.2F, {{0,0},{2.95F,0},{10.6F,10},{7.65F,10}});
        letter_piece(35.2F, {{7.65F,0},{10.6F,0},{2.95F,10},{0,10}});
        for (int light = 0; light < 5; ++light) {
            const float y = (light - 2) * 0.083F;
            // Magenta range encodes the light index for the scenery material.
            mesh.box({-0.092F, y - 0.026F, 0.455F}, {-0.08F, y + 0.026F, 0.495F},
                     {1.0F, light * 0.25F, 1.0F});
        }

        for (auto &vertex : mesh.vertices) {
            vertex.position = track_position(origin,vertex.position);
        }

        const auto place = [&](const Builder &local, const simulation::TrackSample &sample) {
            for (const auto &vertex : local.vertices) {
                mesh.vertices.push_back({track_position(sample,vertex.position),vertex.color,vertex.material});
            }
        };
        // Small marshal shelters break up the long views between the paddock and bridge.
        struct MarshalPost { double lap_fraction; float side; };
        constexpr MarshalPost marshal_posts[]{{.27,1.0F},{.63,-1.0F},{.90,-1.0F}};
        for (const auto &post : marshal_posts) {
            const auto sample = track.path->sample_at(track.path->length() * post.lap_fraction);
            const float side = post.side * 1.22F;
            const auto center = track_position(sample,{0,side,0});
            const float ground_height = ground.height_at(center.x,center.y);
            const auto offset = track.path->project({center.x,center.y}).lateral_offset;
            if (std::abs(offset) < half + simulation::runoff_width + .40F || ground_height < -.05F) continue;

            Builder shelter;
            const float front = side - .12F * post.side;
            const float back = side + .12F * post.side;
            const auto box = [&](float x0, float y0, float z0, float x1, float y1, float z1, glm::vec3 color) {
                shelter.box({x0,std::min(y0,y1),ground_height+z0},
                            {x1,std::max(y0,y1),ground_height+z1},color);
            };
            box(-.25F,side-.16F,-.07F,.25F,side+.16F,.025F,concrete);
            box(-.21F,back-.012F,.025F,.21F,back+.012F,.245F,graphite);
            for (const float x : {-.198F,.179F}) {
                box(x,side-.12F,.025F,x+.019F,side+.12F,.094F,slate);
                box(x,front-.009F,.090F,x+.019F,front+.009F,.246F,graphite);
            }
            box(-.225F,side-.14F,.239F,.225F,side+.14F,.256F,graphite);
            box(-.222F,front-.012F,.216F,.222F,front+.012F,.246F,vermilion);
            box(-.184F,front-.015F,.222F,.184F,front+.015F,.229F,ivory);
            place(shelter,sample);
        }
        const auto add_floodlight = [&](const simulation::TrackSample &sample, float along, float side,
                                        float base_height, float reach, float height) {
            const auto position = [&](float local_x, float local_y, float z) {
                return track_position(sample,{local_x,local_y,z});
            };
            const auto ground_point = position(along, side, 0.0F);
            const float foot = ground.height_at(ground_point.x, ground_point.y) + base_height;
            const float top = foot + height;
            const float head = side - std::copysign(reach, side);
            Builder fixture;
            fixture.box({along-.033F,side-.033F,foot},{along+.033F,side+.033F,foot+.049F},concrete);
            fixture.box({along-.013F,side-.013F,foot+.038F},{along+.013F,side+.013F,top},slate);
            fixture.beam(along,.022F,{side,top-.093F},{head,top},.022F,aluminum);
            const float front = std::min(head,side);
            const float back = std::max(head,side);
            fixture.box({along-.055F,front-.026F,top-.024F},{along+.055F,back+.024F,top+.007F},graphite);
            for (float x : {along-.032F, along+.032F}) {
                fixture.box({x-.021F,head-.023F,top-.032F},{x+.021F,head+.023F,top-.024F},lamp);
            }
            place(fixture,sample);
            floodlights_.push_back(position(along, head, top-.036F));
            const float inward = -std::copysign(1.0F, side);
            floodlight_directions_.emplace_back(-sample.tangent.y*inward, sample.tangent.x*inward);
        };

        for (float x : {-3.92F,-1.68F,1.12F}) {
            add_floodlight(origin,x,-.97F,.43F,.46F,.23F);
        }
        const double lap_length = track.path->length();
        const int stations = max_floodlights - 5;
        for (int station = 0; station < stations; ++station) {
            const double distance = (station+.62) * lap_length / stations;
            if (distance >= simulation::grid_distance(simulation::grid_size-1)-.6 &&
                distance <= simulation::start_line_distance + 2.45) {
                continue;
            }
            const auto sample = track.path->sample_at(distance);
            const float side = std::copysign(half + static_cast<float>(simulation::runoff_width) + .18F,
                                             station % 2 == 0 ? 1.0F : -1.0F);
            const math::Vec2 foot{sample.position.x - side*sample.tangent.y,
                                  sample.position.y + side*sample.tangent.x};
            if (std::abs(track.path->project(foot).lateral_offset) < half + simulation::runoff_width + .09) {
                continue;
            }
            add_floodlight(sample,0.0F,side,0.0F,.25F,.68F);
        }

        // A glazed footbridge over the north straight connects the hillside and infield.
        const double bridge_distance = .487 * lap_length;
        const auto crossing = track.path->sample_at(bridge_distance);
        const float bank = half + .62F;
        const auto support_clear = [&](float side) {
            const auto position = track_position(crossing,{0,side*bank,0});
            const auto nearby = track.path->project({position.x,position.y});
            return std::abs(nearby.lateral_offset) > half + simulation::runoff_width + .23;
        };
        if (support_clear(-1) && support_clear(1)) {
            footbridge_ = crossing;
            Builder bridge;
            constexpr float underside = .398F;
            constexpr float deck = .466F;
            constexpr float roof = .625F;
            const float end = bank + .19F;
            bridge.box({-.24F,-end,underside},{.24F,end,deck},graphite);
            bridge.box({-.269F,-end,roof},{.269F,end,roof+.025F},graphite);
            for (float x : {-.245F,.245F}) {
                bridge.box({x-.009F,-end,deck},{x+.009F,end,deck+.026F},aluminum);
                bridge.box({x-.012F,-end,roof-.011F},{x+.012F,end,roof+.004F},blue_light);
                for (int window = 0; window < 14; ++window) {
                    const float first = std::lerp(-end,end,window / 14.0F);
                    const float last = std::lerp(-end,end,(window+1) / 14.0F);
                    const glm::vec3 glass = window % 5 == 2 ? glm::vec3{.55F,.43F,.28F}
                                                             : glm::vec3{.15F,.25F,.34F};
                    bridge.box({x-.006F,first+.006F,deck+.023F},{x+.006F,last-.007F,roof-.009F},glass,Surface::glass);
                    bridge.box({x-.011F,first,deck},{x+.011F,first+.012F,roof},slate);
                }
            }
            for (float side : {-1.0F,1.0F}) {
                const float y = side * bank;
                const auto position = track_position(crossing,{0,y,0});
                const float foot = ground.height_at(position.x,position.y);
                bridge.box({-.16F,y-.075F,foot-.015F},{.16F,y+.075F,foot+.062F},concrete);
                bridge.box({-.10F,y-.065F,foot+.047F},{.10F,y+.065F,deck},graphite);
                const std::array rib{glm::vec2{y,foot+.045F},glm::vec2{side*(bank-.09F),.44F},
                                     glm::vec2{side*bank*.46F,.72F},glm::vec2{0,.78F}};
                for (float x : {-.29F,.29F}) {
                    for (std::size_t segment = 1; segment < rib.size(); ++segment) {
                        bridge.beam(x,.028F,rib[segment-1],rib[segment],.036F,slate);
                        const glm::vec2 edge{0,.026F};
                        bridge.beam(x,.022F,rib[segment-1]+edge,rib[segment]+edge,.009F,blue_light);
                    }
                    for (float lateral : {.24F,.48F,.72F}) {
                        bridge.beam(x,.012F,{y*lateral,deck},{y*lateral,.76F-.15F*lateral},.012F,aluminum);
                    }
                }

                const float stair_y = side * (bank + .10F);
                const float stairs_start = .22F;
                const float stairs_end = 1.18F;
                const auto landing = track_position(crossing,{side*stairs_end,stair_y,0});
                const auto nearby = track.path->project({landing.x,landing.y});
                if (std::abs(nearby.lateral_offset) > half + simulation::runoff_width + .24) {
                    const float landing_height = ground.height_at(landing.x,landing.y) + .022F;
                    const auto flight = [&](float x0, float x1, float bottom, float top, float width, glm::vec3 color) {
                        const float left = std::min(side*x0,side*x1);
                        const float right = std::max(side*x0,side*x1);
                        bridge.box({left,stair_y-width*.5F,bottom},{right,stair_y+width*.5F,top},color);
                    };
                    for (int step = 0; step < 12; ++step) {
                        const float start = step / 12.0F;
                        const float finish = (step+1) / 12.0F;
                        const float x0 = std::lerp(stairs_start,stairs_end,start);
                        const float x1 = std::lerp(stairs_start,stairs_end,finish);
                        const float height = std::lerp(deck,landing_height,finish);
                        flight(x0,x1,height-.032F,height,.19F,slate);
                        flight(x0,x0+.011F,height,height+.002F,.19F,ivory);
                    }
                    for (float rail_y : {stair_y-.097F, stair_y+.097F}) {
                        const auto rail = [&](float x, float z) { return glm::vec3{side*x,rail_y,z}; };
                        bridge.quad(rail(stairs_start,deck-.039F),rail(stairs_end,landing_height-.039F),
                                    rail(stairs_end,landing_height-.011F),rail(stairs_start,deck-.011F),graphite);
                        bridge.quad(rail(stairs_start,deck+.077F),rail(stairs_end,landing_height+.077F),
                                    rail(stairs_end,landing_height+.086F),rail(stairs_start,deck+.086F),aluminum);
                        for (float fraction : {0.0F,.33F,.66F,1.0F}) {
                            const float x = std::lerp(stairs_start,stairs_end,fraction);
                            const float z = std::lerp(deck,landing_height,fraction);
                            bridge.quad(rail(x-.005F,z),rail(x+.005F,z),rail(x+.005F,z+.081F),
                                        rail(x-.005F,z+.081F),aluminum);
                        }
                    }
                    flight(stairs_end,stairs_end+.16F,landing_height-.027F,landing_height,.22F,concrete);
                }
            }
            for (float x : {-.144F,.144F}) {
                bridge.box({x-.009F,-bank,underside-.008F},{x+.009F,bank,underside},lamp);
            }
            place(bridge,crossing);
            for (float side : {-1.0F,1.0F}) {
                const auto light = track_position(crossing,{0,side*.26F,underside-.013F});
                floodlights_.push_back(light);
                floodlight_directions_.emplace_back(crossing.tangent.y*side,-crossing.tangent.x*side);
            }
        }

        // A timber jetty follows the actual shoreline of the infield reservoir.
        constexpr float jetty_x = -1.7F;
        for (float shore = -2.8F; shore < .25F; shore += .04F) {
            if (ground.height_at(jetty_x, shore) > GroundMesh::reservoir_level) continue;
            const float end = shore + .73F;
            const auto projection = track.path->project({jetty_x, end});
            if (std::abs(projection.lateral_offset) < half + 1.2 ||
                ground.height_at(jetty_x, end) > GroundMesh::reservoir_level) break;

            const float approach = shore - .47F;
            const float deck = GroundMesh::reservoir_level + .074F;
            const glm::vec3 timber{.43F,.28F,.17F};
            const glm::vec3 frame{.23F,.20F,.17F};
            const auto trail_edge = [&](float progress, float side) {
                const float x = jetty_x - .43F * (1.0F-progress) * (1.0F-progress) + side * .125F;
                const float y = std::lerp(-3.74F, approach, progress);
                return glm::vec3{x,y,ground.height_at(x,y)+.014F};
            };
            constexpr int trail_segments = 14;
            for (int segment = 0; segment < trail_segments; ++segment) {
                const float from = static_cast<float>(segment) / trail_segments;
                const float to = static_cast<float>(segment + 1) / trail_segments;
                mesh.quad(trail_edge(from, -1), trail_edge(from, 1), trail_edge(to, 1), trail_edge(to, -1),
                          {.49F,.44F,.34F});
            }
            mesh.beam(jetty_x,.25F,{approach,ground.height_at(jetty_x, approach)+.027F},
                      {shore,deck},.022F,timber);
            for (int plank = 0; plank < 12; ++plank) {
                const float y = shore + plank * .061F;
                const float shade = plank % 3 == 0 ? .88F : 1.0F;
                mesh.box({jetty_x-.125F,y,deck-.014F},{jetty_x+.125F,y+.058F,deck},timber*shade);
            }
            for (float x : {jetty_x-.107F, jetty_x+.107F}) {
                for (float y : {shore, shore+.34F, end-.035F}) {
                    mesh.box({x-.011F,y-.011F,GroundMesh::reservoir_level-.08F},
                             {x+.011F,y+.011F,deck+.066F},frame);
                }
                mesh.box({x-.009F,shore,deck+.048F},{x+.009F,end-.035F,deck+.064F},timber);
            }
            break;
        }

        for (int slot = 0; slot < simulation::grid_size; ++slot) {
            const auto sample = track.path->sample_at(simulation::grid_distance(slot));
            const auto tangent = sample.tangent;
            const auto point = [&](float x, float y) {
                const double lateral = simulation::grid_lane(slot) + y;
                return glm::vec3{sample.position.x + x * tangent.x - lateral * tangent.y,
                                 sample.position.y + x * tangent.y + lateral * tangent.x, 0.0065F};
            };
            const glm::vec3 paint{0.86F, 0.88F, 0.83F};
            mesh.quad(point(.17F,-.087F), point(.181F,-.087F),
                      point(.181F,.087F), point(.17F,.087F), paint);
            for (float y : {-.087F, .076F}) {
                mesh.quad(point(.065F,y), point(.17F,y), point(.17F,y+.011F), point(.065F,y+.011F), paint);
            }
        }

        // Turn One and the infield hairpin share their reference stations with the road shader.
        constexpr std::array zone_fractions{.228,.632};
        const glm::vec3 board_accents[]{vermilion,{.17F,.53F,.62F}};
        for (std::size_t i = 0; i < zone_fractions.size(); ++i) {
            const double center = lap_length * zone_fractions[i];
            const auto entry = track.path->sample_at(center - .5).tangent;
            const auto exit = track.path->sample_at(center + .5).tangent;
            const float outside = std::copysign(1.0,entry.x*exit.y - entry.y*exit.x);
            braking_zones_[i] = {center,outside};
            add_braking_boards(mesh,track,ground,center,outside,board_accents[i]);
        }
        // Repeated chevrons mark the tighter turns from the approach side of the barrier.
        struct Corner { double distance; double bend; };
        std::vector<Corner> bends;
        for (double distance = 0; distance < lap_length; distance += 0.16) {
            const auto entry = track.path->sample_at(distance - .42).tangent;
            const auto exit = track.path->sample_at(distance + .42).tangent;
            const double bend = std::atan2(entry.x * exit.y - entry.y * exit.x,
                                           entry.x * exit.x + entry.y * exit.y);
            if (std::abs(bend) > .52) {
                bends.push_back({distance, bend});
            }
        }
        std::ranges::sort(bends, {}, [](const Corner &corner) { return -std::abs(corner.bend); });
        std::vector<double> marked_corners;
        for (const auto &corner : bends) {
            if (std::ranges::any_of(marked_corners, [&](double other) {
                    return std::abs(std::remainder(corner.distance - other, lap_length)) < 2.1;
                })) {
                continue;
            }
            marked_corners.push_back(corner.distance);
            const float turn = std::copysign(1.0F, static_cast<float>(corner.bend));
            if (std::abs(corner.bend) > .64) {
                const auto barrier_face = [&](double distance, float height) {
                    const auto sample = track.path->sample_at(distance);
                    const double offset = (half + simulation::runoff_width - .008) * turn;
                    return glm::vec3{sample.position.x + sample.tangent.y*offset,
                                     sample.position.y - sample.tangent.x*offset, height};
                };
                for (int strip = 0; strip < 14; ++strip) {
                    const double start = corner.distance - .7 + strip * .105;
                    const double end = start + .103;
                    mesh.quad(barrier_face(start,.013F), barrier_face(end,.013F), barrier_face(end,.052F),
                              barrier_face(start,.052F), (strip / 2) % 2 == 0 ? vermilion : ivory);
                }
            }
            // A tapered gravel apron sits just beyond the barrier at the sharpest bends.
            if (std::abs(corner.bend) > .60) {
                constexpr int segments = 16;
                const float barrier_edge = half + static_cast<float>(simulation::runoff_width) + .023F;
                const auto apron_edge = [&](int station) {
                    const float fraction = static_cast<float>(station) / segments;
                    const auto sample = track.path->sample_at(corner.distance + (fraction - .48F) * 3.1F);
                    const auto point = [&](float offset) {
                        return glm::vec3{
                            sample.position.x + sample.tangent.y * offset * turn,
                            sample.position.y - sample.tangent.x * offset * turn,
                            .002F
                        };
                    };
                    const float width = .025F + .66F * std::sin(fraction * std::numbers::pi_v<float>);
                    return std::array{point(barrier_edge), point(barrier_edge + width)};
                };
                std::array<std::array<glm::vec3, 2>, segments + 1> apron;
                bool clear = true;
                for (std::size_t station = 0; station < apron.size(); ++station) {
                    apron[station] = apron_edge(static_cast<int>(station));
                    const auto outer = apron[station][1];
                    clear &= std::abs(track.path->project({outer.x, outer.y}).lateral_offset) >= barrier_edge;
                }
                if (clear) {
                    for (std::size_t station = 1; station < apron.size(); ++station) {
                        const auto &previous = apron[station - 1];
                        const auto &current = apron[station];
                        mesh.quad(previous[0], current[0], current[1], previous[1], {.52F,.47F,.38F});
                    }
                }
            }
            for (const double advance : {0.90, 0.48, 0.10}) {
                const auto sample = track.path->sample_at(corner.distance - advance);
                const auto approach = track.path->sample_at(corner.distance - advance - .85).tangent;
                const double offset = -(half + simulation::runoff_width + .092) * turn;
                const auto point = [&](float depth, float lateral, float z) {
                    return glm::vec3{
                        sample.position.x - sample.tangent.y * offset + approach.x * depth - approach.y * lateral,
                        sample.position.y + sample.tangent.x * offset + approach.y * depth + approach.x * lateral,
                        z
                    };
                };
                constexpr float edge = .093F;
                mesh.quad(point(.001F,-.008F,0), point(.001F,.008F,0),
                          point(.001F,.008F,.137F), point(.001F,-.008F,.137F), aluminum);
                mesh.quad(point(0,-edge,.090F), point(0,edge,.090F),
                          point(0,edge,.181F), point(0,-edge,.181F), graphite);
                const auto chevron = [&](float lateral, float z) {
                    return point(-.0015F, lateral * turn, z);
                };
                for (float shift : {-.032F, .032F}) {
                    const glm::vec3 amber{1.0F,.52F,.23F};
                    const auto tip = chevron(shift+.026F,.135F);
                    const auto inner = chevron(shift+.004F,.135F);
                    mesh.quad(chevron(shift-.024F,.166F), chevron(shift-.002F,.166F), tip, inner, amber);
                    mesh.quad(inner, tip, chevron(shift-.002F,.104F), chevron(shift-.024F,.104F), amber);
                }
            }
        }

        add_landscape_details(mesh, track, ground);
        count_ = static_cast<GLsizei>(mesh.vertices.size());
        glGenVertexArrays(1, &array_);
        glGenBuffers(1, &buffer_);
        glBindVertexArray(array_);
        glBindBuffer(GL_ARRAY_BUFFER, buffer_);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(mesh.vertices.size() * sizeof(Vertex)),
                     mesh.vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, color)));
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, material)));
        glEnableVertexAttribArray(5);
        glBindVertexArray(0);
    }

    RaceScenery::~RaceScenery() {
        glDeleteBuffers(1, &buffer_);
        glDeleteVertexArrays(1, &array_);
    }

    void RaceScenery::draw() const {
        if (count_ > 0) {
            glBindVertexArray(array_);
            glDrawArrays(GL_TRIANGLES, 0, count_);
        }
    }
}
