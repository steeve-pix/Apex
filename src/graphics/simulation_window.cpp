#include "graphics/simulation_window.hpp"

#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <string>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cmath>
#include <numbers>
#include <vector>
#include <limits>
#include <optional>
#include <random>
#include <utility>

#include "graphics/tree_mesh.hpp"
#include "graphics/barrier_mesh.hpp"
#include "graphics/camera.hpp"
#include "graphics/car_mesh.hpp"
#include "platform/glfw_session.hpp"
#include "graphics/sensor_lines.hpp"
#include "graphics/shader.hpp"
#include "graphics/shader_program.hpp"
#include "graphics/track_mesh.hpp"
#include "graphics/ground_mesh.hpp"
#include "graphics/shadow_map.hpp"
#include "graphics/race_scenery.hpp"
#include "graphics/race_hud.hpp"
#include "graphics/scene_compositor.hpp"

namespace apex::graphics {
    namespace {
        static_assert(RaceScenery::max_floodlights == 24);
        static_assert(RaceScenery::braking_zone_count == 2);
        constexpr char wheel_shader_source[] = R"glsl(
            #version 330 core

            layout(location = 4) in vec4 wheel_pivot;
            uniform vec3 wheel_angles; // Rear roll, front roll, and steering.
            uniform float mesh_scale;

            mat3 wheel_rotation() {
                if (wheel_pivot.w < 1.5) return mat3(1.0);
                bool front = wheel_pivot.w > 2.5;
                float angle = front ? wheel_angles.y : wheel_angles.x;
                float c = cos(angle);
                float s = sin(angle);
                mat3 roll = mat3(c,0,-s, 0,1,0, s,0,c);
                if (!front) return roll;

                c = cos(wheel_angles.z);
                s = sin(wheel_angles.z);
                mat3 steer = mat3(c,s,0, -s,c,0, 0,0,1);
                return steer * roll;
            }
        )glsl";

        constexpr char shadow_vertex_shader_source[] = R"glsl(
            layout(location = 0) in vec3 position;

            uniform vec2 offset;
            uniform float heading;
            uniform float elevation;
            uniform mat4 light_view_projection;

            void main() {
                float c = cos(heading);
                float s = sin(heading);
                vec3 local = (wheel_pivot.xyz + wheel_rotation() * (position - wheel_pivot.xyz)) * mesh_scale;

                vec2 rotated_position = vec2(
                    c * local.x - s * local.y,
                    s * local.x + c * local.y
                );

                vec3 world_position = vec3(
                    rotated_position + offset,
                    local.z + elevation
                );

                gl_Position =
                    light_view_projection * vec4(world_position, 1.0);
            }
        )glsl";

        constexpr char shadow_fragment_shader_source[] = R"glsl(
            #version 330 core

            void main() {
            }
        )glsl";

        constexpr char vertex_shader_source[] = R"glsl(
            layout(location = 0) in vec3 position;
            layout(location = 1) in vec3 vertex_color;
            layout(location = 2) in vec3 road_coordinates;
            layout(location = 3) in vec3 vertex_normal;
            layout(location = 5) in float material_parameter;

            uniform vec2 offset;
            uniform float heading;
            uniform float elevation;
            uniform mat4 view_projection;

            out vec3 surface_position;
            out vec3 local_position;
            out vec3 surface_color;
            out vec3 surface_normal;
            out vec3 surface_road_coordinates;
            out float surface_material;

            void main() {
                float c = cos(heading);
                float s = sin(heading);
                mat3 wheel = wheel_rotation();
                vec3 local = (wheel_pivot.xyz + wheel * (position - wheel_pivot.xyz)) * mesh_scale;
                vec3 normal = wheel * vertex_normal;

                vec2 rotated_position = vec2(
                    c * local.x - s * local.y,
                    s * local.x + c * local.y
                );

                vec3 world_position = vec3(
                    rotated_position + offset,
                    local.z + elevation
                );

                surface_position = world_position;
                local_position = position;
                surface_color = vertex_color;
                surface_normal = vec3(c * normal.x - s * normal.y,
                                      s * normal.x + c * normal.y, normal.z);
                surface_road_coordinates = road_coordinates;
                surface_material = material_parameter;

                gl_Position = view_projection * vec4(world_position, 1.0);
            }
        )glsl";

        constexpr char fragment_shader_source[] = R"glsl(
            #version 330 core

            in vec3 surface_position;
            in vec3 local_position;
            in float surface_material;
            in vec3 surface_road_coordinates;

            uniform vec3 color;
            uniform bool curb_pattern;
            uniform bool road_markings_enabled;
            uniform float road_half_width;
            uniform float curb_segments;

            uniform bool lighting_enabled;
            uniform vec3 direction_to_light;
            uniform int floodlight_count;
            uniform vec3 floodlights[24];
            uniform vec2 floodlight_directions[24];

            uniform float track_length;
            uniform float start_line;
            uniform vec2 braking_zones[2]; // Distance along the lap and outside edge sign.
            uniform bool terrain;
            uniform bool water;
            uniform float scene_time;
            uniform bool race_scene;
            uniform vec3 camera_position;
            uniform vec3 livery;
            uniform int car_number;
            uniform bool car_material;
            uniform bool scenery_material;
            uniform bool foliage_material;
            uniform int start_lights;

            out vec4 fragment_color;

            in vec3 surface_color;
            in vec3 surface_normal;
            uniform bool use_vertex_color;

            uniform sampler2DShadow shadow_depth;
            uniform mat4 light_view_projection;

            float shadow_amount(vec3 normal, vec3 light) {
                float receiver_offset = car_material ? .0065 : .003;
                vec4 light_position =
                    light_view_projection * vec4(surface_position + normal * receiver_offset, 1.0);

                vec3 projected = light_position.xyz / light_position.w;
                projected = projected * 0.5 + 0.5;

                if (any(lessThan(projected, vec3(0.0))) ||
                    any(greaterThan(projected, vec3(1.0)))) {
                    return 0.0;
                }

                float bias = max(
                    0.00013 * (1.0 - max(dot(normal, light), 0.0)),
                    0.000055
                );

                vec2 texel_size =
                    1.0 / vec2(textureSize(shadow_depth, 0));

                float shadow = 0.0;

                for (int y = -1; y <= 1; ++y) {
                    for (int x = -1; x <= 1; ++x) {
                        vec2 sample_position =
                            projected.xy + vec2(x, y) * texel_size;

                        float weight = float((2 - abs(x)) * (2 - abs(y)));
                        shadow += weight * (1.0 - texture(shadow_depth, vec3(sample_position, projected.z - bias)));
                    }
                }

                vec2 margin = min(projected.xy, 1.0 - projected.xy);
                float strength = car_material ? .78 : 1.0;
                return strength * shadow / 16.0 * smoothstep(0.0,.09,min(margin.x,margin.y));
            }

            float hash(vec2 p) {
                vec3 q = fract(vec3(p.xyx) * .1031);
                q += dot(q, q.yzx + 33.33);
                return fract((q.x + q.y) * q.z);
            }

            float noise(vec2 p) {
                vec2 cell = floor(p);
                vec2 f = fract(p);
                f = f * f * (3.0 - 2.0 * f);
                return mix(mix(hash(cell), hash(cell + vec2(1,0)), f.x),
                           mix(hash(cell + vec2(0,1)), hash(cell + vec2(1,1)), f.x), f.y);
            }

            float checker_axis(float cell, float footprint) {
                float phase = sin(cell*3.14159265);
                float edge = clamp(footprint*1.57,.015,.8);
                return mix(smoothstep(-edge,edge,phase),.5,smoothstep(.50,1.1,footprint));
            }

            float number_digit(vec2 p, int digit, float smoothing) {
                const int masks[10] = int[10](63,6,91,79,102,109,125,7,127,111);
                const vec2 starts[7] = vec2[7](vec2(.22,.88),vec2(.80,.84),vec2(.80,.46),vec2(.22,.12),
                                               vec2(.20,.17),vec2(.20,.53),vec2(.22,.50));
                const vec2 ends[7] = vec2[7](vec2(.78,.88),vec2(.80,.54),vec2(.80,.17),vec2(.78,.12),
                                             vec2(.20,.46),vec2(.20,.84),vec2(.78,.50));
                float distance = 2.0;
                for (int i = 0; i < 7; ++i) {
                    if ((masks[digit] & (1 << i)) == 0) continue;
                    vec2 edge = ends[i] - starts[i];
                    vec2 delta = p - starts[i];
                    distance = min(distance, length(delta - edge*clamp(dot(delta,edge)/dot(edge,edge),0.0,1.0)));
                }
                return 1.0 - smoothstep(.055,.085+smoothing,distance);
            }

            void main() {
                vec3 illumination = vec3(1.0);
                float visibility = 1.0;
                float lamp_specular = 0.0;
                vec3 normal = dot(surface_normal, surface_normal) > .01
                    ? normalize(surface_normal)
                    : normalize(cross(dFdx(surface_position), dFdy(surface_position)));
                vec3 sight = normalize(camera_position - surface_position);
                float foliage_detail = 0.0;
                if (foliage_material && surface_color.g > surface_color.r) {
                    vec2 leaves = local_position.xy + vec2(.63,.82)*local_position.z;
                    float detail = 1.0 - smoothstep(.003,.023,length(fwidth(leaves)));
                    float spray = noise(leaves*83.0 + .32*noise(leaves*31.0));
                    float leaf_cells = noise(leaves*225.0);
                    foliage_detail = (spray*.69 + leaf_cells*.31 - .5) * detail;
                    float edge = abs(dot(normal,sight));
                    if (edge < .19 && detail > .55 && spray*.6+leaf_cells*.4 < .23) discard;
                    vec3 lift = vec3(spray-.5,noise(leaves*168.0)-.5,leaf_cells-.5);
                    normal = normalize(normal + lift*detail*.34);
                }

                if (water) {
                    vec2 p = surface_position.xy;
                    float depth = max(surface_material,0.0);
                    vec2 wind = vec2(.82,.57);
                    vec2 across = vec2(-wind.y,wind.x);
                    float gust = noise(p*.61 - scene_time*.06*wind);
                    float waves = cos(dot(p,wind)*22.0 - scene_time*2.1 + gust*3.0);
                    float cross_waves = cos(dot(p,across)*34.0 + scene_time*2.9 + waves*.85);
                    float capillary = cos(dot(p,vec2(41.0,59.0)) - scene_time*4.1 + gust*5.0);
                    float fine = 1.0-smoothstep(.013,.047,length(fwidth(p)));
                    vec2 slope = wind * (waves*(.028+.05*gust) + .028*capillary*fine)
                               + across * cross_waves * (.018 + .043*gust);
                    vec3 ripple = normalize(vec3(slope,1.0));
                    vec3 reflected = reflect(-sight, ripple);
                    float fresnel = .025 + .975 * pow(1.0 - clamp(dot(ripple,sight),0.0,1.0),5.0);
                    float sunset = pow(max(dot(reflected,direction_to_light),0.0),9.0);
                    vec3 sky_reflection = mix(vec3(.39,.46,.58),vec3(.19,.33,.49),smoothstep(.08,.94,reflected.z));
                    sky_reflection += vec3(.45,.21,.09)*sunset;
                    float deep = 1.0-exp(-depth*9.0);
                    vec3 shallow = mix(vec3(.39,.42,.29),vec3(.23,.39,.34),noise(p*3.4));
                    vec3 deep_water = mix(vec3(.057,.22,.27),vec3(.095,.31,.35),gust);
                    vec3 water_color = mix(mix(shallow,deep_water,deep),sky_reflection,.17+.80*fresnel);
                    float glint = pow(max(dot(reflected,direction_to_light),0.0),98.0);
                    if (floodlight_count > 0) {
                        water_color *= vec3(.78,.90,1.03);
                        glint *= .38;
                        for (int i = 0; i < floodlight_count; ++i) {
                            vec3 lamp = floodlights[i] - surface_position;
                            float distance = dot(lamp.xy,lamp.xy);
                            if (distance < 11.0) {
                                float reflection = pow(max(dot(reflected,normalize(lamp)),0.0),48.0);
                                water_color += reflection * (1.0-smoothstep(.3,11.0,distance)) * vec3(.56,.35,.15);
                            }
                        }
                    }
                    float waterline = smoothstep(.002,.025,depth);
                    vec3 water_light = pow(water_color + glint*vec3(.81,.49,.26),vec3(2.2));
                    fragment_color = vec4(water_light,mix(.22,.96,waterline) + (1.0-waterline)*fresnel*.25);
                    return;
                }

                if (lighting_enabled) {
                    vec3 light = normalize(direction_to_light);

                    float diffuse = max(dot(normal, light), 0.0);

                    visibility = 1.0 - shadow_amount(normal, light);
                    float sky = normal.z * .5 + .5;
                    vec3 bounce = mix(vec3(.28,.24,.23), vec3(.40,.49,.64), sky);
                    illumination = bounce + vec3(2.55,1.60,.90) * diffuse * visibility;
                    if (floodlight_count > 0) {
                        illumination = mix(vec3(.21,.24,.31),vec3(.38,.46,.62),sky)
                                     + vec3(.64,.46,.35) * diffuse * visibility;
                        vec3 local_light = vec3(0.0);
                        for (int i = 0; i < floodlight_count; ++i) {
                            vec3 lamp = floodlights[i] - surface_position;
                            float distance = dot(lamp.xy,lamp.xy);
                            if (distance < 14.0 && lamp.z > -.05) {
                                vec2 aim = floodlight_directions[i];
                                vec2 projected = lamp.xy + aim*.46;
                                vec2 axes = vec2(dot(projected,vec2(-aim.y,aim.x)),dot(projected,aim));
                                float lane_distance = dot(axes*axes,vec2(.095,.76));
                                float beam = 1.0 - smoothstep(.06,1.05,lane_distance);
                                float spill = .17 * (1.0 - smoothstep(.08,1.4,distance));
                                float power = mix(.52,1.0,smoothstep(.40,.66,floodlights[i].z));
                                float pool = (beam*beam + spill*spill) * power;
                                vec3 incident = normalize(lamp);
                                float facing = .31 + .69*max(dot(normal,incident),0.0);
                                local_light += pool*facing * vec3(2.45,1.95,1.51);
                                if (car_material) {
                                    vec3 halfway = normalize(incident + sight);
                                    lamp_specular += pool*pow(max(dot(normal,halfway),0.0),34.0);
                                }
                            }
                        }
                        illumination += local_light;
                    }
                }

                vec3 base_color = color;

                if (use_vertex_color) {
                    base_color = surface_color * color;
                }
                float lacquer = car_material ? smoothstep(.16,.43,surface_color.r - surface_color.g) : 0.0;
                base_color = mix(base_color, livery, lacquer);
                bool rear_number = lacquer > .8 && abs(local_position.y) > .0465
                                   && abs(local_position.y) < .050 && local_position.x < -.127;
                bool nose_number = surface_color.g > .80 && local_position.x > .067
                                   && local_position.x < .112 && local_position.z > .022;
                if (car_material && car_number > 0 && (rear_number || nose_number)) {
                    vec2 number_uv = nose_number
                        ? vec2(.5 + local_position.y/.0085, .5 - (local_position.x-.089)/.021)
                        : vec2(.5 - sign(local_position.y)*(local_position.x+.145)/.020,
                               .5 + (local_position.z-.0565)/.013);
                    float digit_column = number_uv.x * 2.0;
                    int digit = digit_column < 1.0 ? car_number / 10 : car_number % 10;
                    float footprint = length(fwidth(number_uv));
                    float smoothing = min(.11,footprint);
                    float fade = 1.0 - smoothstep(.13,.32,footprint);
                    if (number_uv.x >= 0.0 && number_uv.x <= 1.0 && number_uv.y >= 0.0 && number_uv.y <= 1.0) {
                        float ink = fade * number_digit(vec2(fract(digit_column),number_uv.y),digit,smoothing);
                        vec3 number_color = nose_number || dot(livery,vec3(.213,.715,.072)) > .58
                                          ? vec3(.052,.062,.074) : vec3(.94,.87,.73);
                        base_color = mix(base_color, number_color, ink);
                    }
                }
                if (terrain) {
                    vec2 p = surface_position.xy;
                    vec2 contour = p + vec2(noise(p*.57),noise(p*.52 + 21.0)) * 1.7;
                    float meadow = noise(contour*.29) * .57 + noise(p*1.7) * .30 + noise(p*6.0) * .13;
                    float height = max(surface_position.z, 0.0);
                    float dry_grass = smoothstep(.36,.79,meadow + min(height*.060,.19));
                    float sward = noise(contour*1.15) * .55 + noise(p*11.0) * .45;
                    vec3 meadow_green = mix(vec3(.15,.24,.125),vec3(.35,.42,.23),sward);
                    base_color = mix(meadow_green,vec3(.53,.443,.283),dry_grass);
                    float slope = 1.0 - normal.z;
                    float stone = smoothstep(.078,.22,slope + .17*(noise(contour*1.7)-.5))
                                * smoothstep(.22,.76,height);
                    vec2 bedding = vec2(dot(p,vec2(.84,.37)),height*5.7 + dot(p,vec2(.29,-.19)));
                    float strata = noise(bedding + vec2(0,noise(p*1.2)*.65));
                    vec3 shale = mix(vec3(.28,.25,.22),vec3(.64,.55,.43),strata*.73+noise(p*14.0)*.27);
                    base_color = mix(base_color, shale, stone);
                    float shore = 1.0 - smoothstep(-.17,-.036,surface_position.z + .035*(noise(p*6.0)-.5));
                    vec3 shore_gravel = mix(vec3(.35,.29,.20),vec3(.64,.55,.39),noise(p*35.0));
                    shore_gravel = mix(vec3(.11,.16,.115),shore_gravel,smoothstep(-.32,-.125,surface_position.z));
                    base_color = mix(base_color,shore_gravel,shore);
                    float footprint = length(fwidth(p));
                    float blades = noise(vec2(dot(p,vec2(.94,.32))*142.0,dot(p,vec2(-.32,.94))*63.0));
                    float detail = (blades-.5)*(1.0-smoothstep(.004,.023,footprint));
                    float clumps = (noise(p*36.0)-.5)*(1.0-smoothstep(.016,.058,footprint));
                    base_color *= 1.0 + detail*.30 + clumps*.29;
                }

                if (curb_pattern) {
                    float edge_distance = abs(surface_road_coordinates.y);
                    float bend = surface_road_coordinates.z;
                    bool outside = bend * surface_road_coordinates.y > 0.0;

                    if (edge_distance <= road_half_width
                        || (race_scene && abs(bend) < (outside ? .30 : .13))) {
                        discard;
                    }

                    float segment = surface_road_coordinates.x * curb_segments / track_length;
                    float stripe = mix(mod(floor(segment), 2.0), .5, smoothstep(.18,.82,fwidth(segment)));

                    base_color = mix(vec3(.66,.091,.062), vec3(.83,.82,.74), stripe);
                    float bevel = smoothstep(0.0,.012,edge_distance-road_half_width);
                    base_color *= mix(.78,1.06,bevel) * mix(.90,1.08,noise(surface_position.xy*75.0));
                }
                if (road_markings_enabled) {
                    float distance_from_center = abs(surface_road_coordinates.y);
                    float racing_line = race_scene ? -.10 * clamp(surface_road_coordinates.z * 1.7,-1.0,1.0) : 0.0;
                    vec2 p = surface_position.xy;
                    float grain = noise(p * 430.0);
                    float detail = 1.0 - smoothstep(.001,.008,length(fwidth(p)));
                    float aggregate = noise(p*48.0);
                    float shoulder = smoothstep(road_half_width - .024, road_half_width, distance_from_center);
                    vec3 runoff = mix(vec3(.26,.31,.32),vec3(.34,.39,.38),aggregate);
                    float braking_rubber = 0.0;
                    if (race_scene) {
                        float bend = surface_road_coordinates.z;
                        float outside = step(0.0,bend*surface_road_coordinates.y);
                        float painted = smoothstep(.39,.65,abs(bend)) * outside
                                      * smoothstep(road_half_width+.045,road_half_width+.055,distance_from_center);
                        vec3 runoff_paint = mix(vec3(.16,.33,.34),vec3(.26,.45,.43),aggregate);
                        runoff = mix(runoff,runoff_paint,painted*.88);
                        for (int zone = 0; zone < 2; ++zone) {
                            float side = braking_zones[zone].y;
                            if (side == 0.0) continue;
                            float along = mod(surface_road_coordinates.x - braking_zones[zone].x + .5*track_length,
                                              track_length) - .5*track_length;
                            float influence = smoothstep(-6.7,-4.8,along) * (1.0-smoothstep(.95,2.1,along));
                            float line = side * (.132 - .257*smoothstep(-2.5,.05,along)
                                               + .255*smoothstep(.14,2.35,along));
                            racing_line = mix(racing_line,line,influence);
                            float across = surface_road_coordinates.y - line;
                            float center_profile = across/.11;
                            float tire_profile = (abs(across)-.059)/.021;
                            float groove = exp(-center_profile*center_profile);
                            float tread = exp(-tire_profile*tire_profile);
                            float wear = noise(vec2(along*2.1,across*31.0));
                            braking_rubber += influence * (.14*groove + .11*tread) * mix(.61,1.12,wear);

                            float edge = distance_from_center - road_half_width;
                            float paint = smoothstep(-2.8,-2.3,along) * (1.0-smoothstep(1.45,1.96,along))
                                        * step(0.0,side*surface_road_coordinates.y)
                                        * smoothstep(.048,.056,edge);
                            float hatch = checker_axis((along+1.85*edge) / (zone == 0 ? .23 : .39),
                                                       fwidth(along) * 4.0);
                            float trim = smoothstep(.112,.124,edge);
                            vec3 zone_color = zone == 0 ? vec3(.56,.14,.11) : vec3(.09,.37,.47);
                            vec3 stripe = zone == 0 ? vec3(.77,.72,.59) : vec3(.50,.73,.75);
                            runoff = mix(runoff,mix(zone_color,stripe,hatch*trim*.82)*mix(.85,1.11,aggregate),paint);
                        }
                    }
                    float tire_line = (abs(surface_road_coordinates.y - racing_line) - .065) * 31.0;
                    float rubber = exp(-tire_line * tire_line);
                    base_color *= .94 + aggregate*.12 - .15*rubber - braking_rubber + .14*(grain-.5)*detail;
                    float seam = abs(distance_from_center - road_half_width - .106);
                    runoff *= mix(.73, 1.0, smoothstep(.001,.009 + fwidth(seam),seam));
                    base_color = mix(base_color, runoff, shoulder);

                    float line_width = road_half_width * 0.04;
                    float line_inset = road_half_width * 0.03;

                    float outer_edge = road_half_width - line_inset;
                    float inner_edge = outer_edge - line_width;

                    float smoothing = max(fwidth(distance_from_center),0.00001);

                    float edge_line =
                        smoothstep(inner_edge - smoothing,inner_edge + smoothing,distance_from_center)* (1.0 - smoothstep(outer_edge - smoothing,outer_edge + smoothing,distance_from_center));

                    base_color = mix(base_color, vec3(0.92), edge_line);
                    const float column_count = 10.0;

                    float cell_size =
                        (road_half_width * 2.0) / column_count;

                    float distance_along_track = surface_road_coordinates.x - start_line;

                    if (distance_along_track > track_length * 0.5) {
                        distance_along_track -= track_length;
                    }

                    float along_edge = max(fwidth(surface_road_coordinates.x)*.5,.00001);
                    float across_edge = max(fwidth(surface_road_coordinates.y)*.5,.00001);
                    float start_paint = (1.0 - smoothstep(cell_size-along_edge,cell_size+along_edge,
                                                         abs(distance_along_track)))
                                      * (1.0 - smoothstep(road_half_width-across_edge,road_half_width+across_edge,
                                                           distance_from_center));
                    if (start_paint > 0.0) {
                        float column = checker_axis((surface_road_coordinates.y+road_half_width)/cell_size,
                                                    2.0*across_edge/cell_size);
                        float row = checker_axis((distance_along_track+cell_size)/cell_size,
                                                 2.0*along_edge/cell_size);
                        float checker = column + row - 2.0*column*row;
                        base_color = mix(base_color,mix(vec3(.06),vec3(.94),checker),start_paint);
                    }
                }
                vec3 lit = base_color * sqrt(illumination);
                if (foliage_material) {
                    float transmission = pow(max(dot(-sight,direction_to_light),0.0),3.0)
                                       * clamp(surface_color.g - surface_color.r,0.0,.23);
                    lit = lit * (1.0 + foliage_detail * .70) + transmission * vec3(.40,.31,.09);
                }
                if (road_markings_enabled) {
                    vec3 road_halfway = normalize(direction_to_light + sight);
                    float sheen = pow(max(dot(normal,road_halfway),0.0),15.0) * visibility;
                    lit += vec3(.15,.115,.081) * sheen * (floodlight_count > 0 ? .20 : 1.0);
                }
                if (scenery_material) {
                    vec2 p = surface_position.xy;
                    if (surface_material > .5 && surface_material < 1.5) {
                        vec3 reflection = reflect(-sight,normal);
                        float sky = smoothstep(-.10,.48,reflection.z);
                        float horizon = exp(-abs(reflection.z+.07)*8.5);
                        vec3 reflected_sky = mix(vec3(.055,.097,.117),vec3(.36,.49,.61),sky)
                                           + horizon*vec3(.26,.126,.075);
                        float facing = pow(1.0-clamp(abs(dot(normal,sight)),0.0,1.0),3.5);
                        lit = mix(lit*.65,reflected_sky,mix(.32,.88,facing));
                    } else if (surface_material > 1.5) {
                        vec2 plane = vec2(p.x+p.y*.42,surface_position.z);
                        float stone = noise(plane*93.0);
                        float mottling = noise(plane*13.0);
                        float joint = abs(fract(plane.y/.085)-.5);
                        float seam = (1.0-smoothstep(.004,.017+fwidth(joint),joint))*.10;
                        lit *= .83 + .18*mottling + .14*stone*(1.0-smoothstep(.003,.020,length(fwidth(plane)))) - seam;
                    } else if (use_vertex_color) {
                        float weathering = noise(p*31.0 + surface_position.z*13.0);
                        lit *= .89 + .19 * weathering;
                    } else {
                        float height = clamp((surface_position.z + .005) / .06, 0.0, 1.0);
                        float folds = .5 + .5 * cos(height * 18.85);
                        float side = 1.0 - smoothstep(.45,.95,abs(normal.z));
                        float detail = 1.0 - smoothstep(.15,.45,fwidth(height * 3.0));
                        lit *= mix(1.05, .79 + .36 * folds * detail, side);
                        lit *= .92 + .16 * noise(p * 14.0);
                    }
                }
                if (car_material) {
                    vec3 halfway = normalize(direction_to_light + sight);
                    vec3 reflected = reflect(-sight, normal);
                    float pearl = smoothstep(.50,.76,min(surface_color.g,surface_color.b));
                    float polished = max(lacquer,pearl);
                    float fresnel = .032 + .36 * pow(1.0 - max(dot(normal,sight),0.0),5.0);
                    float sky = smoothstep(-.12,.20,reflected.z);
                    vec3 horizon = mix(vec3(.50,.60,.73),vec3(1.0,.66,.36),
                                       pow(max(dot(reflected,direction_to_light),0.0),3.0));
                    if (floodlight_count > 0) {
                        horizon = mix(vec3(.33,.44,.65),vec3(.68,.43,.44),
                                      pow(max(dot(reflected,direction_to_light),0.0),3.0));
                    }
                    vec3 environment = mix(vec3(.10,.105,.079),horizon,sky);
                    float specular_angle = max(dot(normal,halfway),0.0);
                    float specular = pow(specular_angle,60.0);
                    float sheen = pow(specular_angle,18.0) * .08;
                    lit *= 1.0 - fresnel * polished * .22;
                    lit += environment * fresnel * mix(.22,1.1,polished);
                    lit += vec3(1.0,.82,.64) * (specular + sheen) * visibility * mix(.06,.60,polished);
                    lit += vec3(1.0,.86,.66) * lamp_specular * mix(.055,.28,polished);
                    float carbon = 1.0 - smoothstep(.05,.115,max(surface_color.r,surface_color.g));
                    vec2 weave = (local_position.xy + local_position.z*vec2(.41,-.31))*1900.0;
                    float thread = sin(weave.x+weave.y)*sin(weave.x-weave.y);
                    float weave_detail = 1.0 - smoothstep(.7,3.0,length(fwidth(weave)));
                    lit *= 1.0 + .095 * carbon * thread * weave_detail;
                    if (surface_color.r > .99 && abs(surface_color.g - .52) < .001) {
                        lit = vec3(1.05,.12,.068);
                    }
                }
                if (scenery_material && surface_color.r > .99 && surface_color.b > .99) {
                    int light_index = int(round(surface_color.g * 4.0));
                    lit = light_index < start_lights ? vec3(1.50,.14,.06) : vec3(.17,.065,.05);
                }
                if (scenery_material && surface_color.r > .99 && surface_color.g > .68 && surface_color.b < .16) {
                    lit = vec3(1.52,1.22,.78);
                }
                if (scenery_material && surface_color.b > .99 && surface_color.r < .10 && surface_color.g > .60) {
                    lit = vec3(.25,1.12,1.48);
                }
                if (scenery_material && surface_color.r > .99 && surface_color.g > .50 &&
                    surface_color.g < .55 && surface_color.b > .21 && surface_color.b < .25) {
                    lit = max(lit,vec3(.98,.60,.23));
                }
                if (race_scene) {
                    vec3 distance_to_eye = camera_position - surface_position;
                    float haze = 1.0 - exp(-length(distance_to_eye.xy) * .036);
                    float aerial_view = smoothstep(.35,.85,sight.z);
                    float forward_scatter = pow(max(dot(-sight,direction_to_light),0.0),5.0);
                    vec3 air = floodlight_count > 0
                             ? mix(vec3(.29,.35,.54),vec3(.57,.40,.50),forward_scatter)
                             : mix(vec3(.61,.53,.56),vec3(.92,.69,.46),forward_scatter);
                    lit = mix(lit, air, haze * mix(.48,.09,aerial_view));
                }
                fragment_color = vec4(pow(max(lit,vec3(0.0)),vec3(2.2)), 1.0);
            }
        )glsl";

        simulation::TrackSettings curb_geometry_settings(const simulation::TrackSettings &track) {
            constexpr double desired_curb_width = 0.045;

            double curb_width = desired_curb_width;

            if (!track.path) {
                const double inner_radius =
                        track.centerline_radius - track.road_width * 0.5;

                curb_width = std::min(desired_curb_width, inner_radius * 0.5);
            }

            auto geometry = track;
            geometry.road_width += curb_width * 2.0;

            return geometry;
        }

        BarrierSettings scene_barriers(const simulation::TrackSettings &track) {
            BarrierSettings barriers{};
            if (track.path && track.road_width >= 0.46) {
                barriers.clearance = simulation::runoff_width;
                barriers.height = 0.06F;
            }
            return barriers;
        }

        simulation::TrackSettings road_geometry_settings(simulation::TrackSettings track) {
            if (track.path && track.road_width >= 0.46) {
                track.road_width += simulation::runoff_width * 2.0;
            }
            return track;
        }

        float scene_half_extent(const simulation::TrackSettings &track) {
            constexpr double scenery_margin = 1.0;
            constexpr double minimum_extent = 3.0;

            const auto barriers = scene_barriers(track);
            const double wall_offset =
                    track.road_width * 0.5
                    + barriers.clearance
                    + barriers.thickness;

            double extent = track.centerline_radius + wall_offset;

            if (track.path) {
                const auto boundaries =
                        simulation::make_track_boundaries(*track.path, wall_offset);

                extent = 0.0;

                for (std::size_t index = 0; index < boundaries.left.size(); ++index) {
                    const auto &left = boundaries.left[index];
                    const auto &right = boundaries.right[index];

                    extent = std::max({
                        extent,
                        std::abs(left.x),
                        std::abs(left.y),
                        std::abs(right.x),
                        std::abs(right.y)
                    });
                }
            }

            extent = std::max(minimum_extent, extent + scenery_margin);

            if (!std::isfinite(extent) ||
                extent > std::numeric_limits<float>::max()) {
                throw std::invalid_argument("Scene exceeds the rendering range");
            }

            return static_cast<float>(extent);
        }

        std::vector<glm::vec2> circuit_groves(const simulation::TrackSettings &track, float crown_radius) {
            const auto &path = *track.path;
            const auto barriers = scene_barriers(track);
            const double road_clearance = track.road_width * 0.5 + barriers.clearance
                                          + barriers.thickness + crown_radius + 0.30;
            const double spacing = crown_radius * 2.9;
            const auto paddock = path.sample_at(simulation::start_line_distance);
            const bool has_paddock = track.road_width >= 0.46;

            struct Grove {
                double lap_fraction;
                double side_offset;
                double length_radius;
                double depth_radius;
                int trees;
            };

            // Offset grove centers from the road and vary their depth to leave open meadows between them.
            constexpr Grove groves[]{
                {.10, -3.2, 2.4, 1.2, 15},
                {.19, -1.9, 2.3, 1.1, 17},
                {.25, -4.3, 2.4, 1.5, 15},
                {.26,  1.7, 1.2, 0.7,  7},
                {.35,  2.5, 1.7, 1.1, 13},
                {.41, -1.8, 1.4, 0.8,  9},
                {.49, -2.8, 2.7, 1.6, 22},
                {.56,  2.4, 2.5, 1.7, 18},
                {.63, -3.1, 2.3, 1.3, 14},
                {.72,  2.1, 1.8, 1.2, 13},
                {.80, -2.7, 2.2, 1.7, 18},
                {.88,  2.2, 1.9, 1.0, 12},
                {.94, -2.2, 1.6, 1.0,  9},
                {.22, -5.0, 3.4, 1.9, 24},
                {.46, -4.5, 3.6, 1.7, 21},
                {.65, -4.0, 3.2, 2.1, 25},
                {.84, -3.5, 2.7, 1.8, 18},
                {.23, -7.0, 3.7, 1.9, 18},
                {.44, -5.8, 4.1, 1.8, 19},
                {.67, -5.5, 3.3, 1.9, 15},
            };

            std::mt19937 random{12631};
            const auto unit = [&random] { return static_cast<double>(random()) / std::mt19937::max(); };
            std::vector<glm::vec2> positions;
            for (const auto &grove : groves) {
                const auto anchor = path.sample_at(grove.lap_fraction * path.length());
                const int target_count = grove.trees + grove.trees / 3;
                for (int planted = 0, attempts = 0; planted < target_count && attempts < target_count * 12; ++attempts) {
                    const double angle = unit() * 2.0 * std::numbers::pi_v<double>;
                    const double radius = std::pow(unit(), .72);
                    const double along = grove.length_radius * radius * std::cos(angle);
                    const double lateral = grove.side_offset + grove.depth_radius * radius * std::sin(angle);
                    const math::Vec2 candidate{
                        anchor.position.x + anchor.tangent.x * along - anchor.tangent.y * lateral,
                        anchor.position.y + anchor.tangent.y * along + anchor.tangent.x * lateral
                    };

                    const double dx = candidate.x - paddock.position.x;
                    const double dy = candidate.y - paddock.position.y;
                    const double paddock_along = dx * paddock.tangent.x + dy * paddock.tangent.y;
                    const double paddock_across = -dx * paddock.tangent.y + dy * paddock.tangent.x;
                    if (has_paddock && paddock_along > -5.2 && paddock_along < 3.0
                        && std::abs(paddock_across) < 2.0) {
                        continue;
                    }

                    // Check the nearest section, including the other side of a hairpin.
                    if (std::abs(path.project(candidate).lateral_offset) < road_clearance) {
                        continue;
                    }

                    if (std::any_of(positions.begin(), positions.end(), [&](const glm::vec2 &tree) {
                        const double x = tree.x - candidate.x;
                        const double y = tree.y - candidate.y;
                        return x * x + y * y < spacing * spacing;
                    })) {
                        continue;
                    }

                    positions.emplace_back(static_cast<float>(candidate.x), static_cast<float>(candidate.y));
                    ++planted;
                }
            }
            return positions;
        }

        CameraSettings scene_camera_settings(const simulation::TrackSettings &track) {
            CameraSettings settings{};

            if (track.path) {
                const float extent = scene_half_extent(track);
                const float radius = std::sqrt(2.0F) * extent;

                const float half_fov =
                        glm::radians(settings.vertical_fov_degrees) * 0.5F;

                const float distance =
                        0.88F * radius / std::sin(half_fov);

                const glm::vec3 direction = glm::normalize(
                    settings.position - settings.target
                );

                settings.position =
                        settings.target + direction * distance;

                settings.maximum_distance = distance * 2.0F;
                settings.far_clip =
                        settings.maximum_distance + radius * 2.0F;

                if (!std::isfinite(settings.far_clip)) {
                    throw std::invalid_argument("Scene exceeds the camera range");
                }
            }

            return settings;
        }

        struct CircuitOverview {
            glm::vec3 target{};
            glm::vec2 half_size{};

            explicit CircuitOverview(const simulation::TrackSettings &track) {
                if (track.path) {
                    const auto points = track.path->points();
                    auto lo = points.front(), hi = lo;
                    for (const auto &point : points) {
                        lo.x = std::min(lo.x, point.x);
                        lo.y = std::min(lo.y, point.y);
                        hi.x = std::max(hi.x, point.x);
                        hi.y = std::max(hi.y, point.y);
                    }
                    target = {0.5 * (lo.x + hi.x), 0.5 * (lo.y + hi.y), 0.0F};
                    half_size = {0.5 * (hi.x - lo.x) + 1.5, 0.5 * (hi.y - lo.y) + 1.5};
                }
            }

            float distance(float aspect, const CameraSettings &camera) const {
                const float half_fov = glm::radians(camera.vertical_fov_degrees * .5F);
                const float extent = std::max(half_size.x / aspect, half_size.y);
                const float depth = half_size.y * std::cos(glm::radians(85.0F));
                return std::min((extent / std::tan(half_fov) + depth) * 1.09F, camera.maximum_distance);
            }
        };

        struct TracksideCamera {
            std::string_view name;
            double distance;
            glm::vec3 eye;
        };

        std::vector<TracksideCamera> make_trackside_cameras(const simulation::TrackPath &path,
                                                            const GroundMesh &ground) {
            struct Station {
                std::string_view name;
                double lap_fraction;
                float along, across, height;
            };
            // Each eye stays fixed. Only its aim and lens change as the car goes by.
            constexpr Station stations[]{
                {"GRID APPROACH",    .017,  .7F,  .99F, .82F},
                {"START STRAIGHT",   .083, -.8F, -.38F, .43F},
                {"PIT EXIT",         .158,  .9F,  .98F, .62F},
                {"TURN ONE",         .228,  .8F, -1.02F, .65F},
                {"LAKESIDE",         .309,  .7F, -.93F, .60F},
                {"EAST ESSES",       .397,  .7F,  .97F, .72F},
                {"FOOTBRIDGE",       .487, -1.4F, .57F, .33F},
                {"NORTH STRAIGHT",   .552,  .8F, -.92F, .76F},
                {"INFIELD HAIRPIN",  .632,  .3F, -.91F, .66F},
                {"RIDGE SWEEP",      .717,  .6F,  .97F, .74F},
                {"WEST BEND",        .798,  .7F,  .92F, .60F},
                {"WEST ESSES",       .872,  .5F, -.98F, .67F},
                {"FINAL CORNER",     .942,  .8F, -1.02F, .72F},
            };

            std::vector<TracksideCamera> cameras;
            cameras.reserve(std::size(stations));
            for (const auto &station : stations) {
                const double distance = path.length() * station.lap_fraction;
                const auto sample = path.sample_at(distance);
                const float x = static_cast<float>(sample.position.x + sample.tangent.x*station.along
                                                   - sample.tangent.y*station.across);
                const float y = static_cast<float>(sample.position.y + sample.tangent.y*station.along
                                                   + sample.tangent.x*station.across);
                cameras.push_back({station.name, distance, {x, y, ground.height_at(x,y) + station.height}});
            }
            return cameras;
        }

        struct WindowDeleter {
            void operator()(GLFWwindow *window) const noexcept {
                glfwDestroyWindow(window);
            }
        };

        using Window = std::unique_ptr<GLFWwindow, WindowDeleter>;

        Window create_window() {
            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
            glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);


#ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GLFW_TRUE);
#endif
            Window window{
                glfwCreateWindow(1200, 720, "Apex", nullptr, nullptr),
            };

            if (!window) {
                throw std::runtime_error{"Failed to create Apex window"};
            }

            glfwMakeContextCurrent(window.get());

            if (gladLoadGL(glfwGetProcAddress) == 0) {
                throw std::runtime_error{"Failed to load OpenGL functions"};
            }

            glfwSwapInterval(1);

            return window;
        }
    }

    struct SimulationWindow::Impl {
        platform::GlfwSession glfw;
        Window window{create_window()};
        simulation::TrackSettings track_settings;

        ShaderProgram program;

        ShaderProgram shadow_program;
        ShadowMap shadow_map;
        SceneCompositor compositor;

        OrbitCamera camera;
        CircuitOverview overview;
        RaceHud hud;
        GLint start_line_location{};
        GLint terrain_location{};
        GLint water_location{};
        GLint scene_time_location{};
        GLint race_scene_location{};
        GLint camera_position_location{};
        GLint livery_location{};
        GLint car_number_location{};
        GLint wheel_angles_location{};
        GLint car_material_location{};
        GLint scenery_material_location{};
        GLint foliage_material_location{};
        GLint start_lights_location{};
        std::string window_title;
        bool race_camera_initialized{};
        bool show_sensors{};
        bool sensors_was_pressed{};

        GLint shadow_offset_location{};
        GLint shadow_heading_location{};
        GLint shadow_scale_location{};
        GLint shadow_wheel_angles_location{};
        GLint shadow_elevation_location{};
        GLint light_view_projection_location{};

        TrackMesh track_mesh;
        TrackMesh curb_mesh;
        CarMesh vehicle_mesh;
        GroundMesh ground_mesh;
        GroundMesh reservoir_surface{6.5F,nullptr,&ground_mesh};
        RaceScenery scenery;
        BarrierMesh barrier_mesh;
        TreeSettings tree_settings{};
        TreeMesh tree_mesh{tree_settings};
        const TreeSettings mature_tree_settings{0.142F, 0.017F, 0.145F, 14, 1.0F, true};
        TreeMesh mature_tree_mesh{mature_tree_settings};
        const TreeSettings pine_settings{0.122F, 0.014F, 0.107F, 12, 2.9F, false, true};
        TreeMesh pine_mesh{pine_settings};
        std::vector<glm::vec2> tree_positions;

        SensorLines sensor_lines;

        GLint offset_location{};
        GLint view_projection_location{};
        GLint elevation_location{};
        GLint heading_location{};
        GLint scale_location{};
        GLint color_location{};
        GLint curb_pattern_location{};
        GLint road_half_width_location{};
        GLint curb_segments_location{};
        GLint lighting_enabled_location{};
        GLint direction_to_light_location{};
        GLint floodlight_count_location{};
        GLint floodlights_location{};
        GLint floodlight_directions_location{};
        GLint use_vertex_color_location{};
        GLint shadow_depth_location{};
        GLint scene_light_view_projection_location{};
        GLint road_markings_enabled_location{};
        GLint track_length_location{};
        GLint braking_zones_location{};

        bool reset_was_pressed{};
        bool pause_was_pressed{};
        bool continue_was_pressed{};
        bool autopilot_was_pressed{};
        bool camera_reset_was_pressed{};
        bool orbit_dragging{};

        bool follow_vehicle{};
        bool high_chase{};
        bool follow_was_pressed{};
        bool trackside_view{};
        bool trackside_was_pressed{};
        std::vector<TracksideCamera> trackside_cameras;
        std::optional<std::size_t> trackside_station;

        std::optional<double> follow_heading;
        std::optional<std::size_t> followed_slot;
        double follow_time{};

        double previous_cursor_x{};
        double previous_cursor_y{};
        double scroll_delta{};
        int pending_car_selection{};
        bool pending_follow_leader{};


        explicit Impl(const simulation::TrackSettings &track)
            : track_settings{track}, camera{scene_camera_settings(track)}, overview{track},
              track_mesh{road_geometry_settings(track)}, curb_mesh{curb_geometry_settings(track)},
              ground_mesh{scene_half_extent(track) * 4.0F, &track}, scenery{track, ground_mesh},
              barrier_mesh{track, scene_barriers(track)} {
            if (track.path) {
                trackside_cameras = make_trackside_cameras(*track.path, ground_mesh);
                tree_positions = circuit_groves(track, mature_tree_settings.crown_radius);
                std::erase_if(tree_positions, [&](glm::vec2 position) {
                    return ground_mesh.height_at(position.x, position.y) < -.085F;
                });
            } else {
                constexpr int tree_count = 16;
                constexpr float tree_clearance = 0.10F;
                constexpr float radial_variation = 0.08F;

                const BarrierSettings barrier_settings{};

                const float base_radius =
                        static_cast<float>(track.centerline_radius + track.road_width * 0.5)
                        + barrier_settings.clearance
                        + barrier_settings.thickness
                        + tree_settings.crown_radius
                        + tree_clearance;

                tree_positions.reserve(tree_count);

                for (int index = 0; index < tree_count; ++index) {
                    const float angle =
                            2.0F * std::numbers::pi_v<float>
                            * (static_cast<float>(index) + 0.5F)
                            / static_cast<float>(tree_count);

                    const float radius =
                            base_radius
                            + radial_variation * 0.5F
                            * (1.0F + std::sin(angle * 3.0F));

                    tree_positions.emplace_back(
                        radius * std::cos(angle),
                        radius * std::sin(angle)
                    );
                }
            }
            Shader vertex_shader{GL_VERTEX_SHADER};
            vertex_shader.compile((std::string{wheel_shader_source} + vertex_shader_source).c_str());

            Shader fragment_shader{GL_FRAGMENT_SHADER};
            fragment_shader.compile(fragment_shader_source);

            program.link(vertex_shader, fragment_shader);

            Shader shadow_vertex_shader{GL_VERTEX_SHADER};
            shadow_vertex_shader.compile((std::string{wheel_shader_source} + shadow_vertex_shader_source).c_str());

            Shader shadow_fragment_shader{GL_FRAGMENT_SHADER};
            shadow_fragment_shader.compile(shadow_fragment_shader_source);

            shadow_program.link(shadow_vertex_shader, shadow_fragment_shader);

            shadow_offset_location = shadow_program.uniform_location("offset");
            shadow_heading_location = shadow_program.uniform_location("heading");
            shadow_scale_location = shadow_program.uniform_location("mesh_scale");
            shadow_wheel_angles_location = shadow_program.uniform_location("wheel_angles");
            shadow_elevation_location = shadow_program.uniform_location("elevation");
            light_view_projection_location = shadow_program.uniform_location("light_view_projection");

            offset_location = program.uniform_location("offset");
            view_projection_location = program.uniform_location("view_projection");
            elevation_location = program.uniform_location("elevation");
            heading_location = program.uniform_location("heading");
            scale_location = program.uniform_location("mesh_scale");
            wheel_angles_location = program.uniform_location("wheel_angles");
            color_location = program.uniform_location("color");

            curb_pattern_location = program.uniform_location("curb_pattern");
            road_markings_enabled_location = program.uniform_location("road_markings_enabled");
            road_half_width_location = program.uniform_location("road_half_width");
            curb_segments_location = program.uniform_location("curb_segments");

            lighting_enabled_location = program.uniform_location("lighting_enabled");
            direction_to_light_location = program.uniform_location("direction_to_light");
            floodlight_count_location = program.uniform_location("floodlight_count");
            floodlights_location = program.uniform_location("floodlights[0]");
            floodlight_directions_location = program.uniform_location("floodlight_directions[0]");

            use_vertex_color_location = program.uniform_location("use_vertex_color");

            shadow_depth_location = program.uniform_location("shadow_depth");
            scene_light_view_projection_location = program.uniform_location("light_view_projection");

            track_length_location = program.uniform_location("track_length");
            start_line_location = program.uniform_location("start_line");
            braking_zones_location = program.uniform_location("braking_zones[0]");
            terrain_location = program.uniform_location("terrain");
            water_location = program.uniform_location("water");
            scene_time_location = program.uniform_location("scene_time");
            race_scene_location = program.uniform_location("race_scene");
            camera_position_location = program.uniform_location("camera_position");
            livery_location = program.uniform_location("livery");
            car_number_location = program.uniform_location("car_number");
            car_material_location = program.uniform_location("car_material");
            scenery_material_location = program.uniform_location("scenery_material");
            foliage_material_location = program.uniform_location("foliage_material");
            start_lights_location = program.uniform_location("start_lights");

            glEnable(GL_DEPTH_TEST);

            glfwSetWindowUserPointer(window.get(), this);

            glfwSetScrollCallback(
                window.get(),
                [](GLFWwindow *window, double, double vertical_offset) {
                    auto *view = static_cast<Impl *>(glfwGetWindowUserPointer(window));

                    view->scroll_delta += vertical_offset;
                }
            );
            glfwSetKeyCallback(window.get(), [](GLFWwindow *window, int key, int, int action, int) {
                auto &view = *static_cast<Impl *>(glfwGetWindowUserPointer(window));
                if (action == GLFW_PRESS || action == GLFW_REPEAT) {
                    if (key == GLFW_KEY_LEFT || key == GLFW_KEY_UP) {
                        --view.pending_car_selection;
                    } else if (key == GLFW_KEY_RIGHT || key == GLFW_KEY_DOWN) {
                        ++view.pending_car_selection;
                    } else if (key == GLFW_KEY_TAB && action == GLFW_PRESS) {
                        view.pending_follow_leader = true;
                    }
                }
            });
        }

        void set_driving_view(bool high) {
            high_chase = high;
            follow_vehicle = true;
            trackside_view = false;
            follow_heading.reset();
            camera.reset();
            camera.set_distance(high ? 1.15F : 0.72F);
            camera.orbit(0.0F, glm::radians(high ? -8.0F : -29.0F));
            orbit_dragging = false;
            scroll_delta = 0.0;
        }

        void set_trackside_view() {
            follow_vehicle = false;
            trackside_view = true;
            trackside_station.reset();
            followed_slot.reset();
            orbit_dragging = false;
            scroll_delta = 0.0;
        }

        std::string_view frame_trackside_camera(CameraSettings &settings, const simulation::WorldState &world,
                                               std::size_t slot) {
            const auto &path = *track_settings.path;
            const auto &car = world.vehicle;
            const auto projection = path.project(car.body.position);
            const auto separation = [&](const TracksideCamera &station) {
                return std::abs(std::remainder(projection.distance_along_track - station.distance, path.length()));
            };
            const auto nearest = std::ranges::min_element(trackside_cameras, {}, separation);
            const std::size_t next = static_cast<std::size_t>(nearest - trackside_cameras.begin());
            constexpr double cut_margin = .36;
            if (!trackside_station || followed_slot != slot || world.elapsed_time < follow_time ||
                separation(trackside_cameras[*trackside_station]) > separation(*nearest) + cut_margin) {
                trackside_station = next;
            }
            followed_slot = slot;
            follow_time = world.elapsed_time;

            const auto &station = trackside_cameras[*trackside_station];
            const auto &position = car.body.position;
            const auto &velocity = car.body.velocity;
            constexpr float lead_time = .075F;
            settings.position = station.eye;
            settings.target = {position.x + std::clamp(velocity.x*lead_time, -.14, .14),
                               position.y + std::clamp(velocity.y*lead_time, -.14, .14), .060F};
            const float range = glm::length(settings.position - settings.target);
            settings.vertical_fov_degrees = std::clamp(glm::degrees(2.0F * std::atan2(.40F,range)), 12.0F, 49.0F);
            return station.name;
        }

        float overview_distance() const {
            int width{}, height{};
            glfwGetFramebufferSize(window.get(), &width, &height);
            const float aspect = static_cast<float>(std::max(width, 1)) / std::max(height, 1);
            return overview.distance(aspect, camera.settings());
        }

        void reset_camera() {
            camera.reset();
            if (track_settings.path) {
                const auto offset = camera.settings().position - camera.settings().target;
                camera.set_target(overview.target);
                camera.set_distance(overview_distance());
                camera.orbit(-glm::half_pi<float>() - std::atan2(offset.y, offset.x), glm::half_pi<float>());
            }
            follow_vehicle = false;
            trackside_view = false;
            orbit_dragging = false;
            scroll_delta = 0.0;
        }

        void draw_trees(GLint offset_uniform, GLint heading_uniform, GLint elevation_uniform, GLint scale_uniform,
                        float elevation) const {
            for (std::size_t index = 0; index < tree_positions.size(); ++index) {
                const auto &position = tree_positions[index];
                glUniform2f(offset_uniform, position.x, position.y);
                glUniform1f(heading_uniform, static_cast<float>(index) * 2.399963F);
                const float terrain_height = ground_mesh.height_at(position.x, position.y);
                glUniform1f(elevation_uniform, elevation + terrain_height);
                const float variation = static_cast<float>((index * 47 + 19) % 101) / 100.0F;
                glUniform1f(scale_uniform, track_settings.path ? std::lerp(.72F,1.28F,variation) : 1.0F);
                const bool mature = track_settings.path && variation > .23F;
                const bool pine = terrain_height > std::lerp(.19F,.77F,variation);
                (pine ? pine_mesh : (mature ? mature_tree_mesh : tree_mesh)).draw();
            }
            glUniform1f(scale_uniform, 1.0F);
        }

        void clear_chase_view(CameraSettings &view, float ground_elevation) const {
            constexpr int samples = 20;
            constexpr float crown_clearance = .205F;
            for (int step = 1; step <= samples; ++step) {
                const float distance = static_cast<float>(step) / samples;
                const auto point = glm::mix(view.target,view.position,distance);
                const float ground = ground_mesh.height_at(point.x,point.y) + ground_elevation;
                const bool blocked = point.z < ground + .045F || std::ranges::any_of(tree_positions,[&](glm::vec2 tree) {
                    const auto delta = glm::vec2{point} - tree;
                    return glm::dot(delta,delta) < crown_clearance*crown_clearance &&
                           point.z < ground_mesh.height_at(tree.x,tree.y) + ground_elevation + .59F;
                });
                if (blocked) {
                    view.position = glm::mix(view.target,view.position,std::max(.20F,distance-.075F));
                    view.position.z = std::max(view.position.z,
                        ground_mesh.height_at(view.position.x,view.position.y) + ground_elevation + .075F);
                    return;
                }
            }
        }
    };

    SimulationWindow::SimulationWindow(const simulation::TrackSettings &track) : impl_(std::make_unique<Impl>(track)) {
    }

    SimulationWindow::~SimulationWindow() {
        close();
    }

    WindowInput SimulationWindow::poll_events(bool spectate) {
        WindowInput input{};

        if (!impl_) {
            input.close_requested = true;
            return input;
        }

        glfwPollEvents();

        auto &view = *impl_;
        auto *window = view.window.get();

        const auto key_down = [window](int key) {
            return glfwGetKey(window, key) == GLFW_PRESS;
        };

        input.close_requested =
                glfwWindowShouldClose(window) == GLFW_TRUE ||
                key_down(GLFW_KEY_ESCAPE);

        if (input.close_requested) {
            return input;
        }

        const auto pressed_once =
                [&key_down](int key, bool &was_pressed) {
            const bool pressed = key_down(key);
            const bool triggered = pressed && !was_pressed;
            was_pressed = pressed;
            return triggered;
        };

        input.reset_requested =
                pressed_once(GLFW_KEY_R, view.reset_was_pressed);

        if (input.reset_requested) {
            view.follow_heading.reset();
            view.trackside_station.reset();
        }

        input.pause_requested =
                pressed_once(GLFW_KEY_SPACE, view.pause_was_pressed);
        input.continue_requested = spectate && pressed_once(GLFW_KEY_ENTER, view.continue_was_pressed);

        input.autopilot_requested =
                pressed_once(GLFW_KEY_P, view.autopilot_was_pressed);

        const bool focused =
                glfwGetWindowAttrib(window, GLFW_FOCUSED) == GLFW_TRUE;
        const int selection = std::exchange(view.pending_car_selection, 0);
        const bool follow_leader = std::exchange(view.pending_follow_leader, false);
        if (spectate && focused) {
            input.car_selection_delta = selection;
            input.follow_leader_requested = follow_leader;
        }

        if (focused) {
            if (pressed_once(GLFW_KEY_V, view.sensors_was_pressed)) {
                view.show_sensors = !view.show_sensors;
            }
            if (pressed_once(GLFW_KEY_F, view.follow_was_pressed)) {
                view.set_driving_view(view.follow_vehicle && !view.high_chase);
            }
            if (pressed_once(GLFW_KEY_T, view.trackside_was_pressed) && !view.trackside_cameras.empty()) {
                if (view.trackside_view) {
                    view.set_driving_view(false);
                } else {
                    view.set_trackside_view();
                }
            }

            if (pressed_once(GLFW_KEY_C, view.camera_reset_was_pressed)) {
                view.reset_camera();
            }

            double cursor_x{};
            double cursor_y{};
            glfwGetCursorPos(window, &cursor_x, &cursor_y);

            const bool dragging =
                    !view.trackside_view && (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS ||
                                             glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

            if (dragging && view.orbit_dragging) {
                constexpr double orbit_sensitivity = 0.005;

                const double horizontal_change =
                        cursor_x - view.previous_cursor_x;

                const double vertical_change =
                        cursor_y - view.previous_cursor_y;

                view.camera.orbit(
                    static_cast<float>(-horizontal_change * orbit_sensitivity),
                    static_cast<float>(vertical_change * orbit_sensitivity)
                );
            }

            view.previous_cursor_x = cursor_x;
            view.previous_cursor_y = cursor_y;
            view.orbit_dragging = dragging;

            if (!view.trackside_view && view.scroll_delta != 0.0) {
                const float amount = static_cast<float>(std::clamp(view.scroll_delta, -50.0, 50.0));

                view.camera.zoom(amount);
                if (view.track_settings.path) {
                    const auto &camera = view.camera.settings();
                    const float limit = view.follow_vehicle ? 2.4F : view.overview_distance();
                    if (glm::length(camera.position - camera.target) > limit) {
                        view.camera.set_distance(limit);
                    }
                }
            }
            view.scroll_delta = 0.0;
        } else {
            view.orbit_dragging = false;
            view.camera_reset_was_pressed = false;
            view.scroll_delta = 0.0;
            view.follow_was_pressed = false;
            view.trackside_was_pressed = false;
        }

        if (!focused || spectate) {
            return input;
        }

        if (key_down(GLFW_KEY_W) || key_down(GLFW_KEY_UP)) {
            input.controls.throttle += 1.0;
        }

        if (key_down(GLFW_KEY_S) || key_down(GLFW_KEY_DOWN)) {
            input.controls.throttle -= 1.0;
        }

        if (key_down(GLFW_KEY_A) || key_down(GLFW_KEY_LEFT)) {
            input.controls.steering += 1.0;
        }

        if (key_down(GLFW_KEY_D) || key_down(GLFW_KEY_RIGHT)) {
            input.controls.steering -= 1.0;
        }

        return input;
    }

    void SimulationWindow::draw(const simulation::WorldState &world, const simulation::SensorSettings &sensors,
                                std::string_view status, std::span<const simulation::VehicleState> opponents,
                                const simulation::RaceStatus *race, std::size_t focused_slot,
                                std::string_view focused_driver, std::span<const std::string> driver_names,
                                const TrainingTelemetry *training, std::span<const std::size_t> eliminated_slots) {
        if (!impl_) {
            throw std::runtime_error{"Cannot draw a closed window"};
        }
        if (!driver_names.empty() &&
            (driver_names.size() != opponents.size() + 1 || focused_slot >= driver_names.size())) {
            throw std::invalid_argument("Driver names must cover all cars in grid order");
        }
        if (training && (!training->standings.empty() || training->recap)) {
            if (training->standings.size() != driver_names.size() || driver_names.empty() ||
                (training->recap && training->recap->winner_slot >= driver_names.size())) {
                throw std::invalid_argument("Training standings must cover the named grid");
            }
            std::vector<bool> seen(driver_names.size());
            for (const auto &standing : training->standings) {
                if (standing.slot >= seen.size() || seen[standing.slot]) {
                    throw std::invalid_argument("Training standings must list each driver once");
                }
                seen[standing.slot] = true;
            }
        }

        auto &view = *impl_;
        auto *window = view.window.get();

        glfwMakeContextCurrent(window);

        if (race && !view.race_camera_initialized) {
            view.race_camera_initialized = true;
            if (!view.trackside_view) view.set_driving_view(false);
        }
        const std::string title = "Apex | " + std::string{status};
        if (view.window_title != title) {
            glfwSetWindowTitle(window, title.c_str());
            view.window_title = title;
        }

        int width{};
        int height{};
        glfwGetFramebufferSize(window, &width, &height);

        if (width <= 0 || height <= 0) {
            return;
        }

        constexpr float road_elevation = 0.0F;
        constexpr float ground_elevation = -0.005F;
        constexpr float sensor_elevation = 0.002F;
        constexpr float vehicle_elevation = 0.0002F;

        const glm::vec3 direction_to_light =
                glm::normalize(glm::vec3{0.94F, -0.26F, 0.25F});

        float shadow_extent =
                view.track_settings.path
                    ? std::sqrt(2.0F) * scene_half_extent(view.track_settings)
                    : std::max(
                        1.0F, static_cast<float>(view.track_settings.centerline_radius + view.track_settings.road_width
                                                 * 0.5) + 0.3F);

        const auto &largest_tree = view.track_settings.path ? view.mature_tree_settings : view.tree_settings;
        const float tree_height = largest_tree.trunk_height + largest_tree.crown_radius * 1.5F;

        for (const auto &position: view.tree_positions) {
            shadow_extent = std::max(shadow_extent,
                                     glm::length(position) + largest_tree.crown_radius + tree_height);
        }

        const glm::vec3 up{0.0F, 0.0F, 1.0F};
        glm::vec3 shadow_center{0.0F};
        if (race && (view.follow_vehicle || view.trackside_view)) {
            // Spend the shadow map's resolution on the visible track around the driver.
            shadow_extent = 4.5F;
            const auto &car = world.vehicle;
            const glm::vec3 focus{
                static_cast<float>(car.body.position.x + std::cos(car.heading)),
                static_cast<float>(car.body.position.y + std::sin(car.heading)),
                0.0F
            };
            const glm::vec3 right = glm::normalize(glm::cross(up, direction_to_light));
            const glm::vec3 vertical = glm::cross(direction_to_light, right);
            const float texel = 2.0F * shadow_extent / view.shadow_map.resolution();
            const auto snap = [texel](float coordinate) { return std::round(coordinate / texel) * texel; };
            shadow_center = right * snap(glm::dot(focus,right))
                            + vertical * snap(glm::dot(focus,vertical))
                            + direction_to_light * glm::dot(focus,direction_to_light);
        }
        const glm::mat4 light_projection = glm::ortho(-shadow_extent, shadow_extent, -shadow_extent, shadow_extent,
                                                      0.1F, shadow_extent * 6.0F);
        const glm::mat4 light_view = glm::lookAt(shadow_center + direction_to_light * (shadow_extent * 3.0F),
                                                shadow_center, up);

        const glm::mat4 light_view_projection =
                light_projection * light_view;

        view.shadow_map.bind_for_writing();
        view.shadow_program.use();
        glUniform1f(view.shadow_scale_location, 1.0F);

        glUniformMatrix4fv(view.light_view_projection_location, 1, GL_FALSE, glm::value_ptr(light_view_projection));

        glUniform2f(view.shadow_offset_location, 0.0F, 0.0F);
        glUniform1f(view.shadow_heading_location, 0.0F);
        glUniform1f(view.shadow_elevation_location, ground_elevation);

        if (race && view.track_settings.path) {
            view.ground_mesh.draw();
        }
        view.barrier_mesh.draw();

        view.draw_trees(view.shadow_offset_location, view.shadow_heading_location, view.shadow_elevation_location,
                        view.shadow_scale_location, ground_elevation);

        if (race) {
            glUniform2f(view.shadow_offset_location, 0.0F, 0.0F);
            glUniform1f(view.shadow_heading_location, 0.0F);
            glUniform1f(view.shadow_elevation_location, ground_elevation);
            view.scenery.draw();
        }

        const auto draw_car = [&view](const simulation::VehicleState &car, GLint offset, GLint heading, GLint wheels) {
            glUniform2f(offset, static_cast<float>(car.body.position.x), static_cast<float>(car.body.position.y));
            glUniform1f(heading, static_cast<float>(car.heading));
            const auto roll = [&](float radius) {
                return static_cast<float>(std::remainder(car.forward_travel / radius, 2.0 * std::numbers::pi));
            };
            glUniform3f(wheels, roll(CarMesh::rear_tire_radius), roll(CarMesh::front_tire_radius),
                        static_cast<float>(car.steering_angle));
            view.vehicle_mesh.draw();
        };

        glUniform1f(view.shadow_elevation_location, vehicle_elevation);
        for (const auto &opponent : opponents) {
            draw_car(opponent, view.shadow_offset_location, view.shadow_heading_location, view.shadow_wheel_angles_location);
        }
        draw_car(world.vehicle, view.shadow_offset_location, view.shadow_heading_location, view.shadow_wheel_angles_location);

        view.compositor.begin(width, height);

        view.program.use();
        glUniform1f(view.scale_location, 1.0F);

        glUniform1i(view.road_markings_enabled_location, GL_FALSE);

        const double track_length = view.track_settings.path
                                        ? view.track_settings.path->length()
                                        : 2.0 * std::numbers::pi_v<double>
                                          * view.track_settings.centerline_radius;

        glUniform1f(view.track_length_location, static_cast<float>(track_length));
        glUniform1f(view.start_line_location, race ? simulation::start_line_distance : 0.0F);
        glUniform2fv(view.braking_zones_location, RaceScenery::braking_zone_count,
                     glm::value_ptr(view.scenery.braking_zones().front()));
        glUniform1i(view.race_scene_location, race != nullptr);
        glUniform1i(view.car_material_location, GL_FALSE);
        glUniform1f(view.road_half_width_location, static_cast<float>(view.track_settings.road_width * 0.5));

        view.shadow_map.bind_texture(0);
        glUniform1i(view.shadow_depth_location, 0);
        glUniformMatrix4fv(
            view.scene_light_view_projection_location,
            1,
            GL_FALSE,
            glm::value_ptr(light_view_projection)
        );

        glUniform1i(view.use_vertex_color_location, GL_FALSE);

        glUniform1i(view.lighting_enabled_location, GL_TRUE);
        glUniform3fv(view.direction_to_light_location, 1, glm::value_ptr(direction_to_light));
        const auto floodlights = race ? view.scenery.floodlights() : std::span<const glm::vec3>{};
        glUniform1i(view.floodlight_count_location, static_cast<GLint>(floodlights.size()));
        if (!floodlights.empty()) {
            glUniform3fv(view.floodlights_location, static_cast<GLsizei>(floodlights.size()),
                         glm::value_ptr(floodlights.front()));
            glUniform2fv(view.floodlight_directions_location, static_cast<GLsizei>(floodlights.size()),
                         glm::value_ptr(view.scenery.floodlight_directions().front()));
        }

        const float aspect =
                static_cast<float>(width) / static_cast<float>(height);

        if (view.follow_vehicle) {
            const auto &vehicle = world.vehicle;
            const auto &position = vehicle.body.position;

            constexpr double full_turn =
                    2.0 * std::numbers::pi_v<double>;

            double yaw_change{};
            double heading = vehicle.heading;
            const double frame_time = world.elapsed_time - view.follow_time;

            if (view.follow_heading && view.followed_slot == focused_slot && frame_time >= 0.0) {
                const double error = std::remainder(vehicle.heading - *view.follow_heading, full_turn);
                yaw_change = error * -std::expm1(-frame_time / 0.10);
                heading = std::remainder(*view.follow_heading + yaw_change, full_turn);
            } else {
                const auto &settings = view.camera.settings();
                const auto offset = settings.position - settings.target;

                const double current_yaw = std::atan2(
                    static_cast<double>(offset.y),
                    static_cast<double>(offset.x)
                );

                const double behind_car_yaw =
                        vehicle.heading + std::numbers::pi_v<double>;

                yaw_change = behind_car_yaw - current_yaw;
            }

            view.camera.orbit(static_cast<float>(std::remainder(yaw_change, full_turn)), 0.0F);

            view.follow_heading = heading;
            view.followed_slot = focused_slot;
            view.follow_time = world.elapsed_time;

            const float follow_height = view.high_chase ? 0.09F : 0.061F;
            const float look_ahead = view.high_chase ? 0.22F : 0.075F;

            view.camera.set_target(glm::vec3{
                static_cast<float>(position.x + std::cos(heading) * look_ahead),
                static_cast<float>(position.y + std::sin(heading) * look_ahead),
                follow_height
            });
        }

        auto camera = view.camera.settings();
        const auto trackside_name = view.trackside_view ? view.frame_trackside_camera(camera, world, focused_slot)
                                                       : std::string_view{};
        if (race && view.follow_vehicle && view.scenery.footbridge() && camera.position.z > .29F) {
            const auto &bridge = *view.scenery.footbridge();
            const glm::vec2 center{bridge.position.x,bridge.position.y};
            const glm::vec2 tangent{bridge.tangent.x,bridge.tangent.y};
            const glm::vec2 lateral{-tangent.y,tangent.x};
            const auto &position = world.vehicle.body.position;
            const glm::vec2 car = glm::vec2{position.x,position.y} - center;
            const glm::vec2 eye = glm::vec2{camera.position} - center;
            const float car_station = glm::dot(car,tangent);
            const float eye_station = glm::dot(eye,tangent);
            const float distance = car_station * eye_station <= 0 ? 0.0F
                                   : std::min(std::abs(car_station),std::abs(eye_station));
            const float width = static_cast<float>(view.track_settings.road_width) * .5F;
            const float near_bridge = (1.0F - glm::smoothstep(.46F,1.42F,distance))
                                    * (1.0F - glm::smoothstep(width+.12F,width+.72F,std::abs(glm::dot(car,lateral))));
            camera.position.z = std::lerp(camera.position.z,.29F,near_bridge);
        }
        if (race && view.follow_vehicle) view.clear_chase_view(camera,ground_elevation);

        const bool circuit_overview = view.track_settings.path && !view.follow_vehicle && !view.trackside_view;
        // Keep the thin paint and curb layers distinct at aerial distances.
        const float near_clip = circuit_overview
                                ? std::max(camera.near_clip, glm::length(camera.position - camera.target) * 0.025F)
                                : camera.near_clip;
        const glm::mat4 projection = glm::perspective(glm::radians(camera.vertical_fov_degrees), aspect,
                                                      near_clip, camera.far_clip);

        const glm::mat4 camera_view =
                glm::lookAt(
                    camera.position, camera.target, camera.up);

        const glm::mat4 view_projection = projection * camera_view;

        if (race) {
            view.hud.draw_sky(glm::inverse(view_projection), camera.position, direction_to_light, !floodlights.empty());
            view.program.use();
        }
        glUniform3fv(view.camera_position_location, 1, glm::value_ptr(camera.position));
        glUniformMatrix4fv(view.view_projection_location, 1, GL_FALSE, glm::value_ptr(view_projection));


        glUniform2f(view.offset_location, 0.0F, 0.0F);
        glUniform1f(view.heading_location, 0.0F);


        glUniform1i(view.curb_pattern_location, GL_FALSE);
        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.23F, 0.32F, 0.20F);

        glUniform1i(view.terrain_location, GL_TRUE);
        view.ground_mesh.draw();
        glUniform1i(view.terrain_location, GL_FALSE);

        if (view.track_settings.path && view.track_settings.road_width >= 0.46) {
            glUniform1f(view.elevation_location, ground_elevation + GroundMesh::reservoir_level);
            glUniform1f(view.scene_time_location, static_cast<float>(glfwGetTime()));
            glUniform1i(view.water_location, GL_TRUE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            view.reservoir_surface.draw();
            glDisable(GL_BLEND);
            glUniform1i(view.water_location, GL_FALSE);
        }

        glUniform1f(view.elevation_location, road_elevation);
        glUniform3f(view.color_location, 0.22F, 0.245F, 0.27F);
        glUniform1i(view.curb_pattern_location, GL_FALSE);
        glUniform1i(view.road_markings_enabled_location, GL_TRUE);
        view.track_mesh.draw();
        glUniform1i(view.road_markings_enabled_location, GL_FALSE);

        constexpr float curb_elevation = 0.001F;
        constexpr double desired_stripe_length = 0.11;

        const float curb_segments =
                view.track_settings.path
                    ? static_cast<float>(2.0 * std::ceil(track_length / (2.0 * desired_stripe_length)))
                    : 80.0F;

        glUniform1i(view.curb_pattern_location, GL_TRUE);
        glUniform1f(view.curb_segments_location, curb_segments);
        glUniform1f(view.elevation_location, curb_elevation);

        view.curb_mesh.draw();

        glUniform1i(view.curb_pattern_location, GL_FALSE);

        glUniform1f(view.elevation_location, ground_elevation);
        glUniform3f(view.color_location, 0.47F, 0.49F, 0.49F);
        glUniform1i(view.scenery_material_location, race && view.track_settings.path);
        view.barrier_mesh.draw();
        glUniform1i(view.scenery_material_location, GL_FALSE);

        glUniform1i(view.use_vertex_color_location, GL_TRUE);
        glUniform3f(view.color_location, 1.0F, 1.0F, 1.0F);

        glUniform1i(view.foliage_material_location, GL_TRUE);
        view.draw_trees(view.offset_location, view.heading_location, view.elevation_location, view.scale_location,
                        ground_elevation);
        glUniform1i(view.foliage_material_location, GL_FALSE);

        if (race) {
            glUniform2f(view.offset_location, 0.0F, 0.0F);
            glUniform1f(view.heading_location, 0.0F);
            glUniform1f(view.elevation_location, ground_elevation);
            glUniform1i(view.scenery_material_location, GL_TRUE);
            glUniform1i(view.start_lights_location, race->start_lights());
            view.scenery.draw();
            glUniform1i(view.scenery_material_location, GL_FALSE);
        }

        if (!race || view.show_sensors) {
            const auto readings = simulation::sense_track(world.vehicle, view.track_settings, sensors);
            view.sensor_lines.update(world.vehicle, sensors, readings);
            glUniform1i(view.use_vertex_color_location, GL_FALSE);
            glUniform2f(view.offset_location, 0.0F, 0.0F);
            glUniform1f(view.heading_location, 0.0F);
            glUniform1f(view.elevation_location, sensor_elevation);
            glUniform3f(view.color_location, 1.0F, 0.85F, 0.15F);
            glUniform1i(view.lighting_enabled_location, GL_FALSE);
            view.sensor_lines.draw();
            glUniform1i(view.lighting_enabled_location, GL_TRUE);
        }

        constexpr glm::vec3 teams[]{
            {0.05F,0.60F,0.72F}, {0.95F,0.48F,0.08F}, {0.78F,0.83F,0.88F},
            {0.23F,0.33F,0.90F}, {0.18F,0.63F,0.36F}, {0.72F,0.105F,0.067F},
            {0.40F,0.26F,0.77F}, {0.71F,0.79F,0.10F}, {0.17F,0.20F,0.24F},
            {0.07F,0.64F,0.54F}, {0.48F,0.64F,0.93F}
        };
        const auto livery = [&](std::size_t slot) -> const glm::vec3 & {
            return teams[(slot / 2) % std::size(teams)];
        };
        glUniform1f(view.elevation_location, vehicle_elevation);
        glUniform3f(view.color_location, 1.0F, 1.0F, 1.0F);
        glUniform1i(view.use_vertex_color_location, GL_TRUE);
        glUniform1i(view.car_material_location, GL_TRUE);
        for (std::size_t index = 0; index < opponents.size(); ++index) {
            const auto slot = index + (index >= focused_slot);
            glUniform3fv(view.livery_location, 1, glm::value_ptr(livery(slot)));
            glUniform1i(view.car_number_location, static_cast<int>(slot % simulation::grid_size) + 1);
            draw_car(opponents[index], view.offset_location, view.heading_location, view.wheel_angles_location);
        }
        glUniform3fv(view.livery_location, 1, glm::value_ptr(livery(focused_slot)));
        glUniform1i(view.car_number_location, static_cast<int>(focused_slot % simulation::grid_size) + 1);
        draw_car(world.vehicle, view.offset_location, view.heading_location, view.wheel_angles_location);

        view.compositor.present(race != nullptr, circuit_overview);

        if (race && view.track_settings.path) {
            const auto eliminated = [&](std::size_t slot) {
                return std::ranges::find(eliminated_slots, slot) != eliminated_slots.end();
            };
            std::vector<BroadcastDriver> standings;
            if (training) {
                standings.reserve(training->standings.size());
                for (const auto &row : training->standings) {
                    const auto slot = row.slot;
                    const auto &car = slot == focused_slot ? world.vehicle : opponents[slot - (slot > focused_slot)];
                    standings.push_back({slot, driver_names[slot], livery(slot), car.body.position,
                                         row.progress, row.lap_time, eliminated(slot)});
                }
            }
            std::vector<DriverNameplate> nameplates;
            if (!driver_names.empty() && !(training && training->recap)) {
                nameplates.reserve(driver_names.size());
                const auto project_helmet = [&](const simulation::VehicleState &car, std::size_t slot) {
                    constexpr float helmet_offset = -.016F;
                    constexpr float helmet_height = .063F;
                    const auto &position = car.body.position;
                    const auto clip = view_projection * glm::vec4{
                        position.x + helmet_offset * std::cos(car.heading),
                        position.y + helmet_offset * std::sin(car.heading),
                        vehicle_elevation + helmet_height, 1.0F
                    };
                    if (clip.w > 0.0F && std::abs(clip.z) <= clip.w) {
                        nameplates.push_back({glm::vec2{clip} / clip.w, driver_names[slot], clip.w,
                                              slot == focused_slot, livery(slot), eliminated(slot)});
                    }
                };
                project_helmet(world.vehicle, focused_slot);
                for (std::size_t index = 0; index < opponents.size(); ++index) {
                    project_helmet(opponents[index], index + (index >= focused_slot));
                }
                std::stable_sort(nameplates.begin(), nameplates.end(), [](const auto &a, const auto &b) {
                    return a.focused != b.focused ? a.focused : a.depth < b.depth;
                });
            }
            view.hud.draw(width, height, *race, *view.track_settings.path, world.vehicle, opponents,
                          circuit_overview, focused_driver, nameplates, training, trackside_name, standings, focused_slot);
        }

        glfwSwapBuffers(window);
    }

    void SimulationWindow::close() noexcept {
        impl_.reset();
    }
}
