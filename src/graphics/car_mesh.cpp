#include "graphics/car_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <numbers>
#include <span>
#include <stdexcept>
#include <vector>

#include <glm/geometric.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

namespace apex::graphics {
    namespace {
        struct Vertex {
            glm::vec3 position;
            glm::vec3 color;
            glm::vec3 normal;
            // Use the default OpenGL attribute value for body vertices.
            glm::vec4 wheel_pivot{0,0,0,1};
        };

        struct Profile {
            float station;
            float center;
            float height;
            float width;
            float depth;
        };

        constexpr glm::vec3 paint{0.85F, 0.06F, 0.04F};
        constexpr glm::vec3 pearl{0.93F, 0.875F, 0.75F};
        constexpr glm::vec3 helmet_shell{0.72F, 0.72F, 0.69F};
        constexpr glm::vec3 visor{0.035F, 0.059F, 0.085F};
        constexpr glm::vec3 carbon{0.055F, 0.066F, 0.079F};
        constexpr glm::vec3 inlet{0.019F, 0.026F, 0.033F};
        constexpr glm::vec3 alloy{0.34F, 0.39F, 0.43F};
        constexpr float tau = 2.0F * std::numbers::pi_v<float>;

        void quad(std::vector<Vertex> &mesh, Vertex a, Vertex b, Vertex c, Vertex d) {
            mesh.insert(mesh.end(), {a, b, c, a, c, d});
        }

        enum class Section { body, sidepod, airfoil, round };

        std::vector<Profile> refine_profiles(std::span<const Profile> profiles) {
            constexpr int subdivisions = 3;
            std::vector<Profile> sections;
            sections.reserve((profiles.size() - 1) * subdivisions + 1);

            // Match the slope at each station without adding bulges beyond the authored sections.
            const auto slope = [&](std::size_t i, float Profile::*field) {
                const auto gradient = [&](std::size_t segment) {
                    return (profiles[segment + 1].*field - profiles[segment].*field)
                           / (profiles[segment + 1].station - profiles[segment].station);
                };
                if (i == 0) return gradient(0);
                if (i + 1 == profiles.size()) return gradient(i - 1);
                const float before = gradient(i - 1);
                const float after = gradient(i);
                if (before * after <= 0.0F) return 0.0F;
                const float left = profiles[i].station - profiles[i - 1].station;
                const float right = profiles[i + 1].station - profiles[i].station;
                const float w1 = 2 * right + left;
                const float w2 = right + 2 * left;
                return (w1 + w2) / (w1 / before + w2 / after);
            };
            for (std::size_t i = 0; i + 1 < profiles.size(); ++i) {
                const auto &a = profiles[i];
                const auto &b = profiles[i + 1];
                const float span = b.station - a.station;
                for (int step = 0; step < subdivisions; ++step) {
                    const float t = static_cast<float>(step) / subdivisions;
                    const float t2 = t * t;
                    const auto curve = [&](float Profile::*field) {
                        return std::lerp(a.*field, b.*field, t2 * (3.0F - 2.0F * t))
                               + span * (t * (1 - t) * (1 - t) * slope(i, field)
                                         - t2 * (1 - t) * slope(i + 1, field));
                    };
                    sections.push_back({std::lerp(a.station, b.station, t),
                                        curve(&Profile::center), curve(&Profile::height),
                                        curve(&Profile::width), curve(&Profile::depth)});
                }
            }
            sections.push_back(profiles.back());
            return sections;
        }

        // Body sections have broad faces with softened shoulders. Wing sections taper at the trailing edge.
        void loft(std::vector<Vertex> &mesh, std::initializer_list<Profile> profiles,
                  glm::vec3 color, Section shape = Section::body, bool cream_crown = false) {
            std::span<const Profile> sections{profiles.begin(), profiles.size()};
            const bool wing = shape == Section::airfoil;
            std::vector<Profile> refined;
            if (!wing && sections.size() >= 3) {
                refined = refine_profiles(sections);
                sections = refined;
            }
            constexpr int sides = 32;
            const auto point = [shape, wing](const Profile &p, float angle) {
                const float chord = std::cos(angle);
                const float vertical = std::sin(angle);
                const auto shoulder = [](float value) {
                    return std::copysign(std::pow(std::abs(value), 0.61F), value);
                };
                if (wing) {
                    const float thickness = std::lerp(0.34F, 1.0F, (chord + 1.0F) * 0.5F);
                    return glm::vec3{p.center + p.width * chord, p.station,
                                     p.height + p.depth * (vertical * thickness - 0.38F * chord)};
                }
                if (shape == Section::sidepod) {
                    const float side = std::copysign(1.0F, p.center);
                    const float outboard = side * chord;
                    // Move the lower shoulder inward to expose the floor beneath the inlet.
                    const float undercut = std::max(-vertical, 0.0F);
                    const float width = p.width * (1.0F - 0.38F * undercut);
                    const float shoulder_height = vertical - 0.22F * outboard;
                    return glm::vec3{p.station,
                                     p.center + width * shoulder(chord) - side * p.width * undercut * 0.32F,
                                     p.height + p.depth * shoulder_height};
                }
                const bool squared = shape == Section::body;
                return glm::vec3{p.station, p.center + p.width * (squared ? shoulder(chord) : chord),
                                 p.height + p.depth * (squared ? shoulder(vertical) : vertical)};
            };
            const auto vertex = [&](std::size_t ring, int side, glm::vec3 tint) {
                const float angle = side * tau / sides;
                const auto &section = sections[ring];
                const auto fore_aft = point(sections[std::min(ring + 1, sections.size() - 1)], angle)
                                    - point(sections[ring > 0 ? ring - 1 : 0], angle);
                const auto around = point(section, angle + 0.01F) - point(section, angle - 0.01F);
                const auto normal = glm::normalize(wing ? glm::cross(fore_aft, around)
                                                        : glm::cross(around, fore_aft));
                return Vertex{point(section, angle), tint, normal};
            };
            for (std::size_t ring = 0; ring + 1 < sections.size(); ++ring) {
                for (int side = 0; side < sides; ++side) {
                    // A narrow center stripe on the body and mirrored cream flashes on the sidepods.
                    const int band = shape == Section::sidepod && sections[ring].center < 0.0F
                                     ? sides / 2 - 1 - side : side;
                    const bool cream = wing ? band >= 1 && band < 6
                                      : shape == Section::sidepod ? band >= 3 && band < 6
                                                                 : band >= 6 && band < 10;
                    const auto tint = cream_crown && cream ? pearl : color;
                    quad(mesh, vertex(ring, side, tint), vertex(ring, side + 1, tint),
                         vertex(ring + 1, side + 1, tint), vertex(ring + 1, side, tint));
                }
            }
            for (const std::size_t end : {std::size_t{0}, sections.size() - 1}) {
                const auto &section = sections[end];
                const float sign = end == 0 ? -1.0F : 1.0F;
                const auto normal = wing ? glm::vec3{0, sign, 0} : glm::vec3{sign, 0, 0};
                const auto center = (point(section, 0) + point(section, tau * .5F)) * .5F;
                for (int side = 0; side < sides; ++side) {
                    mesh.insert(mesh.end(), {{center, color, normal},
                        {point(section, side * tau / sides), color, normal},
                        {point(section, (side + 1) * tau / sides), color, normal}});
                }
            }
        }

        // Extrude a thin panel for crisp aero edges rather than rounding it into a tube.
        void plate(std::vector<Vertex> &mesh, std::initializer_list<glm::vec3> outline,
                   glm::vec3 thickness, glm::vec3 color) {
            const std::span points{outline.begin(), outline.end()};
            const auto winding = glm::normalize(glm::cross(points[1] - points[0], points[2] - points[0]));
            const bool reversed = glm::dot(winding, thickness) < 0.0F;
            const auto front = reversed ? -winding : winding;
            for (std::size_t i = 1; i + 1 < points.size(); ++i) {
                mesh.insert(mesh.end(), {{points[0] + thickness, color, front},
                    {points[i] + thickness, color, front}, {points[i+1] + thickness, color, front},
                    {points[0], color, -front}, {points[i+1], color, -front}, {points[i], color, -front}});
            }
            for (std::size_t i = 0; i < points.size(); ++i) {
                const auto a = points[i];
                const auto b = points[(i + 1) % points.size()];
                const auto edge_normal = glm::normalize(glm::cross(b - a, thickness));
                const auto normal = reversed ? -edge_normal : edge_normal;
                quad(mesh, {a,color,normal}, {b,color,normal},
                     {b+thickness,color,normal}, {a+thickness,color,normal});
            }
        }

        void link(std::vector<Vertex> &mesh, glm::vec3 a, glm::vec3 b, float radius, glm::vec3 color) {
            const auto direction = glm::normalize(b - a);
            const auto reference = std::abs(direction.z) > .85F ? glm::vec3{0,1,0} : glm::vec3{0,0,1};
            const auto u = glm::normalize(glm::cross(direction, reference));
            const auto v = glm::cross(direction, u);
            for (int side = 0; side < 10; ++side) {
                const auto normal_at = [&](int i) { return u * std::cos(i * tau / 10) + v * std::sin(i * tau / 10); };
                const auto n = normal_at(side), m = normal_at(side + 1);
                quad(mesh, {a+n*radius,color,n}, {a+m*radius,color,m},
                           {b+m*radius,color,m}, {b+n*radius,color,n});
            }
        }

        void wheel(std::vector<Vertex> &mesh, glm::vec3 center, float radius, float half_width, float side, bool front) {
            struct Ring { float lateral, radial; glm::vec3 color; };
            constexpr glm::vec3 rubber{.045F,.047F,.049F};
            constexpr glm::vec3 sidewall{.054F,.056F,.059F};
            constexpr glm::vec3 cover{.105F,.120F,.135F};
            constexpr glm::vec3 sidewall_ink{.69F,.64F,.54F};
            const Ring rings[]{
                {-.89F,.001F,inlet}, {-.89F,.65F,inlet}, {-.97F,.72F,sidewall},
                {-1.0F,.86F,sidewall}, {-.91F,.97F,rubber}, {-.68F,1.0F,rubber},
                {.68F,1.0F,rubber}, {.91F,.97F,rubber}, {1.0F,.86F,sidewall},
                {.994F,.822F,sidewall}, {.985F,.763F,sidewall},
                {.98F,.72F,sidewall}, {1.02F,.69F,cover}, {1.02F,.63F,cover},
                {.96F,.59F,carbon}, {.97F,.22F,cover}, {1.04F,.15F,alloy}, {1.04F,.001F,carbon},
            };
            const glm::vec4 pivot{center, front ? 3.0F : 2.0F};
            const auto sample = [&](std::size_t row, float angle, glm::vec3 color) {
                const auto &ring = rings[row];
                const auto &prev = rings[row > 0 ? row - 1 : 0];
                const auto &next = rings[std::min(row + 1, std::size(rings) - 1)];
                const glm::vec3 radial{std::cos(angle), 0, std::sin(angle)};
                const glm::vec3 axle{0,side,0};
                const auto normal = glm::normalize(radial * ((next.lateral - prev.lateral) * half_width)
                                                  - axle * ((next.radial - prev.radial) * radius));
                return Vertex{center + axle * (ring.lateral * half_width) + radial * (ring.radial * radius),
                              color, normal, pivot};
            };
            constexpr int segments = 48;
            for (std::size_t row = 0; row + 1 < std::size(rings); ++row) {
                for (int i = 0; i < segments; ++i) {
                    const float a = i * tau / segments, b = (i + 1) * tau / segments;
                    const bool stencil = row == 9 && ((i >= 9 && i < 15) || (i >= 34 && i < 36));
                    const bool wheel_slot = row == 14 && i % 6 == 0;
                    const auto color = stencil ? sidewall_ink : wheel_slot ? inlet : rings[row].color;
                    quad(mesh, sample(row,a,color), sample(row,b,color),
                         sample(row+1,b,color), sample(row+1,a,color));
                }
            }
        }
    }

    CarMesh::CarMesh() {
        std::vector<Vertex> vertices;

        // Carbon floor, narrow rear coke-bottle section and a drooping nose.
        loft(vertices, {{-.153F,0,.009F,.024F,.002F},{-.118F,0,.006F,.032F,.002F},
                        {-.084F,0,.006F,.044F,.002F},{-.033F,0,.006F,.048F,.0017F},
                        {.018F,0,.006F,.049F,.0017F},{.052F,0,.008F,.028F,.0016F},
                        {.069F,0,.010F,.013F,.0016F}}, carbon);
        loft(vertices, {{-.150F,0,.018F,.004F,.003F},{-.128F,0,.021F,.010F,.006F},
                        {-.095F,0,.024F,.014F,.010F},{-.058F,0,.025F,.017F,.014F},
                        {-.024F,0,.027F,.021F,.015F},{.021F,0,.028F,.019F,.014F},
                        {.049F,0,.027F,.014F,.009F},{.083F,0,.023F,.009F,.005F},
                        {.113F,0,.019F,.0065F,.0037F},{.146F,0,.014F,.006F,.0024F},
                        {.165F,0,.012F,.005F,.0017F},{.172F,0,.012F,.004F,.0015F}}, paint, Section::body, true);
        // The lower tub joins the floor to the nose and carries the front suspension pickups.
        loft(vertices, {{-.134F,0,.013F,.009F,.006F},{-.106F,0,.015F,.015F,.009F},
                        {-.047F,0,.016F,.019F,.011F},{.026F,0,.016F,.015F,.010F},
                        {.065F,0,.015F,.010F,.007F},{.083F,0,.016F,.008F,.007F},
                        {.114F,0,.016F,.0065F,.006F},{.128F,0,.016F,.004F,.003F}}, carbon);
        loft(vertices, {{-.143F,0,.020F,.003F,.002F},{-.119F,0,.026F,.006F,.006F},
                        {-.092F,0,.032F,.010F,.010F},{-.071F,0,.039F,.011F,.013F},
                        {-.049F,0,.044F,.009F,.017F},{-.035F,0,.047F,.007F,.017F}}, paint, Section::body, true);
        loft(vertices, {{-.042F,0,.059F,.0068F,.0052F},{-.036F,0,.060F,.0075F,.0052F},
                        {-.032F,0,.060F,.0075F,.0047F}}, inlet);
        link(vertices, {-.040F,0,.062F}, {-.040F,0,.068F}, .0017F, carbon);
        loft(vertices, {{-.043F,0,.067F,.007F,.001F},{-.038F,0,.068F,.008F,.001F}}, carbon);

        // Cockpit rim, driver helmet, and the rounded halo.
        loft(vertices, {{-.040F,0,.037F,.012F,.009F},{-.024F,0,.043F,.0155F,.009F},
                        {.001F,0,.043F,.015F,.010F},{.024F,0,.039F,.010F,.010F},
                        {.038F,0,.035F,.007F,.006F}}, paint);
        loft(vertices, {{-.037F,0,.050F,.008F,.003F},{-.023F,0,.052F,.017F,.004F},
                        {.001F,0,.051F,.016F,.004F},{.024F,0,.048F,.008F,.002F}}, carbon);
        loft(vertices, {{-.025F,0,.053F,.0015F,.0025F},{-.022F,0,.0535F,.006F,.0055F},
                        {-.016F,0,.0545F,.0072F,.0070F},{-.010F,0,.0535F,.006F,.0059F},
                        {-.007F,0,.052F,.002F,.0024F}}, helmet_shell, Section::round);
        loft(vertices, {{-.017F,0,.054F,.0073F,.0024F},{-.012F,0,.054F,.0070F,.0026F},
                        {-.008F,0,.0533F,.0058F,.0022F},{-.006F,0,.0527F,.0027F,.0015F}}, visor, Section::round);
        for (float side : {-1.0F, 1.0F}) {
            // High radiator inlet over an undercut, with the sidepod tapering into the floor.
            loft(vertices, {{-.128F,side*.013F,.012F,.0016F,.0017F},{-.105F,side*.017F,.014F,.0044F,.0032F},
                            {-.071F,side*.023F,.018F,.009F,.0046F},{-.038F,side*.029F,.022F,.013F,.006F},
                            {-.010F,side*.030F,.025F,.0145F,.006F},{.018F,side*.030F,.028F,.014F,.006F},
                            {.041F,side*.027F,.032F,.012F,.0045F}}, paint, Section::sidepod, true);
            // The lower sidepod wall reaches the tub while leaving the outer undercut open.
            loft(vertices, {{-.121F,side*.013F,.010F,.004F,.004F},
                            {-.089F,side*.017F,.011F,.008F,.006F},
                            {-.041F,side*.020F,.014F,.012F,.009F},
                            {.009F,side*.020F,.017F,.012F,.011F},
                            {.035F,side*.020F,.022F,.010F,.009F},
                            {.040F,side*.020F,.029F,.006F,.004F}}, carbon, Section::sidepod);
            plate(vertices, {{.0407F,side*.016F,.031F},{.0407F,side*.020F,.0356F},
                             {.0407F,side*.034F,.0347F},{.0407F,side*.038F,.031F},
                             {.0407F,side*.032F,.029F}}, {.0007F,0,0}, inlet);
            const float floor_edge = side * .043F;
            plate(vertices, {{.026F,floor_edge,.0065F},{.021F,floor_edge,.015F},
                             {-.011F,floor_edge,.012F},{-.049F,floor_edge,.009F},
                             {-.049F,floor_edge,.0065F}}, {0,side*.0012F,0}, carbon);
            link(vertices, {.013F,side*.0115F,.044F}, {.024F,side*.032F,.050F}, .00095F, carbon);
            loft(vertices, {{.021F,side*.032F,.050F,.0038F,.0015F},
                            {.028F,side*.032F,.050F,.0038F,.0015F}}, carbon, Section::round);
            link(vertices, {-.026F,side*.013F,.049F}, {-.022F,side*.0185F,.065F}, .0022F, carbon);
            loft(vertices, {{-.024F,side*.0185F,.065F,.0022F,.0021F},
                            {-.011F,side*.0187F,.066F,.0022F,.0021F},
                            {.003F,side*.017F,.065F,.0024F,.0021F},
                            {.015F,side*.010F,.063F,.0034F,.0021F},
                            {.021F,side*.0035F,.0595F,.0040F,.0021F},
                            {.023F,0,.058F,.0032F,.0022F}}, carbon, Section::round);

            for (float axle : {-.10F,.10F}) {
                const bool rear = axle < 0.0F;
                const float radius = rear ? rear_tire_radius : front_tire_radius;
                const float half_width = rear ? .0143F : .0109F;
                const glm::vec3 hub{axle,side*(rear ? .0490F : .0515F),radius};
                const glm::vec3 upright = hub - glm::vec3{0,side*(half_width + .0018F),0};
                const glm::vec3 upper_joint = upright + glm::vec3{0,0,.005F};
                const glm::vec3 lower_joint = upright - glm::vec3{0,0,.004F};
                const float pickup = side * (rear ? .008F : .0045F);
                const float upper_height = rear ? .023F : .021F;
                link(vertices, {axle-.020F,pickup,upper_height+.003F}, upper_joint, .0015F, carbon);
                link(vertices, {axle+.017F,pickup,upper_height}, upper_joint, .0015F, carbon);
                link(vertices, {axle-.017F,pickup,.013F}, lower_joint, .0014F, carbon);
                link(vertices, {axle+.013F,pickup,.013F}, lower_joint, .0014F, carbon);
                link(vertices, {axle+.006F,pickup,upper_height+.002F}, lower_joint, .00085F, alloy);
                link(vertices, lower_joint - glm::vec3{0,0,.0012F}, upper_joint + glm::vec3{0,0,.0012F},
                     .0022F, carbon);
                link(vertices, upright - glm::vec3{0,side*.001F,0}, hub, .0028F, carbon);
                if (rear) {
                    link(vertices, {axle,side*.010F,radius}, upright, .0012F, alloy);
                }
                wheel(vertices, hub, radius, half_width, side, !rear);
            }
        }
        link(vertices, {.023F,0,.058F}, {.036F,0,.039F}, .0021F, carbon);
        // Three front wing elements, stacked behind the leading edge.
        for (float side : {-1.0F,1.0F}) {
            plate(vertices, {{.157F,side*.003F,.013F},{.160F,side*.003F,.007F},
                             {.146F,side*.003F,.007F},{.144F,side*.003F,.014F}},
                  {0,side*.0015F,0}, carbon);
        }
        loft(vertices, {{-.064F,.152F,.009F,.012F,.0015F},{-.045F,.154F,.008F,.014F,.0018F},
                        {-.021F,.154F,.006F,.015F,.0017F},{0,.155F,.006F,.016F,.0018F},
                        {.021F,.154F,.006F,.015F,.0017F},{.045F,.154F,.008F,.014F,.0018F},
                        {.064F,.152F,.009F,.012F,.0015F}}, carbon, Section::airfoil, true);
        loft(vertices, {{-.064F,.144F,.014F,.009F,.0011F},{-.043F,.143F,.014F,.011F,.0016F},
                        {-.016F,.139F,.010F,.010F,.0014F},{0,.137F,.010F,.010F,.0014F},
                        {.016F,.139F,.010F,.010F,.0014F},{.043F,.143F,.014F,.011F,.0016F},
                        {.064F,.144F,.014F,.009F,.0011F}}, carbon, Section::airfoil);
        loft(vertices, {{-.064F,.137F,.019F,.006F,.0011F},{-.045F,.137F,.019F,.007F,.0013F},
                        {-.018F,.130F,.015F,.008F,.0011F},{0,.127F,.014F,.007F,.0010F},
                        {.018F,.130F,.015F,.008F,.0011F},{.045F,.137F,.019F,.007F,.0013F},
                        {.064F,.137F,.019F,.006F,.0011F}}, carbon, Section::airfoil);

        // The gearbox carries the rear wing pylon and the center of the lower beam wing.
        loft(vertices, {{-.153F,0,.018F,.006F,.004F},{-.144F,0,.018F,.008F,.008F},
                        {-.127F,0,.017F,.011F,.008F},{-.115F,0,.016F,.012F,.007F}}, carbon);
        loft(vertices, {{-.047F,-.140F,.055F,.012F,.0019F},{-.032F,-.143F,.052F,.014F,.0022F},
                        {0,-.145F,.050F,.014F,.0024F},{.032F,-.143F,.052F,.014F,.0022F},
                        {.047F,-.140F,.055F,.012F,.0019F}}, carbon, Section::airfoil);
        loft(vertices, {{-.047F,-.151F,.063F,.007F,.0011F},{-.030F,-.153F,.062F,.010F,.0015F},
                        {0,-.153F,.061F,.011F,.0016F},{.030F,-.153F,.062F,.010F,.0015F},
                        {.047F,-.151F,.063F,.007F,.0011F}}, carbon, Section::airfoil, true);
        loft(vertices, {{-.038F,-.139F,.024F,.006F,.0012F},{0,-.141F,.025F,.008F,.0014F},
                        {.038F,-.139F,.024F,.006F,.0012F}}, carbon, Section::airfoil);
        plate(vertices, {{-.132F,-.001F,.024F},{-.150F,-.001F,.053F},
                         {-.145F,-.001F,.058F},{-.126F,-.001F,.028F}}, {0,.002F,0}, carbon);
        plate(vertices, {{-.127F,-.0008F,.026F},{-.095F,-.0008F,.044F},
                         {-.050F,-.0008F,.064F},{-.044F,-.0008F,.051F}},
              {0,.0016F,0}, paint);
        for (float side : {-1.0F,1.0F}) {
            const float front = side * .0637F;
            plate(vertices, {{.167F,front,.008F},{.161F,front,.016F},{.140F,front,.024F},
                             {.127F,front,.022F},{.127F,front,.010F}}, {0,side*.0014F,0}, carbon);
            const float paint_panel = side * .065F;
            plate(vertices, {{.157F,paint_panel,.013F},{.143F,paint_panel,.020F},
                             {.130F,paint_panel,.020F},{.130F,paint_panel,.014F}},
                  {0,side*.0004F,0}, paint);
            const float rear = side * .0467F;
            plate(vertices, {{-.160F,rear,.045F},{-.160F,rear,.063F},{-.151F,rear,.066F},
                             {-.127F,rear,.059F},{-.130F,rear,.050F},{-.143F,rear,.045F}},
                  {0,side*.0014F,0}, paint);
            const float beam_support = side * .028F;
            plate(vertices, {{-.150F,beam_support,.015F},{-.143F,beam_support,.025F},
                             {-.136F,beam_support,.025F},{-.138F,beam_support,.013F}},
                  {0,-side*.0014F,0}, carbon);
        }

        // An open diffuser under the gearbox, with vertical channels visible from behind.
        plate(vertices, {{-.153F,-.028F,.016F},{-.115F,-.022F,.006F},
                         {-.115F,.022F,.006F},{-.153F,.028F,.016F}}, {0,0,.0017F}, carbon);
        for (float y : {-.024F,-.012F,.012F,.024F}) {
            plate(vertices, {{-.150F,y,.007F},{-.153F,y,.017F},
                             {-.119F,y,.009F},{-.124F,y,.005F}}, {0,.0011F,0}, carbon);
        }
        loft(vertices, {{-.155F,0,.020F,.0036F,.0028F},{-.148F,0,.020F,.0043F,.0030F}}, carbon);
        plate(vertices, {{-.1549F,-.0028F,.019F},{-.1549F,.0028F,.019F},
                         {-.1549F,.0028F,.0205F},{-.1549F,-.0028F,.0205F}},
              {-.0005F,0,0}, {1.0F,.52F,.38F});

        vertex_count_ = static_cast<GLsizei>(vertices.size());

        glGenVertexArrays(1, &vertex_array_);
        glGenBuffers(1, &vertex_buffer_);

        if (vertex_array_ == 0 || vertex_buffer_ == 0) {
            glDeleteBuffers(1, &vertex_buffer_);
            glDeleteVertexArrays(1, &vertex_array_);

            throw std::runtime_error(
                "Failed to create car mesh resources"
            );
        }

        glBindVertexArray(vertex_array_);
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
            vertices.data(),
            GL_STATIC_DRAW
        );

        glVertexAttribPointer(
            0, 3, GL_FLOAT, GL_FALSE,
            static_cast<GLsizei>(sizeof(Vertex)),
            nullptr
        );
        glEnableVertexAttribArray(0);

        glVertexAttribPointer(
            1, 3, GL_FLOAT, GL_FALSE,
            static_cast<GLsizei>(sizeof(Vertex)),
            reinterpret_cast<const void *>(offsetof(Vertex, color))
        );
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, normal)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                              reinterpret_cast<const void *>(offsetof(Vertex, wheel_pivot)));
        glEnableVertexAttribArray(4);

        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    CarMesh::~CarMesh() {
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteVertexArrays(1, &vertex_array_);
    }

    void CarMesh::draw() const {
        glBindVertexArray(vertex_array_);
        glDrawArrays(GL_TRIANGLES, 0, vertex_count_);
    }
}
