#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <stdexcept>

#include "math/vec2.hpp"
#include "simulation/body.hpp"
#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"
#include "simulation/world.hpp"
#include "simulation/sensors.hpp"
#include "graphics/simulation_window.hpp"
#include "simulation/race.hpp"
#ifdef __APPLE__
#include "platform/macos_application.hpp"
#endif
namespace py = pybind11;

PYBIND11_MODULE(apex_core, module) {
    using apex::math::Vec2;
    using apex::simulation::Body2D;
    using apex::simulation::VehicleControls;
    using apex::simulation::VehicleState;
    using apex::simulation::MotionSettings;
    using apex::simulation::VehicleSettings;
    using apex::simulation::TrackSettings;
    using apex::simulation::WorldSettings;
    using apex::simulation::WorldState;
    using apex::simulation::RayReading;
    using apex::simulation::SensorSettings;
    using apex::graphics::WindowInput;
    using apex::graphics::SimulationWindow;
    using apex::simulation::Race;
    using apex::simulation::RaceStatus;

    module.doc() = "Apex C++ simulation bindings";

    py::class_<Vec2>(module, "Vec2")
            .def(py::init<>())
            .def_readwrite("x", &Vec2::x)
            .def_readwrite("y", &Vec2::y);

    py::class_<Body2D>(module, "Body2D")

            .def(py::init<>())
            .def_readwrite("position", &Body2D::position)
            .def_readwrite("velocity", &Body2D::velocity);

    py::class_<VehicleState>(module, "VehicleState")
            .def(py::init<>())
            .def_readwrite("body", &VehicleState::body)
            .def_readwrite("heading", &VehicleState::heading);

    py::class_<VehicleControls>(module, "VehicleControls")
            .def(py::init<>())
            .def_readwrite("throttle", &VehicleControls::throttle)
            .def_readwrite("steering", &VehicleControls::steering);

    py::class_<MotionSettings>(module, "MotionSettings")
            .def(py::init<>())
            .def_readwrite(
                "acceleration_strength", &MotionSettings::acceleration_strength)
            .def_readwrite("linear_drag", &MotionSettings::linear_drag)
            .def_readwrite("quadratic_drag", &MotionSettings::quadratic_drag);
    py::class_<VehicleSettings>(module, "VehicleSettings")
            .def(py::init<>())
            .def_readwrite("motion", &VehicleSettings::motion)
            .def_readwrite("wheelbase", &VehicleSettings::wheelbase)
            .def_readwrite("max_steering_angle", &VehicleSettings::max_steering_angle)
            .def_readwrite("lateral_grip", &VehicleSettings::lateral_grip)
            .def_readwrite("braking_strength", &VehicleSettings::braking_strength)
            .def_readwrite("lateral_acceleration_limit", &VehicleSettings::lateral_acceleration_limit);

    py::class_<TrackSettings>(module, "TrackSettings")
            .def(py::init<>())
            .def_readwrite("centerline_radius", &TrackSettings::centerline_radius)
            .def_readwrite("road_width", &TrackSettings::road_width)
            .def_readwrite("off_road_drag", &TrackSettings::off_road_drag);

    py::class_<WorldSettings>(module, "WorldSettings")
            .def(py::init<>())
            .def_readwrite("vehicle", &WorldSettings::vehicle)
            .def_readwrite("track", &WorldSettings::track);

    py::class_<WorldState>(module, "WorldState")
            .def(py::init<>())
            .def_readwrite("vehicle", &WorldState::vehicle)
            .def_readwrite("elapsed_time", &WorldState::elapsed_time);

    py::class_<RaceStatus>(module, "RaceStatus")
            .def_readonly("on_track", &RaceStatus::on_track)
            .def_readonly("completed_laps", &RaceStatus::completed_laps)
        .def_readonly("last_lap", &RaceStatus::last_lap)
        .def_readonly("speed", &RaceStatus::speed);

    py::class_<Race>(module, "Race")
            .def(py::init<const WorldSettings &>(), py::arg("settings"))
            .def(py::init([](const WorldSettings &settings, int starting_slot) {
                return Race(settings, apex::simulation::SessionMode::time_trial,
                            starting_slot);
            }), py::arg("settings"), py::arg("starting_slot"))
            .def("step", &Race::step, py::arg("controls"), py::arg("automatic"), py::arg("dt"))
            .def_property_readonly("player", &Race::player, py::return_value_policy::copy)
            .def_property_readonly("status", &Race::status, py::return_value_policy::copy)
            .def_property_readonly("player_controls", &Race::player_controls);;

    module.def("make_race_settings", &apex::simulation::make_race_settings);
    module.def("track_progress",
               [](const Vec2 &position, const TrackSettings &track) {
                   if (!track.path) {
                       throw std::invalid_argument("Track progress requires a circuit");
                   }

                   const auto projection = track.path->project(position);
                   return py::make_tuple(
                       projection.distance_along_track,
                       track.path->length()
                   );
               },
               py::arg("position"), py::arg("track"));

    py::class_<RayReading>(module, "RayReading")
            .def_readwrite("distance", &RayReading::distance)
            .def_readwrite("hit", &RayReading::hit);

    py::class_<SensorSettings>(module, "SensorSettings")
            .def(py::init<>())
            .def_readwrite("angle_offset", &SensorSettings::angle_offset)
            .def_readwrite("max_distance", &SensorSettings::max_distance);


    py::class_<WindowInput>(module, "WindowInput")
            .def_readonly("controls", &WindowInput::controls)
            .def_readonly("close_requested", &WindowInput::close_requested)
            .def_readonly("reset_requested", &WindowInput::reset_requested)
            .def_readonly("pause_requested", &WindowInput::pause_requested)
            .def_readonly("continue_requested", &WindowInput::continue_requested)
            .def_readonly("autopilot_requested", &WindowInput::autopilot_requested)
            .def_readonly("car_selection_delta", &WindowInput::car_selection_delta)
            .def_readonly("follow_leader_requested", &WindowInput::follow_leader_requested);

    py::class_<SimulationWindow>(module, "SimulationWindow")
            .def(py::init<const TrackSettings &>(), py::arg("track"))
            .def("poll_events", &SimulationWindow::poll_events, py::arg("spectate") = false)
            .def("draw", [](SimulationWindow &window, const WorldState &world,
                            const SensorSettings &sensors, std::string_view status) {
                window.draw(world, sensors, status);
            }, py::arg("world"), py::arg("sensors"), py::arg("status"))
            .def("draw_race", [](SimulationWindow &window, const Race &race,
                                 const SensorSettings &sensors, bool automatic,
                                 std::string_view title) {
                     auto status = race.status();
                     status.automatic = automatic;

                     window.draw(race.player(), sensors, title,
                                 race.opponents(), &status);
                 }, py::arg("race"), py::arg("sensors"),
                 py::arg("automatic") = false,
                 py::arg("title") = "Sunset Circuit")
            .def("draw_population",
                 [](SimulationWindow &window, const Race &leader,
                    const SensorSettings &sensors,
                    const std::vector<VehicleState> &other_cars,
                    std::string_view title, std::size_t focused_slot, std::string_view focused_driver,
                    const std::vector<std::string> &driver_names,
                    std::optional<std::tuple<std::size_t, std::size_t, double>> telemetry,
                    const std::vector<std::size_t> &eliminated_slots,
                    const std::vector<std::tuple<std::size_t, double, std::optional<double>>> &standings,
                    std::optional<std::tuple<std::size_t, double, bool>> recap) {
                     if (focused_slot >= apex::simulation::grid_size) {
                         throw std::out_of_range("Focused grid slot is out of range");
                     }
                     auto status = leader.status();
                     std::vector<apex::graphics::TrainingStanding> rows;
                     rows.reserve(standings.size());
                     for (const auto &[slot, progress, lap_time] : standings) {
                         rows.push_back({slot, progress, lap_time});
                     }
                     std::optional<apex::graphics::TrainingTelemetry> training;
                     if (telemetry) {
                         const auto [generation, active_cars, best_distance] = *telemetry;
                         training.emplace(generation, active_cars, best_distance, rows);
                         if (recap) {
                             const auto [winner, remaining_seconds, held] = *recap;
                             training->recap.emplace(winner, remaining_seconds, held);
                         }
                     }
                     window.draw(leader.player(), sensors, title, other_cars, &status, focused_slot, focused_driver,
                                 driver_names, training ? &*training : nullptr, eliminated_slots);
                 },
                 py::arg("leader"), py::arg("sensors"),
                 py::arg("other_cars"), py::arg("title"),
                 py::arg("focused_slot") = apex::simulation::player_slot, py::arg("focused_driver") = "",
                 py::arg("driver_names") = std::vector<std::string>{}, py::arg("telemetry") = py::none(),
                 py::arg("eliminated_slots") = std::vector<std::size_t>{},
                 py::arg("standings") = std::vector<std::tuple<std::size_t, double, std::optional<double>>>{},
                 py::arg("recap") = py::none())
            .def("close", &SimulationWindow::close);

    module.def("make_initial_world", &apex::simulation::make_initial_world, py::arg("settings"));
    module.def("step_world", &apex::simulation::step_world, py::arg("world"), py::arg("controls"), py::arg("settings"),
               py::arg("dt"));
    module.def("sense_track", &apex::simulation::sense_track, py::arg("vehicle"), py::arg("track"), py::arg("sensors"));
    module.def("is_on_track", &apex::simulation::is_on_track, py::arg("position"), py::arg("track"));

#ifdef __APPLE__
    module.def("set_application_icon", &apex::platform::set_application_icon, py::arg("path"));
#endif
}
