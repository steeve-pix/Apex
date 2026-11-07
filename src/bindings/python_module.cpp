#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "math/vec2.hpp"
#include "simulation/body.hpp"
#include "simulation/track.hpp"
#include "simulation/vehicle.hpp"
#include "simulation/world.hpp"
#include "simulation/sensors.hpp"

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
            .def_readwrite("linear_drag", &MotionSettings::linear_drag);
    py::class_<VehicleSettings>(module, "VehicleSettings")
            .def(py::init<>())
            .def_readwrite("motion", &VehicleSettings::motion)
            .def_readwrite("wheelbase", &VehicleSettings::wheelbase)
            .def_readwrite("max_steering_angle", &VehicleSettings::max_steering_angle)
            .def_readwrite("lateral_grip", &VehicleSettings::lateral_grip);

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

    py::class_<RayReading>(module, "RayReading")
            .def_readwrite("distance", &RayReading::distance)
            .def_readwrite("hit", &RayReading::hit);

    py::class_<SensorSettings>(module, "SensorSettings")
            .def(py::init<>())
            .def_readwrite("angle_offset", &SensorSettings::angle_offset)
            .def_readwrite("max_distance", &SensorSettings::max_distance);


    module.def("make_initial_world", &apex::simulation::make_initial_world, py::arg("settings"));
    module.def("step_world", &apex::simulation::step_world, py::arg("world"), py::arg("controls"), py::arg("settings"),
               py::arg("dt"));
    module.def("sense_track", &apex::simulation::sense_track, py::arg("vehicle"), py::arg("track"), py::arg("sensors"));
    module.def("is_on_track", &apex::simulation::is_on_track, py::arg("position"), py::arg("track"));
}
