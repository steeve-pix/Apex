#pragma once

#include "simulation/track.hpp"

namespace apex::simulation {
    struct DriverSettings {
        double target_speed{0.30};
        double speed_gain{4.0};
        double heading_gain{1.5};
        double centerline_gain{3.0};
    };

    [[nodiscard]] VehicleControls drive_automatically(
        const TrackObservation &observation, const VehicleSettings &vehicle, const TrackSettings &track,
        const DriverSettings &driver);
}
