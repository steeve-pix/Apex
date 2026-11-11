#pragma once

#include "simulation/track_path.hpp"

namespace apex::simulation {
    [[nodiscard]] TrackPath make_apex_circuit(int samples_per_section = 16);
}
