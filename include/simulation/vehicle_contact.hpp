#pragma once

#include <optional>
#include <span>
#include "vehicle.hpp"

namespace apex::simulation {
    struct VehicleContact {
        math::Vec2 normal{};
        double penetration{};
    };

    // Compound top-down footprint, including wings and individually steered front tires.
    [[nodiscard]] std::optional<VehicleContact> vehicle_contact(
        const VehicleState &a, const VehicleState &b, double margin = 0.0);
    // Returns the normal closing speed before the equal-mass impact response.
    std::optional<double> resolve_vehicle_contact(VehicleState &a, VehicleState &b);
    void solve_vehicle_contacts(std::span<VehicleState *> cars);
}
