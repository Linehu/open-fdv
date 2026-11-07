#ifndef FDV_VEHICLE_BASE_H
#define FDV_VEHICLE_BASE_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/binder_common.hpp>

#include <cmath>
#include <type_traits>

#include "fd_core.h"
#include "fdv_vehicle_sim.h"

namespace fdv {
// C++20 concept: an arithmetic scalar (used to type-check speed storage).
template <typename T>
concept ArithmeticScalar = std::is_arithmetic_v<T>;

// Thin Godot-facing shell around the arcade scooter simulation.
//
// The rules themselves live in fd_core (shared/include/fd_core.h), which the
// Unity build reaches through P/Invoke. This class is the Godot half of the
// same contract: it owns an FdVehicle, forwards the step to fd_vehicle_step(),
// and exposes the tuning as ClassDB properties so designers can change handling
// without recompiling. The shared infrastructure (fixed-step accumulation,
// collision, replication, transform mirroring) is a composed VehicleSim, not an
// inherited base -- ManualCar reuses the same component without an is-a chain.
class VehicleBase : public godot::Node3D {
    GDCLASS(VehicleBase, godot::Node3D)

    // km/h -> m/s conversion factor.
    static constexpr double KMH_TO_MPS = 1.0 / 3.6;

    double speed_kmh = 0.0;

    // Simulation state and configuration.
    FdVehicle vehicle_state{};
    int32_t preset = FD_HANDLING_WILD;
    int32_t riders = 1;
    double throttle = 0.0;
    double steer = 0.0;
    bool steer_assist = false;

    // Shared infrastructure: fixed-step, collision, replication, transform.
    VehicleSim sim;

    static_assert(KMH_TO_MPS > 0.0, "KMH_TO_MPS must be positive");
    static_assert(ArithmeticScalar<decltype(speed_kmh)>, "speed_kmh must be an arithmetic scalar");

protected:
    static void _bind_methods();

public:
    VehicleBase() = default;
    ~VehicleBase() = default;

    auto _ready() -> void override;
    auto _physics_process(double p_delta) -> void override;

    auto set_speed_kmh(double p_speed_kmh) -> void;
    [[nodiscard]] auto get_speed_kmh() const -> double;

    // Exposed to GDScript for smoke-testing the binding.
    [[nodiscard]] auto speed_mps() const -> double;

    auto set_preset(int32_t p_preset) -> void;
    [[nodiscard]] auto get_preset() const -> int32_t;

    auto set_riders(int32_t p_riders) -> void;
    [[nodiscard]] auto get_riders() const -> int32_t;

    auto set_throttle(double p_throttle) -> void;
    [[nodiscard]] auto get_throttle() const -> double;

    auto set_steer(double p_steer) -> void;
    [[nodiscard]] auto get_steer() const -> double;

    // 0 = upright, 1 = skidding, 2 = flipping, 3 = wrecked.
    [[nodiscard]] auto get_vehicle_state() const -> int32_t;
    [[nodiscard]] auto get_lean() const -> double;

    // One fixed step. Called by _physics_process at FD_VEH_FIXED_DT; exposed so
    // a test or tool script can step deterministically without a scene.
    [[nodiscard]] auto step(double p_delta) -> bool;

    // Reset to a clean upright state at the current transform.
    auto reset_vehicle() -> void;

    // Client side: take the host's state verbatim. Simulation is suspended
    // until reset_vehicle() is called again.
    auto adopt_authoritative(const godot::Dictionary &p_state) -> void;

    // True while the node is following a host instead of simulating.
    [[nodiscard]] auto is_remote() const -> bool;

    // Hand control back to the local simulation after adopt_authoritative().
    // Used by callers that adopt a pose only to place the vehicle, and by any
    // caller that takes ownership back from a host.
    auto resume_local_simulation() -> void;

    // Static world collision. boxes is an Array of Dictionaries, each
    // {pos: Vector3, size: Vector3, kind: int}.
    auto set_colliders(const godot::Array &p_boxes) -> void;
    [[nodiscard]] auto get_collider_count() const -> int64_t;

    // Impact speed from the most recent step, and whether it hit anything.
    [[nodiscard]] auto get_last_impact() const -> double;
    [[nodiscard]] auto get_had_collision() const -> bool;
};

} // namespace fdv

#endif // FDV_VEHICLE_BASE_H
