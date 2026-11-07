#ifndef FDV_MANUAL_CAR_H
#define FDV_MANUAL_CAR_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>

#include "fd_core.h"
#include "fdv_vehicle_sim.h"

namespace fdv {

// Godot-facing shell around the manual-transmission car.
//
// The rules live in the C core (fd_car_step / fd_car_collide); this class owns
// an FdCar + FdCarTuning and the shared VehicleSim infrastructure, and exposes
// the tunables as ClassDB properties so GDScript can configure the car (max
// torque, gear ratios, mouse sensitivity) without recompiling. Faults are
// read-only: the core reports engine failure and gear grind as edge-triggered
// events, and there is deliberately no repair primitive here -- the business
// layer decides what a fault costs. Composition over inheritance: ManualCar is
// a sibling of VehicleBase, both composing VehicleSim, not a subclass of it.
class ManualCar : public godot::Node3D {
    GDCLASS(ManualCar, godot::Node3D)

    static constexpr double KMH_TO_MPS = 1.0 / 3.6;

    // C core state (includes the active tuning in car_state.tuning).
    FdCar car_state{};

    // Input-side steer sensitivity (mouse pixels -> steer). Godot-facing, not in
    // the C core: Unity uses a different input device.
    double mouse_dpi = 1.0;

    // Driver inputs, consumed by step().
    double throttle = 0.0;
    double steer = 0.0;
    double clutch = 0.0;
    int32_t requested_gear = FD_GEAR_NEUTRAL;

    double speed_kmh = 0.0;

    // Shared infrastructure: fixed-step, collision, transform mirroring.
    VehicleSim sim;

protected:
    static void _bind_methods();

public:
    ManualCar();
    ~ManualCar() = default;

    auto _ready() -> void override;
    auto _physics_process(double p_delta) -> void override;

    // --- config (GDScript-configurable primitives) ---
    auto set_max_torque(double p_torque) -> void;
    [[nodiscard]] auto get_max_torque() const -> double;

    auto set_gear_ratios(const godot::PackedFloat32Array &p_ratios) -> void;
    [[nodiscard]] auto get_gear_ratios() const -> godot::PackedFloat32Array;

    auto set_gear_count(int32_t p_count) -> void;
    [[nodiscard]] auto get_gear_count() const -> int32_t;

    auto set_final_drive_ratio(double p_ratio) -> void;
    [[nodiscard]] auto get_final_drive_ratio() const -> double;

    auto set_idle_rpm(double p_rpm) -> void;
    [[nodiscard]] auto get_idle_rpm() const -> double;

    auto set_redline_rpm(double p_rpm) -> void;
    [[nodiscard]] auto get_redline_rpm() const -> double;

    auto set_mouse_dpi(double p_dpi) -> void;
    [[nodiscard]] auto get_mouse_dpi() const -> double;

    // --- driver input ---
    auto set_throttle(double p_throttle) -> void;
    [[nodiscard]] auto get_throttle() const -> double;

    auto set_steer(double p_steer) -> void;
    [[nodiscard]] auto get_steer() const -> double;

    auto set_clutch(double p_clutch) -> void;
    [[nodiscard]] auto get_clutch() const -> double;

    // Request a gear change (1..5, 0 neutral, -1 reverse). The core refuses the
    // shift and reports a grind event if the clutch is still engaged.
    auto shift_gear(int32_t p_gear) -> void;

    // --- read-only state ---
    [[nodiscard]] auto get_rpm() const -> double;
    [[nodiscard]] auto get_gear() const -> int32_t;
    [[nodiscard]] auto get_speed_kmh() const -> double;
    [[nodiscard]] auto is_engine_failed() const -> bool;
    [[nodiscard]] auto get_events() const -> int32_t;

    [[nodiscard]] auto step(double p_delta) -> bool;
    auto reset_car() -> void;

    // Place the car at a world pose and stop it there, keeping the tuning.
    // Used when the crew swaps vehicles: the car must appear where they got
    // out, not wherever it was last parked.
    auto adopt_pose(const godot::Vector3 &p_position, double p_heading) -> void;

    // Hot refresh: apply a full tuning without touching runtime state (speed,
    // rpm, steering, position, gear). The tuning struct is rewritten in place
    // -- no allocation or zero-initialisation -- so the car keeps driving
    // through the change. The "out of control" effect changes the feel mid-
    // drive. Script-agnostic: the caller hands in the values; re-reading the
    // config script is the business layer's job.
    auto apply_tuning(const godot::Dictionary &p_tuning) -> void;

    // Static world collision, delegated to the shared component.
    auto set_colliders(const godot::Array &p_boxes) -> void;
    [[nodiscard]] auto get_collider_count() const -> int64_t;
    [[nodiscard]] auto get_last_impact() const -> double;
    [[nodiscard]] auto get_had_collision() const -> bool;
};

} // namespace fdv

#endif // FDV_MANUAL_CAR_H
