#include "fdv_manual_car.h"

#include <godot_cpp/core/class_db.hpp>

auto fdv::ManualCar::_bind_methods() -> void {
    godot::ClassDB::bind_method(godot::D_METHOD("set_max_torque", "torque"), &ManualCar::set_max_torque);
    godot::ClassDB::bind_method(godot::D_METHOD("get_max_torque"), &ManualCar::get_max_torque);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "max_torque"), "set_max_torque", "get_max_torque");

    godot::ClassDB::bind_method(godot::D_METHOD("set_gear_ratios", "ratios"), &ManualCar::set_gear_ratios);
    godot::ClassDB::bind_method(godot::D_METHOD("get_gear_ratios"), &ManualCar::get_gear_ratios);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::PACKED_FLOAT32_ARRAY, "gear_ratios"), "set_gear_ratios", "get_gear_ratios");

    godot::ClassDB::bind_method(godot::D_METHOD("set_gear_count", "count"), &ManualCar::set_gear_count);
    godot::ClassDB::bind_method(godot::D_METHOD("get_gear_count"), &ManualCar::get_gear_count);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::INT, "gear_count", godot::PROPERTY_HINT_RANGE, "1,5,1"), "set_gear_count", "get_gear_count");

    godot::ClassDB::bind_method(godot::D_METHOD("set_final_drive_ratio", "ratio"), &ManualCar::set_final_drive_ratio);
    godot::ClassDB::bind_method(godot::D_METHOD("get_final_drive_ratio"), &ManualCar::get_final_drive_ratio);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "final_drive_ratio"), "set_final_drive_ratio", "get_final_drive_ratio");

    godot::ClassDB::bind_method(godot::D_METHOD("set_idle_rpm", "rpm"), &ManualCar::set_idle_rpm);
    godot::ClassDB::bind_method(godot::D_METHOD("get_idle_rpm"), &ManualCar::get_idle_rpm);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "idle_rpm"), "set_idle_rpm", "get_idle_rpm");

    godot::ClassDB::bind_method(godot::D_METHOD("set_redline_rpm", "rpm"), &ManualCar::set_redline_rpm);
    godot::ClassDB::bind_method(godot::D_METHOD("get_redline_rpm"), &ManualCar::get_redline_rpm);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "redline_rpm"), "set_redline_rpm", "get_redline_rpm");

    godot::ClassDB::bind_method(godot::D_METHOD("set_mouse_dpi", "dpi"), &ManualCar::set_mouse_dpi);
    godot::ClassDB::bind_method(godot::D_METHOD("get_mouse_dpi"), &ManualCar::get_mouse_dpi);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "mouse_dpi"), "set_mouse_dpi", "get_mouse_dpi");

    godot::ClassDB::bind_method(godot::D_METHOD("set_throttle", "throttle"), &ManualCar::set_throttle);
    godot::ClassDB::bind_method(godot::D_METHOD("get_throttle"), &ManualCar::get_throttle);
    godot::ClassDB::bind_method(godot::D_METHOD("set_steer", "steer"), &ManualCar::set_steer);
    godot::ClassDB::bind_method(godot::D_METHOD("get_steer"), &ManualCar::get_steer);
    godot::ClassDB::bind_method(godot::D_METHOD("set_clutch", "clutch"), &ManualCar::set_clutch);
    godot::ClassDB::bind_method(godot::D_METHOD("get_clutch"), &ManualCar::get_clutch);
    godot::ClassDB::bind_method(godot::D_METHOD("shift_gear", "gear"), &ManualCar::shift_gear);

    godot::ClassDB::bind_method(godot::D_METHOD("get_rpm"), &ManualCar::get_rpm);
    godot::ClassDB::bind_method(godot::D_METHOD("get_gear"), &ManualCar::get_gear);
    godot::ClassDB::bind_method(godot::D_METHOD("get_speed_kmh"), &ManualCar::get_speed_kmh);
    godot::ClassDB::bind_method(godot::D_METHOD("is_engine_failed"), &ManualCar::is_engine_failed);
    godot::ClassDB::bind_method(godot::D_METHOD("get_events"), &ManualCar::get_events);
    godot::ClassDB::bind_method(godot::D_METHOD("restart_engine"), &ManualCar::restart_engine);

    godot::ClassDB::bind_method(godot::D_METHOD("step", "delta"), &ManualCar::step);
    godot::ClassDB::bind_method(godot::D_METHOD("reset_car"), &ManualCar::reset_car);
    godot::ClassDB::bind_method(godot::D_METHOD("apply_tuning", "tuning"), &ManualCar::apply_tuning);

    godot::ClassDB::bind_method(godot::D_METHOD("set_colliders", "boxes"), &ManualCar::set_colliders);
    godot::ClassDB::bind_method(godot::D_METHOD("get_collider_count"), &ManualCar::get_collider_count);
    godot::ClassDB::bind_method(godot::D_METHOD("get_last_impact"), &ManualCar::get_last_impact);
    godot::ClassDB::bind_method(godot::D_METHOD("get_had_collision"), &ManualCar::get_had_collision);
    godot::ClassDB::bind_method(godot::D_METHOD("adopt_pose", "position", "heading"),
                                &ManualCar::adopt_pose);
}

fdv::ManualCar::ManualCar() {
    fd_car_default_tuning(&car_state.tuning);
    // Matches reset_car(): a car handed to a player is in first gear, not
    // neutral, so pressing the throttle does something.
    car_state.gear = FD_GEAR_1;
    car_state.rpm = car_state.tuning.idle_rpm;
    requested_gear = FD_GEAR_1;
}

auto fdv::ManualCar::_ready() -> void {
    godot::Node3D::_ready(); // 父
    reset_car();             // 子
}

auto fdv::ManualCar::_physics_process(double p_delta) -> void {
    godot::Node3D::_physics_process(p_delta); // 父
    (void)step(p_delta);                       // 子
}

auto fdv::ManualCar::step(double p_delta) -> bool {
    if (sim.remote) {
        return false;
    }
    if (!sim.begin_step(p_delta)) {
        return false;
    }

    bool advanced = false;
    int guard = 0;
    while (sim.accum >= FD_VEH_FIXED_DT && guard++ < 8) {
        FdCarInput input{};
        input.throttle = static_cast<float>(throttle);
        input.steer = static_cast<float>(steer);
        input.clutch = static_cast<float>(clutch);
        input.requested_gear = requested_gear;

        if (fd_car_step(&car_state, &input, FD_VEH_FIXED_DT) != FD_OK) {
            break;
        }

        // The shift request is an edge, not a level: consume it after one step.
        requested_gear = car_state.gear;

        sim.resolve_collisions(&car_state, fd_car_collide);
        sim.consume_step();
        advanced = true;
    }

    if (advanced) {
        speed_kmh = car_state.speed * 3.6;
        sim.mirror_pose(this, car_state.x, car_state.z, car_state.heading);
    }
    return advanced;
}

auto fdv::ManualCar::set_colliders(const godot::Array &p_boxes) -> void {
    sim.colliders = p_boxes;
}

auto fdv::ManualCar::get_collider_count() const -> int64_t {
    return sim.colliders.size();
}

auto fdv::ManualCar::get_last_impact() const -> double {
    return sim.last_impact;
}

auto fdv::ManualCar::get_had_collision() const -> bool {
    return sim.had_collision;
}

auto fdv::ManualCar::reset_car() -> void {
    // Carry the tuning across the reset. Assigning a fresh FdCar zeroes the
    // whole struct, tuning included, so a car that had been configured (or
    // driven through a hot refresh) would come back with max_torque 0, idle_rpm
    // 0 and a redline of 0 -- an engine that can never produce torque and
    // reports redline the instant it turns over.
    const FdCarTuning keep = car_state.tuning;
    car_state = FdCar{};
    car_state.tuning = keep;
    car_state.x = static_cast<float>(get_global_position().x);
    car_state.z = static_cast<float>(get_global_position().z);
    // The node's rotation is heading + pi; see VehicleSim::kHeadingToNodeYaw.
    car_state.heading = static_cast<float>(get_global_rotation().y
                                           - VehicleSim::kHeadingToNodeYaw);
    /* First gear, not neutral. A car parked in neutral sits there revving when
     * the player presses the throttle, which reads as "the car is broken"
     * rather than "you need to select a gear": nobody gets into a car and
     * leaves it in neutral. Neutral is still reachable with N, and reverse
     * with R. */
    car_state.gear = FD_GEAR_1;
    car_state.rpm = car_state.tuning.idle_rpm;
    requested_gear = FD_GEAR_1;
    throttle = 0.0;
    steer = 0.0;
    clutch = 0.0;
    speed_kmh = 0.0;
    sim.accum = 0.0;
    sim.remote = false;
    sim.last_impact = 0.0;
    sim.had_collision = false;
    sim.mirror_pose(this, car_state.x, car_state.z, car_state.heading);
}

auto fdv::ManualCar::apply_tuning(const godot::Dictionary &p_tuning) -> void {
    // Build the next tuning as a partial override of the current one, then apply
    // it through the C ABI so the change is a single deterministic mutation the
    // replication protocol can sync. The runtime state is untouched.
    FdCarTuning next = car_state.tuning;
    if (p_tuning.has("max_torque")) {
        next.max_torque = static_cast<float>(static_cast<double>(p_tuning["max_torque"]));
    }
    if (p_tuning.has("idle_rpm")) {
        next.idle_rpm = static_cast<float>(static_cast<double>(p_tuning["idle_rpm"]));
    }
    if (p_tuning.has("redline_rpm")) {
        next.redline_rpm = static_cast<float>(static_cast<double>(p_tuning["redline_rpm"]));
    }
    if (p_tuning.has("final_drive_ratio")) {
        next.final_drive_ratio = static_cast<float>(static_cast<double>(p_tuning["final_drive_ratio"]));
    }
    if (p_tuning.has("gear_count")) {
        next.gear_count = static_cast<int32_t>(static_cast<int64_t>(p_tuning["gear_count"]));
    }
    if (p_tuning.has("wheel_radius")) {
        next.wheel_radius = static_cast<float>(static_cast<double>(p_tuning["wheel_radius"]));
    }
    if (p_tuning.has("mass")) {
        next.mass = static_cast<float>(static_cast<double>(p_tuning["mass"]));
    }
    if (p_tuning.has("drag")) {
        next.drag = static_cast<float>(static_cast<double>(p_tuning["drag"]));
    }
    if (p_tuning.has("steer_rate")) {
        next.steer_rate = static_cast<float>(static_cast<double>(p_tuning["steer_rate"]));
    }
    if (p_tuning.has("gear_ratios")) {
        const godot::Array ratios = p_tuning["gear_ratios"];
        const int n = ratios.size();
        for (int i = 0; i < 6; i++) {
            next.gear_ratios[i] = i < n ? static_cast<float>(static_cast<double>(ratios[i])) : 0.0f;
        }
    }
    fd_car_apply_tuning(&car_state, &next);
    if (p_tuning.has("mouse_dpi")) {
        mouse_dpi = static_cast<double>(p_tuning["mouse_dpi"]);
    }
}

auto fdv::ManualCar::set_max_torque(double p_torque) -> void {
    car_state.tuning.max_torque = static_cast<float>(p_torque < 0.0 ? 0.0 : p_torque);
}

auto fdv::ManualCar::get_max_torque() const -> double {
    return car_state.tuning.max_torque;
}

auto fdv::ManualCar::set_gear_ratios(const godot::PackedFloat32Array &p_ratios) -> void {
    const int n = p_ratios.size();
    for (int i = 0; i < 6; i++) {
        car_state.tuning.gear_ratios[i] = i < n ? p_ratios[i] : 0.0f;
    }
}

auto fdv::ManualCar::get_gear_ratios() const -> godot::PackedFloat32Array {
    godot::PackedFloat32Array out;
    for (int i = 0; i < 6; i++) {
        out.append(car_state.tuning.gear_ratios[i]);
    }
    return out;
}

auto fdv::ManualCar::set_gear_count(int32_t p_count) -> void {
    if (p_count < 1) {
        p_count = 1;
    }
    if (p_count > 5) {
        p_count = 5;
    }
    car_state.tuning.gear_count = p_count;
}

auto fdv::ManualCar::get_gear_count() const -> int32_t {
    return car_state.tuning.gear_count;
}

auto fdv::ManualCar::set_final_drive_ratio(double p_ratio) -> void {
    car_state.tuning.final_drive_ratio = static_cast<float>(p_ratio < 0.0 ? 0.0 : p_ratio);
}

auto fdv::ManualCar::get_final_drive_ratio() const -> double {
    return car_state.tuning.final_drive_ratio;
}

auto fdv::ManualCar::set_idle_rpm(double p_rpm) -> void {
    car_state.tuning.idle_rpm = static_cast<float>(p_rpm < 0.0 ? 0.0 : p_rpm);
}

auto fdv::ManualCar::get_idle_rpm() const -> double {
    return car_state.tuning.idle_rpm;
}

auto fdv::ManualCar::set_redline_rpm(double p_rpm) -> void {
    car_state.tuning.redline_rpm = static_cast<float>(p_rpm < 0.0 ? 0.0 : p_rpm);
}

auto fdv::ManualCar::get_redline_rpm() const -> double {
    return car_state.tuning.redline_rpm;
}

auto fdv::ManualCar::set_mouse_dpi(double p_dpi) -> void {
    mouse_dpi = p_dpi < 0.0 ? 0.0 : p_dpi;
}

auto fdv::ManualCar::get_mouse_dpi() const -> double {
    return mouse_dpi;
}

auto fdv::ManualCar::set_throttle(double p_throttle) -> void {
    throttle = p_throttle < 0.0 ? 0.0 : (p_throttle > 1.0 ? 1.0 : p_throttle);
}

auto fdv::ManualCar::get_throttle() const -> double {
    return throttle;
}

auto fdv::ManualCar::set_steer(double p_steer) -> void {
    steer = p_steer < -1.0 ? -1.0 : (p_steer > 1.0 ? 1.0 : p_steer);
}

auto fdv::ManualCar::get_steer() const -> double {
    return steer;
}

auto fdv::ManualCar::set_clutch(double p_clutch) -> void {
    clutch = p_clutch < 0.0 ? 0.0 : (p_clutch > 1.0 ? 1.0 : p_clutch);
}

auto fdv::ManualCar::get_clutch() const -> double {
    return clutch;
}

auto fdv::ManualCar::shift_gear(int32_t p_gear) -> void {
    if (p_gear < FD_GEAR_REVERSE || p_gear > FD_GEAR_5) {
        return;
    }
    requested_gear = p_gear;
}

auto fdv::ManualCar::get_rpm() const -> double {
    return car_state.rpm;
}

auto fdv::ManualCar::get_gear() const -> int32_t {
    return car_state.gear;
}

auto fdv::ManualCar::get_speed_kmh() const -> double {
    return speed_kmh;
}

auto fdv::ManualCar::is_engine_failed() const -> bool {
    return car_state.engine_failed != 0;
}

auto fdv::ManualCar::get_events() const -> int32_t {
    return car_state.events;
}

auto fdv::ManualCar::adopt_pose(const godot::Vector3 &p_position, double p_heading) -> void {
    car_state.x = static_cast<float>(p_position.x);
    car_state.z = static_cast<float>(p_position.z);
    // p_heading is the node's yaw, which is the core heading + pi.
    car_state.heading = static_cast<float>(p_heading) - static_cast<float>(VehicleSim::kHeadingToNodeYaw);
    car_state.speed = 0.0f;
    car_state.rpm = car_state.tuning.idle_rpm;
    car_state.gear = FD_GEAR_1;
    car_state.events = FD_CAR_EVENT_NONE;
    requested_gear = FD_GEAR_1;
    throttle = 0.0;
    steer = 0.0;
    clutch = 0.0;
    speed_kmh = 0.0;
    sim.accum = 0.0;
    sim.remote = false;
    sim.mirror_pose(this, car_state.x, car_state.z, car_state.heading);
}

auto fdv::ManualCar::restart_engine() -> void {
    fd_car_restart(&car_state);
    requested_gear = car_state.gear;
    throttle = 0.0;
    clutch = 0.0;
    speed_kmh = 0.0;
    sim.accum = 0.0;
}


