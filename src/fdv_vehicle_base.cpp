#include "fdv_vehicle_base.h"

#include <godot_cpp/core/class_db.hpp>

#include <cmath>

auto fdv::VehicleBase::_bind_methods() -> void {
    godot::ClassDB::bind_method(godot::D_METHOD("set_speed_kmh", "speed_kmh"), &VehicleBase::set_speed_kmh);
    godot::ClassDB::bind_method(godot::D_METHOD("get_speed_kmh"), &VehicleBase::get_speed_kmh);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "speed_kmh"), "set_speed_kmh", "get_speed_kmh");

    godot::ClassDB::bind_method(godot::D_METHOD("speed_mps"), &VehicleBase::speed_mps);

    godot::ClassDB::bind_method(godot::D_METHOD("set_preset", "preset"), &VehicleBase::set_preset);
    godot::ClassDB::bind_method(godot::D_METHOD("get_preset"), &VehicleBase::get_preset);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::INT, "preset", godot::PROPERTY_HINT_RANGE, "0,2,1"),
                 "set_preset", "get_preset");

    godot::ClassDB::bind_method(godot::D_METHOD("set_riders", "riders"), &VehicleBase::set_riders);
    godot::ClassDB::bind_method(godot::D_METHOD("get_riders"), &VehicleBase::get_riders);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::INT, "riders", godot::PROPERTY_HINT_RANGE, "1,4,1"),
                 "set_riders", "get_riders");

    godot::ClassDB::bind_method(godot::D_METHOD("set_throttle", "throttle"), &VehicleBase::set_throttle);
    godot::ClassDB::bind_method(godot::D_METHOD("get_throttle"), &VehicleBase::get_throttle);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "throttle", godot::PROPERTY_HINT_RANGE, "-1,1,0.01"),
                 "set_throttle", "get_throttle");

    godot::ClassDB::bind_method(godot::D_METHOD("set_steer", "steer"), &VehicleBase::set_steer);
    godot::ClassDB::bind_method(godot::D_METHOD("get_steer"), &VehicleBase::get_steer);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "steer", godot::PROPERTY_HINT_RANGE, "-1,1,0.01"),
                 "set_steer", "get_steer");

    godot::ClassDB::bind_method(godot::D_METHOD("get_vehicle_state"), &VehicleBase::get_vehicle_state);
    godot::ClassDB::bind_method(godot::D_METHOD("get_lean"), &VehicleBase::get_lean);
    godot::ClassDB::bind_method(godot::D_METHOD("step", "delta"), &VehicleBase::step);
    godot::ClassDB::bind_method(godot::D_METHOD("reset_vehicle"), &VehicleBase::reset_vehicle);
    godot::ClassDB::bind_method(godot::D_METHOD("adopt_authoritative", "state"),
                                &VehicleBase::adopt_authoritative);
    godot::ClassDB::bind_method(godot::D_METHOD("is_remote"), &VehicleBase::is_remote);
    godot::ClassDB::bind_method(godot::D_METHOD("resume_local_simulation"),
                                &VehicleBase::resume_local_simulation);
    godot::ClassDB::bind_method(godot::D_METHOD("set_colliders", "boxes"),
                                &VehicleBase::set_colliders);
    godot::ClassDB::bind_method(godot::D_METHOD("get_collider_count"),
                                &VehicleBase::get_collider_count);
    godot::ClassDB::bind_method(godot::D_METHOD("get_last_impact"), &VehicleBase::get_last_impact);
    godot::ClassDB::bind_method(godot::D_METHOD("get_had_collision"),
                                &VehicleBase::get_had_collision);
}

auto fdv::VehicleBase::_ready() -> void {
    godot::Node3D::_ready(); // 父
    reset_vehicle();         // 子
}

auto fdv::VehicleBase::_physics_process(double p_delta) -> void {
    godot::Node3D::_physics_process(p_delta); // 父
    (void)step(p_delta);                       // 子
}

auto fdv::VehicleBase::step(double p_delta) -> bool {
    // A remote vehicle is exactly where the host says it is; running the local
    // simulation as well would fight the replicated state every tick.
    if (sim.remote) {
        return false;
    }
    if (!sim.begin_step(p_delta)) {
        return false;
    }

    bool advanced = false;
    // Cap the catch-up so a frame hitch does not spiral into hundreds of steps.
    int guard = 0;
    while (sim.accum >= FD_VEH_FIXED_DT && guard++ < 8) {
        FdVehicleInput input{};
        input.throttle = static_cast<float>(throttle);
        input.steer = static_cast<float>(steer);
        input.steer_assist = steer_assist ? 1 : 0;

        if (fd_vehicle_step(&vehicle_state, &input, preset, riders, FD_VEH_FIXED_DT) != FD_OK) {
            break;
        }

        sim.resolve_collisions(&vehicle_state, fd_vehicle_collide);
        sim.consume_step();
        advanced = true;
    }

    if (advanced) {
        speed_kmh = vehicle_state.speed * 3.6;
        sim.mirror_pose(this, vehicle_state.x, vehicle_state.z, vehicle_state.heading);
    }
    return advanced;
}

auto fdv::VehicleBase::set_colliders(const godot::Array &p_boxes) -> void {
    sim.colliders = p_boxes;
}

auto fdv::VehicleBase::get_collider_count() const -> int64_t {
    return sim.colliders.size();
}

auto fdv::VehicleBase::get_last_impact() const -> double {
    return sim.last_impact;
}

auto fdv::VehicleBase::get_had_collision() const -> bool {
    return sim.had_collision;
}

auto fdv::VehicleBase::reset_vehicle() -> void {
    vehicle_state = FdVehicle{};
    vehicle_state.x = static_cast<float>(get_global_position().x);
    vehicle_state.z = static_cast<float>(get_global_position().z);
    // The node's rotation is heading + pi, so converting back needs the same
    // offset subtracted.
    //
    // Written as an explicit wrap rather than trusting the raw subtraction: at
    // rotation 0 the node's heading is exactly -pi, and the core treats -pi and
    // +pi as the same bearing. Subtracting without wrapping lands somewhere in
    // (-2pi, 0], and a heading of -pi + delta then steers the scooter the
    // opposite way from a heading of +pi + delta, which inverts A and D from
    // the seat without changing anything else.
    float h = static_cast<float>(get_global_rotation().y) - VehicleSim::kHeadingToNodeYaw;
    while (h > static_cast<float>(3.14159265358979)) h -= static_cast<float>(6.283185307179586);
    while (h < static_cast<float>(-3.14159265358979)) h += static_cast<float>(6.283185307179586);
    vehicle_state.heading = h;
    vehicle_state.state = FD_VEH_UPRIGHT;
    speed_kmh = 0.0;
    sim.accum = 0.0;
    sim.remote = false;
    sim.last_impact = 0.0;
    sim.had_collision = false;
    // Write the pose back immediately. Without this the node keeps whatever
    // rotation it had while the core believes it faces the heading just
    // computed, and the two only realign on the next step.
    sim.mirror_pose(this, static_cast<float>(vehicle_state.x),
                    static_cast<float>(vehicle_state.z),
                    static_cast<float>(vehicle_state.heading));
}

auto fdv::VehicleBase::adopt_authoritative(const godot::Dictionary &p_state) -> void {
    vehicle_state.x = static_cast<float>(static_cast<double>(p_state.get("x", 0.0)));
    vehicle_state.z = static_cast<float>(static_cast<double>(p_state.get("z", 0.0)));
    vehicle_state.heading = static_cast<float>(static_cast<double>(p_state.get("heading", 0.0)));
    vehicle_state.speed = static_cast<float>(static_cast<double>(p_state.get("speed", 0.0)));
    vehicle_state.lateral_speed =
        static_cast<float>(static_cast<double>(p_state.get("lateral_speed", 0.0)));
    vehicle_state.lean = static_cast<float>(static_cast<double>(p_state.get("lean", 0.0)));
    vehicle_state.steer_angle =
        static_cast<float>(static_cast<double>(p_state.get("steer_angle", 0.0)));
    vehicle_state.state = static_cast<int32_t>(static_cast<int64_t>(p_state.get("state", 0)));
    vehicle_state.state_timer =
        static_cast<float>(static_cast<double>(p_state.get("state_timer", 0.0)));

    speed_kmh = vehicle_state.speed * 3.6;
    sim.remote = true;

    sim.mirror_pose(this, vehicle_state.x, vehicle_state.z, vehicle_state.heading);
}

auto fdv::VehicleBase::is_remote() const -> bool {
    return sim.remote;
}

auto fdv::VehicleBase::resume_local_simulation() -> void {
    sim.remote = false;
    sim.accum = 0.0;
}

auto fdv::VehicleBase::set_speed_kmh(double p_speed_kmh) -> void {
    speed_kmh = p_speed_kmh;
    // Keep the simulation in step when a script writes the property directly.
    vehicle_state.speed = static_cast<float>(p_speed_kmh * KMH_TO_MPS);
}

auto fdv::VehicleBase::get_speed_kmh() const -> double {
    return speed_kmh;
}

auto fdv::VehicleBase::speed_mps() const -> double {
    return speed_kmh * KMH_TO_MPS;
}

auto fdv::VehicleBase::set_preset(int32_t p_preset) -> void {
    if (p_preset < FD_HANDLING_MILD || p_preset > FD_HANDLING_ABSURD) {
        return;
    }
    preset = p_preset;
}

auto fdv::VehicleBase::get_preset() const -> int32_t {
    return preset;
}

auto fdv::VehicleBase::set_riders(int32_t p_riders) -> void {
    if (p_riders < 1) {
        p_riders = 1;
    }
    if (p_riders > 4) {
        p_riders = 4;
    }
    riders = p_riders;
}

auto fdv::VehicleBase::get_riders() const -> int32_t {
    return riders;
}

auto fdv::VehicleBase::set_throttle(double p_throttle) -> void {
    throttle = p_throttle < -1.0 ? -1.0 : (p_throttle > 1.0 ? 1.0 : p_throttle);
}

auto fdv::VehicleBase::get_throttle() const -> double {
    return throttle;
}

auto fdv::VehicleBase::set_steer(double p_steer) -> void {
    steer = p_steer < -1.0 ? -1.0 : (p_steer > 1.0 ? 1.0 : p_steer);
}

auto fdv::VehicleBase::get_steer() const -> double {
    return steer;
}

auto fdv::VehicleBase::get_vehicle_state() const -> int32_t {
    return static_cast<int32_t>(vehicle_state.state);
}

auto fdv::VehicleBase::get_lean() const -> double {
    return vehicle_state.lean;
}
