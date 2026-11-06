#include "fdv_vehicle_base.h"

#include <godot_cpp/core/class_db.hpp>

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
}

auto fdv::VehicleBase::_ready() -> void {
    reset_vehicle();
}

auto fdv::VehicleBase::_physics_process(double p_delta) -> void {
    step(p_delta);
}

auto fdv::VehicleBase::step(double p_delta) -> bool {
    // A remote vehicle is exactly where the host says it is; running the local
    // simulation as well would fight the replicated state every tick.
    if (remote) {
        return false;
    }

    // The core rejects anything but the fixed step, deliberately: a host that
    // supplies its own delta desyncs from every other peer on the first frame
    // and never recovers. Accumulate Godot's variable delta instead.
    if (p_delta <= 0.0) {
        return false;
    }

    accum += p_delta;
    bool advanced = false;
    // Cap the catch-up so a frame hitch does not spiral into hundreds of steps.
    int guard = 0;
    while (accum >= FD_VEH_FIXED_DT && guard++ < 8) {
        FdVehicleInput input{};
        input.throttle = static_cast<float>(throttle);
        input.steer = static_cast<float>(steer);
        input.steer_assist = steer_assist ? 1 : 0;

        if (fd_vehicle_step(&vehicle_state, &input, preset, riders, FD_VEH_FIXED_DT) != FD_OK) {
            break;
        }

        accum -= FD_VEH_FIXED_DT;
        advanced = true;
    }

    if (advanced) {
        speed_kmh = vehicle_state.speed * 3.6;

        // Mirror the simulation into the node's transform so it can be seen.
        set_position(godot::Vector3(vehicle_state.x, get_position().y, vehicle_state.z));
        set_rotation(godot::Vector3(0.0, vehicle_state.heading, 0.0));
    }
    return advanced;
}

auto fdv::VehicleBase::reset_vehicle() -> void {
    vehicle_state = FdVehicle{};
    vehicle_state.x = static_cast<float>(get_position().x);
    vehicle_state.z = static_cast<float>(get_position().z);
    vehicle_state.heading = static_cast<float>(get_rotation().y);
    vehicle_state.state = FD_VEH_UPRIGHT;
    speed_kmh = 0.0;
    accum = 0.0;
    remote = false;
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
    remote = true;

    set_position(godot::Vector3(vehicle_state.x, get_position().y, vehicle_state.z));
    set_rotation(godot::Vector3(0.0, vehicle_state.heading, 0.0));
}

auto fdv::VehicleBase::is_remote() const -> bool {
    return remote;
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
