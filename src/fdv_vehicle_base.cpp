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
    godot::ClassDB::bind_method(godot::D_METHOD("set_colliders", "boxes"),
                                &VehicleBase::set_colliders);
    godot::ClassDB::bind_method(godot::D_METHOD("get_collider_count"),
                                &VehicleBase::get_collider_count);
    godot::ClassDB::bind_method(godot::D_METHOD("get_last_impact"), &VehicleBase::get_last_impact);
    godot::ClassDB::bind_method(godot::D_METHOD("get_had_collision"),
                                &VehicleBase::get_had_collision);
}

auto fdv::VehicleBase::_ready() -> void {
    reset_vehicle();
}

auto fdv::VehicleBase::_physics_process(double p_delta) -> void {
    // The engine owns the tick; whether a step advanced is the caller's
    // question, not this one's.
    (void)step(p_delta);
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
    last_impact = 0.0f;
    had_collision = false;
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

        (void)resolve_collisions();

        accum -= FD_VEH_FIXED_DT;
        advanced = true;
    }

    if (advanced) {
        speed_kmh = vehicle_state.speed * 3.6;

        // Mirror the simulation into the node's transform so it can be seen.
        // The simulation is in world space; this node may not be at the origin,
        // so the world position is converted rather than written to position.
        write_transform_from_world();
    }
    return advanced;
}

auto fdv::VehicleBase::set_colliders(const godot::Array &p_boxes) -> void {
    colliders = p_boxes;
}

auto fdv::VehicleBase::get_collider_count() const -> int64_t {
    return colliders.size();
}

auto fdv::VehicleBase::resolve_collisions() -> void {
    if (colliders.is_empty()) {
        return;
    }

    // At most one contact per step. Resolving several at once needs an
    // ordering rule, and every ordering rule that depends on iteration order
    // is a desync waiting to happen. Successive steps settle a corner.
    const double pos_x = vehicle_state.x;
    const double pos_z = vehicle_state.z;

    double best_dist = 1e30;
    double best_nx = 0.0, best_nz = 0.0;
    godot::Dictionary best_box;

    const double radius = kVehicleRadius;
    for (int i = 0; i < colliders.size(); i++) {
        const godot::Dictionary box = colliders[i];
        if (!box.has("pos") || !box.has("size")) {
            continue;
        }
        const godot::Vector3 centre = box["pos"];
        const godot::Vector3 half = static_cast<godot::Vector3>(box["size"]) * 0.5;

        // Closest point on the box to the scooter, in the XZ plane.
        // Written out rather than std::clamp because Vector3 components are
        // godot::real_t, and mixing that with double in a template argument
        // fails to deduce.
        const double min_x = centre.x - half.x;
        const double max_x = centre.x + half.x;
        const double min_z = centre.z - half.z;
        const double max_z = centre.z + half.z;
        const double cx = pos_x < min_x ? min_x : (pos_x > max_x ? max_x : pos_x);
        const double cz = pos_z < min_z ? min_z : (pos_z > max_z ? max_z : pos_z);
        double dx = pos_x - cx;
        double dz = pos_z - cz;
        double dist = std::sqrt(dx * dx + dz * dz);

        if (dist >= radius) {
            continue;
        }

        if (dist < 1e-4) {
            // Centre is inside the box: no surface normal exists, so push out
            // along the shallowest axis, the direction of least overlap.
            const double ox = pos_x - centre.x;
            const double oz = pos_z - centre.z;
            const double pen_x = half.x - std::fabs(ox);
            const double pen_z = half.z - std::fabs(oz);
            if (pen_x < pen_z) {
                dx = (ox >= 0.0) ? 1.0 : -1.0;
                dz = 0.0;
            } else {
                dx = 0.0;
                dz = (oz >= 0.0) ? 1.0 : -1.0;
            }
            dist = 1e-4;
        }

        if (dist < best_dist) {
            best_dist = dist;
            const double len = std::sqrt(dx * dx + dz * dz);
            best_nx = dx / len;
            best_nz = dz / len;
            best_box = box;
        }
    }

    if (best_box.is_empty()) {
        return;
    }

    // Push back to the surface first, so the core always resolves a contact
    // that is already resolved geometrically and cannot leave it embedded.
    vehicle_state.x = static_cast<float>(pos_x + best_nx * radius);
    vehicle_state.z = static_cast<float>(pos_z + best_nz * radius);

    const int32_t kind = best_box.has("kind")
        ? static_cast<int32_t>(static_cast<int64_t>(best_box["kind"])) : 0;
    float impact = 0.0f;
    if (fd_vehicle_collide(&vehicle_state, static_cast<float>(best_nx),
                           static_cast<float>(best_nz), kind, &impact) != 0) {
        had_collision = true;
        last_impact = impact;
    }
}

auto fdv::VehicleBase::get_last_impact() const -> double {
    return last_impact;
}

auto fdv::VehicleBase::get_had_collision() const -> bool {
    return had_collision;
}

auto fdv::VehicleBase::reset_vehicle() -> void {
    vehicle_state = FdVehicle{};
    vehicle_state.x = static_cast<float>(get_global_position().x);
    vehicle_state.z = static_cast<float>(get_global_position().z);
    vehicle_state.heading = static_cast<float>(get_global_rotation().y);
    vehicle_state.state = FD_VEH_UPRIGHT;
    speed_kmh = 0.0;
    accum = 0.0;
    remote = false;
    last_impact = 0.0;
    had_collision = false;
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

    write_transform_from_world();
}

auto fdv::VehicleBase::write_transform_from_world() -> void {
    // Keep the parent's Y and yaw offsets: this node is a child of the rotating
    // bike root, so its local transform is not the world pose.
    const godot::Vector3 world(vehicle_state.x, get_global_position().y, vehicle_state.z);
    const godot::Node3D *parent = godot::Object::cast_to<godot::Node3D>(get_parent());
    if (parent != nullptr) {
        set_position(parent->to_local(world));
    } else {
        set_position(world);
    }
    // Heading comes from the core, in world space.
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
