#include "fdv_core_binding.h"

#include <godot_cpp/core/class_db.hpp>

#include "fd_core.h"

// Dictionary keys are short and stable; the Unity side uses field names on a
// struct, so the two are compared by value in the shared tests rather than by
// name. Keeping them short keeps the per-tick cost down.
namespace {
constexpr const char *K_X = "x";
constexpr const char *K_Z = "z";
constexpr const char *K_HEADING = "heading";
constexpr const char *K_SPEED = "speed";
constexpr const char *K_LATERAL = "lateral";
constexpr const char *K_LEAN = "lean";
constexpr const char *K_STEER = "steer";
constexpr const char *K_STATE = "state";
constexpr const char *K_TIMER = "timer";

auto dict_to_vehicle(const godot::Dictionary &d) -> FdVehicle {
    FdVehicle v{};
    v.x = static_cast<float>(static_cast<double>(d.get(K_X, 0.0)));
    v.z = static_cast<float>(static_cast<double>(d.get(K_Z, 0.0)));
    v.heading = static_cast<float>(static_cast<double>(d.get(K_HEADING, 0.0)));
    v.speed = static_cast<float>(static_cast<double>(d.get(K_SPEED, 0.0)));
    v.lateral_speed = static_cast<float>(static_cast<double>(d.get(K_LATERAL, 0.0)));
    v.lean = static_cast<float>(static_cast<double>(d.get(K_LEAN, 0.0)));
    v.steer_angle = static_cast<float>(static_cast<double>(d.get(K_STEER, 0.0)));
    v.state = static_cast<int32_t>(static_cast<int64_t>(d.get(K_STATE, 0)));
    v.state_timer = static_cast<float>(static_cast<double>(d.get(K_TIMER, 0.0)));
    return v;
}

auto vehicle_to_dict(const FdVehicle &v) -> godot::Dictionary {
    godot::Dictionary d;
    d[K_X] = v.x;
    d[K_Z] = v.z;
    d[K_HEADING] = v.heading;
    d[K_SPEED] = v.speed;
    d[K_LATERAL] = v.lateral_speed;
    d[K_LEAN] = v.lean;
    d[K_STEER] = v.steer_angle;
    d[K_STATE] = static_cast<int64_t>(v.state);
    d[K_TIMER] = v.state_timer;
    return d;
}

auto dict_to_order_run(const godot::Dictionary &d) -> FdOrderRun {
    FdOrderRun r{};
    r.order_id = static_cast<int32_t>(static_cast<int64_t>(d.get("order_id", 0)));
    r.tier = static_cast<int32_t>(static_cast<int64_t>(d.get("tier", 0)));
    r.base_payout = static_cast<int32_t>(static_cast<int64_t>(d.get("base_payout", 0)));
    r.time_limit = static_cast<float>(static_cast<double>(d.get("time_limit", 0.0)));
    r.elapsed = static_cast<float>(static_cast<double>(d.get("elapsed", 0.0)));
    r.soup = static_cast<float>(static_cast<double>(d.get("soup", 1.0)));
    r.heat = static_cast<float>(static_cast<double>(d.get("heat", 1.0)));
    r.outcome = static_cast<int32_t>(static_cast<int64_t>(d.get("outcome", 0)));
    return r;
}

auto order_run_to_dict(const FdOrderRun &r) -> godot::Dictionary {
    godot::Dictionary d;
    d["order_id"] = static_cast<int64_t>(r.order_id);
    d["tier"] = static_cast<int64_t>(r.tier);
    d["base_payout"] = static_cast<int64_t>(r.base_payout);
    d["time_limit"] = r.time_limit;
    d["elapsed"] = r.elapsed;
    d["soup"] = r.soup;
    d["heat"] = r.heat;
    d["outcome"] = static_cast<int64_t>(r.outcome);
    return d;
}

auto default_tuning() -> FdOrderTuning {
    FdOrderTuning t{};
    fd_order_default_tuning(&t);
    return t;
}
} // namespace

auto fdv::FdvCore::_bind_methods() -> void {
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("core_version"), &FdvCore::core_version);

    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("encode_voice", "samples", "block_samples", "gain"),
                                       &FdvCore::encode_voice);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("decode_voice", "bytes", "block_samples", "state"),
                                       &FdvCore::decode_voice);

    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("step_vehicle", "state", "throttle", "steer",
                                                       "steer_assist", "preset", "riders", "dt"),
                                       &FdvCore::step_vehicle);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("vehicle_preset", "preset"),
                                       &FdvCore::vehicle_preset);
    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("vehicle_collide", "state", "normal_x",
                                                       "normal_z", "kind"),
                                       &FdvCore::vehicle_collide);

    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("box_catch_radius", "half_extent", "padding"),
                                       &FdvCore::box_catch_radius);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("box_throw_dir", "from", "to", "seed", "gravity"),
                                       &FdvCore::box_throw_dir);
    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("box_pick_claimant", "box", "radius", "in_flight",
                                                       "holder", "damage", "slots", "positions"),
                                       &FdvCore::box_pick_claimant);

    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("order_begin", "order_id", "tier", "time_limit",
                                                       "base_payout"),
                                       &FdvCore::order_begin);
    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("order_tick", "run", "dt", "vehicle_speed",
                                                       "box_damage", "time_limit"),
                                       &FdvCore::order_tick);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("order_settle", "run", "base_payout"),
                                       &FdvCore::order_settle);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("order_hard_hit", "heat", "impact_speed"),
                                       &FdvCore::order_hard_hit);

    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("monster_step", "m", "players", "visible", "noises", "dt"),
                                       &FdvCore::monster_step);
    godot::ClassDB::bind_static_method("FdvCore", godot::D_METHOD("monster_noise_radius", "source"),
                                       &FdvCore::monster_noise_radius);

    godot::ClassDB::bind_static_method("FdvCore",
                                       godot::D_METHOD("mood_tick", "state", "dt", "now",
                                                       "monster_distance", "monster_visible"),
                                       &FdvCore::mood_tick);
}

auto fdv::FdvCore::core_version() -> int64_t {
    return fd_core_version();
}

auto fdv::FdvCore::encode_voice(const godot::PackedFloat32Array &samples, int64_t p_block_samples,
                                double p_gain) -> godot::PackedByteArray {
    const int n = samples.size();
    if (n <= 0) {
        return godot::PackedByteArray();
    }

    godot::PackedByteArray out;
    out.resize(n);
    const int written = fd_adpcm_encode(samples.ptr(), n, static_cast<int>(p_block_samples),
                                        static_cast<float>(p_gain), reinterpret_cast<uint8_t *>(out.ptrw()), n);
    if (written < 0) {
        return godot::PackedByteArray();
    }
    if (written < n) {
        out.resize(written);
    }
    return out;
}

auto fdv::FdvCore::decode_voice(const godot::PackedByteArray &bytes, int64_t p_block_samples,
                                godot::Dictionary r_state) -> godot::PackedFloat32Array {
    const int n = bytes.size();
    if (n <= 0) {
        return godot::PackedFloat32Array();
    }

    int predictor = static_cast<int>(static_cast<int64_t>(r_state.get("predictor", 0)));
    int step = static_cast<int>(static_cast<int64_t>(r_state.get("step", 0)));

    godot::PackedFloat32Array out;
    out.resize(n);
    const int written = fd_adpcm_decode(reinterpret_cast<const uint8_t *>(bytes.ptr()), n,
                                        static_cast<int>(p_block_samples), &predictor, &step, out.ptrw(), n);
    if (written < 0) {
        return godot::PackedFloat32Array();
    }

    // Callers store this back to keep the stream's predictor alive.
    r_state["predictor"] = static_cast<int64_t>(predictor);
    r_state["step"] = static_cast<int64_t>(step);
    return out;
}

auto fdv::FdvCore::step_vehicle(godot::Dictionary state, double p_throttle, double p_steer,
                                bool p_steer_assist, int64_t p_preset, int64_t p_riders,
                                double p_dt) -> godot::Dictionary {
    FdVehicle v = dict_to_vehicle(state);
    FdVehicleInput in{};
    in.throttle = static_cast<float>(p_throttle);
    in.steer = static_cast<float>(p_steer);
    in.steer_assist = p_steer_assist ? 1 : 0;

    if (fd_vehicle_step(&v, &in, static_cast<int32_t>(p_preset), static_cast<int32_t>(p_riders),
                        static_cast<float>(p_dt)) != FD_OK) {
        // Return the state unchanged rather than a half-stepped one; the caller
        // can detect the bad dt from the unchanged timer.
        return state;
    }
    return vehicle_to_dict(v);
}

auto fdv::FdvCore::vehicle_preset(int64_t p_preset) -> godot::Dictionary {
    FdVehicleTuning t{};
    godot::Dictionary d;
    if (fd_vehicle_preset(static_cast<int32_t>(p_preset), &t) != FD_OK) {
        return d;
    }

    d["preset"] = static_cast<int64_t>(t.preset);
    d["max_speed"] = t.max_speed;
    d["acceleration"] = t.acceleration;
    d["steer_rate"] = t.steer_rate;
    d["grip"] = t.grip;
    d["slide_force"] = t.slide_force;
    d["lean_per_lateral"] = t.lean_per_lateral;
    d["max_lean"] = t.max_lean;
    d["flip_lean_threshold"] = t.flip_lean_threshold;
    d["flip_duration"] = t.flip_duration;
    d["skid_lateral_threshold"] = t.skid_lateral_threshold;
    return d;
}

auto fdv::FdvCore::vehicle_collide(godot::Dictionary state, double p_normal_x, double p_normal_z,
                                   int64_t p_kind) -> godot::Dictionary {
    FdVehicle v = dict_to_vehicle(state);
    float impact = 0.0f;
    const int32_t hit = fd_vehicle_collide(&v, static_cast<float>(p_normal_x),
                                           static_cast<float>(p_normal_z),
                                           static_cast<int32_t>(p_kind), &impact);

    godot::Dictionary d;
    d["collided"] = hit != 0;
    d["impact"] = impact;
    d["state"] = vehicle_to_dict(v);
    return d;
}

auto fdv::FdvCore::box_catch_radius(double p_half_extent, double p_padding) -> double {
    return static_cast<double>(fd_box_catch_radius(static_cast<float>(p_half_extent),
                                                   static_cast<float>(p_padding)));
}

auto fdv::FdvCore::box_throw_dir(godot::Vector3 from, godot::Vector3 to, int64_t p_seed,
                                 double p_gravity) -> godot::Vector3 {
    FdVec3 vel{};
    if (!fd_box_throw_dir(from.x, from.y, from.z, to.x, to.y, to.z,
                          static_cast<uint32_t>(p_seed), static_cast<float>(p_gravity), &vel)) {
        return godot::Vector3(0, 0, 0);
    }
    return godot::Vector3(vel.x, vel.y, vel.z);
}

auto fdv::FdvCore::box_pick_claimant(godot::Vector3 box, double p_radius, bool p_in_flight,
                                     int64_t p_holder, double p_damage,
                                     godot::PackedInt32Array slots,
                                     godot::PackedVector3Array positions) -> int64_t {
    const int n = slots.size();
    if (n <= 0 || positions.size() < n) {
        return -1;
    }

    godot::PackedFloat32Array xs, ys, zs;
    xs.resize(n);
    ys.resize(n);
    zs.resize(n);
    for (int i = 0; i < n; i++) {
        const godot::Vector3 p = positions[i];
        xs.set(i, p.x);
        ys.set(i, p.y);
        zs.set(i, p.z);
    }

    return fd_box_pick_claimant(box.x, box.y, box.z, static_cast<float>(p_radius), p_in_flight ? 1 : 0,
                                static_cast<int32_t>(p_holder), static_cast<float>(p_damage),
                                slots.ptr(), xs.ptr(), ys.ptr(), zs.ptr(), n);
}

auto fdv::FdvCore::order_begin(int64_t p_order_id, int64_t p_tier, double p_time_limit,
                               int64_t p_base_payout) -> godot::Dictionary {
    FdOrderRun run{};
    if (fd_order_begin(&run, static_cast<int32_t>(p_order_id), static_cast<int32_t>(p_tier),
                       static_cast<float>(p_time_limit), static_cast<int32_t>(p_base_payout)) != FD_OK) {
        return godot::Dictionary();
    }
    return order_run_to_dict(run);
}

auto fdv::FdvCore::order_tick(godot::Dictionary run, double p_dt, double p_vehicle_speed,
                              double p_box_damage, double p_time_limit) -> godot::Dictionary {
    FdOrderRun r = dict_to_order_run(run);

    const FdOrderTuning t = default_tuning();
    if (fd_order_tick(&r, static_cast<float>(p_dt), static_cast<float>(p_vehicle_speed),
                      static_cast<float>(p_box_damage), static_cast<float>(p_time_limit), &t) != FD_OK) {
        return run;
    }
    return order_run_to_dict(r);
}

auto fdv::FdvCore::order_settle(godot::Dictionary run, int64_t p_base_payout) -> godot::Dictionary {
    const FdOrderRun r = dict_to_order_run(run);

    const FdOrderTuning t = default_tuning();
    FdSettlement s{};
    if (fd_order_settle(&r, static_cast<int32_t>(p_base_payout), &t, &s) != FD_OK) {
        return godot::Dictionary();
    }

    godot::Dictionary d;
    d["order_id"] = static_cast<int64_t>(s.order_id);
    d["outcome"] = static_cast<int64_t>(s.outcome);
    d["payout"] = static_cast<int64_t>(s.payout);
    d["time_margin"] = s.time_margin;
    d["final_soup"] = s.final_soup;
    d["final_heat"] = s.final_heat;
    d["customer_enraged"] = static_cast<int64_t>(s.customer_enraged);
    return d;
}

auto fdv::FdvCore::order_hard_hit(double p_heat, double p_impact_speed) -> double {
    const FdOrderTuning t = default_tuning();
    return static_cast<double>(fd_order_hard_hit(static_cast<float>(p_heat),
                                                 static_cast<float>(p_impact_speed), &t));
}

auto fdv::FdvCore::monster_noise_radius(int64_t p_source) -> double {
    return static_cast<double>(fd_monster_noise_radius(static_cast<int32_t>(p_source)));
}

auto fdv::FdvCore::mood_tick(godot::Dictionary r_state, double p_dt, double p_now,
                             double p_monster_distance, bool p_monster_visible) -> godot::Dictionary {
    // Saturation, the two hold timers and the kick all live on the caller's
    // Dictionary, so one Godot node can own a director without this binding
    // keeping hidden per-object state (statics are shared by every caller).
    float saturation = static_cast<float>(static_cast<double>(r_state.get("saturation", 0.65)));
    float laugh_until = static_cast<float>(static_cast<double>(r_state.get("laugh_until", 0.0)));
    float fear_until = static_cast<float>(static_cast<double>(r_state.get("fear_until", 0.0)));
    float kick = static_cast<float>(static_cast<double>(r_state.get("kick", 0.0)));

    const FdMoodOutput out = fd_mood_tick(&saturation, static_cast<float>(p_dt),
                                          static_cast<float>(p_now), laugh_until, fear_until,
                                          static_cast<float>(p_monster_distance),
                                          p_monster_visible ? 1 : 0,
                                          4.0f, 1.6f, kick, 3.5f,
                                          &laugh_until, &fear_until, &kick);

    r_state["saturation"] = saturation;
    r_state["laugh_until"] = laugh_until;
    r_state["fear_until"] = fear_until;
    r_state["kick"] = kick;

    godot::Dictionary d;
    d["saturation"] = out.saturation;
    d["horror_tint"] = out.horror_tint;
    d["grain"] = out.grain;
    d["kick"] = out.kick;
    return d;
}

auto fdv::FdvCore::monster_step(godot::Dictionary m, godot::PackedVector3Array players,
                                godot::PackedByteArray visible, godot::PackedFloat32Array noises,
                                double p_dt) -> godot::Dictionary {
    FdMonster mon{};
    mon.x = static_cast<float>(static_cast<double>(m.get("x", 0.0)));
    mon.z = static_cast<float>(static_cast<double>(m.get("z", 0.0)));
    mon.heading = static_cast<float>(static_cast<double>(m.get("heading", 0.0)));
    mon.speed = static_cast<float>(static_cast<double>(m.get("speed", 0.0)));
    mon.state = static_cast<int32_t>(static_cast<int64_t>(m.get("state", 0)));
    mon.state_timer = static_cast<float>(static_cast<double>(m.get("state_timer", 0.0)));
    mon.target_slot = static_cast<int32_t>(static_cast<int64_t>(m.get("target_slot", -1)));
    mon.last_noise_x = static_cast<float>(static_cast<double>(m.get("last_noise_x", 0.0)));
    mon.last_noise_z = static_cast<float>(static_cast<double>(m.get("last_noise_z", 0.0)));
    mon.patrol_index = static_cast<int32_t>(static_cast<int64_t>(m.get("patrol_index", 0)));

    const int pc = players.size();
    godot::PackedFloat32Array xy;
    xy.resize(pc * 2);
    for (int i = 0; i < pc; i++) {
        xy.set(i * 2, players[i].x);
        xy.set(i * 2 + 1, players[i].z);
    }

    fd_monster_step(&mon, xy.ptr(),
                    visible.size() >= pc ? reinterpret_cast<const uint8_t *>(visible.ptr()) : nullptr,
                    pc,
                    noises.size() > 0 ? noises.ptr() : nullptr, noises.size() / 4,
                    12.0f, 0.52f, 3.0f, static_cast<float>(p_dt));

    godot::Dictionary d;
    d["x"] = mon.x;
    d["z"] = mon.z;
    d["heading"] = mon.heading;
    d["speed"] = mon.speed;
    d["state"] = static_cast<int64_t>(mon.state);
    d["state_timer"] = mon.state_timer;
    d["target_slot"] = static_cast<int64_t>(mon.target_slot);
    d["last_noise_x"] = mon.last_noise_x;
    d["last_noise_z"] = mon.last_noise_z;
    d["patrol_index"] = static_cast<int64_t>(mon.patrol_index);
    return d;
}
