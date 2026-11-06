#ifndef FDV_CORE_BINDING_H
#define FDV_CORE_BINDING_H

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace fdv {

// Godot-facing wrapper over shared/fd_core (the engine-neutral C rules).
//
// The Unity build reaches the same functions through P/Invoke; this class is
// the Godot equivalent. Keeping the wrapper thin — one method per core entry
// point, plain values in / plain values out — means a change to the rules only
// needs the C file edited, and both hosts pick it up.
class FdvCore : public godot::RefCounted {
    GDCLASS(FdvCore, godot::RefCounted)

protected:
    static void _bind_methods();

public:
    FdvCore() = default;
    ~FdvCore() = default;

    // Version guard: returns the compiled-in core version. A host should check
    // this matches what it expects before trusting any other result.
    static auto core_version() -> int64_t;

    // -- voice ------------------------------------------------------
    // Encode float samples (-1..1) to ADPCM bytes. Returns an empty array on
    // failure rather than a partially written buffer.
    static auto encode_voice(const godot::PackedFloat32Array &samples, int64_t p_block_samples,
                             double p_gain) -> godot::PackedByteArray;

    // Decode ADPCM bytes. `r_state` carries the per-stream predictor and step
    // across calls, because ADPCM is differential; it is written in place.
    static auto decode_voice(const godot::PackedByteArray &bytes, int64_t p_block_samples,
                             godot::Dictionary r_state) -> godot::PackedFloat32Array;

    // -- vehicle ----------------------------------------------------
    // One fixed step. `state` is a Dictionary mirroring FdVehicle; returns the
    // new state. dt must be 0.02 or the core refuses (determinism guard).
    static auto step_vehicle(godot::Dictionary state, double p_throttle, double p_steer,
                             bool p_steer_assist, int64_t p_preset, int64_t p_riders,
                             double p_dt) -> godot::Dictionary;

    static auto vehicle_preset(int64_t p_preset) -> godot::Dictionary;

    // -- box --------------------------------------------------------
    static auto box_catch_radius(double p_half_extent, double p_padding) -> double;

    // Returns a Vector3 velocity, or Vector3.ZERO if the throw is degenerate.
    static auto box_throw_dir(godot::Vector3 from, godot::Vector3 to, int64_t p_seed,
                              double p_gravity) -> godot::Vector3;

    // Closest claimant wins; ties break on lowest slot. -1 when nobody can.
    static auto box_pick_claimant(godot::Vector3 box, double p_radius, bool p_in_flight,
                                  int64_t p_holder, double p_damage,
                                  godot::PackedInt32Array slots,
                                  godot::PackedVector3Array positions) -> int64_t;

    // -- order ------------------------------------------------------
    static auto order_begin(int64_t p_order_id, int64_t p_tier, double p_time_limit,
                            int64_t p_base_payout) -> godot::Dictionary;
    static auto order_tick(godot::Dictionary run, double p_dt, double p_vehicle_speed,
                           double p_box_damage, double p_time_limit) -> godot::Dictionary;
    static auto order_settle(godot::Dictionary run, int64_t p_base_payout) -> godot::Dictionary;

    // -- monster ----------------------------------------------------
    // `players` is an array of Vector3, `visible` one byte per player, and
    // `noises` a flat array of (x, z, radius, source) quads.
    static auto monster_step(godot::Dictionary m, godot::PackedVector3Array players,
                             godot::PackedByteArray visible, godot::PackedFloat32Array noises,
                             double p_dt) -> godot::Dictionary;
};

} // namespace fdv

#endif // FDV_CORE_BINDING_H
