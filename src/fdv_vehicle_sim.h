#ifndef FDV_VEHICLE_SIM_H
#define FDV_VEHICLE_SIM_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <cmath>

#include "fd_core.h"

namespace fdv {

// The shared, engine-neutral vehicle infrastructure: fixed-step accumulation,
// static AABB collision, host-authoritative replication, and world->local
// transform mirroring. Owned by value by VehicleBase (scooter) and ManualCar
// (car). The physics step itself stays in the C core; each owner calls its own
// fd_*_step() and then resolves collisions through this component. Keeping the
// infrastructure as a component (composition, not inheritance) is what lets the
// two vehicle kinds share it without an is-a chain.
class VehicleSim {
public:
    static constexpr double kDefaultRadius = 0.55;

    // Fixed-step bookkeeping. The core rejects any dt but FD_VEH_FIXED_DT, so
    // the owner accumulates Godot's variable delta and asks begin_step().
    double accum = 0.0;
    bool remote = false;

    // Collision footprint and the world it collides with. Empty colliders means
    // "no world", which is what the camera and unit tests want.
    double radius = kDefaultRadius;
    godot::Array colliders;
    double last_impact = 0.0;
    bool had_collision = false;

    // Accumulate p_delta; true when at least one fixed step is due.
    [[nodiscard]] auto begin_step(double p_delta) -> bool {
        if (p_delta <= 0.0) {
            return false;
        }
        accum += p_delta;
        return accum >= FD_VEH_FIXED_DT;
    }

    // Consume one fixed step's worth of accumulated time.
    auto consume_step() -> void {
        accum -= FD_VEH_FIXED_DT;
    }

    // Mirror a world-space pose into owner's parent-local transform.
    auto mirror_pose(godot::Node3D *owner, float x, float z, float heading) const -> void {
        const godot::Vector3 world(x, owner->get_global_position().y, z);
        const godot::Node3D *parent = godot::Object::cast_to<godot::Node3D>(owner->get_parent());
        if (parent != nullptr) {
            owner->set_position(parent->to_local(world));
        } else {
            owner->set_position(world);
        }
        owner->set_rotation(godot::Vector3(0.0, heading, 0.0));
    }

    // Resolve up to one AABB contact per call. `state` is the C core state whose
    // x/z are read (and written on push-out); `collide` is the C core collide
    // function for that state type. Sets last_impact/had_collision.
    template <typename TState>
    auto resolve_collisions(TState *state, int32_t (*collide)(TState *, float, float, int32_t, float *)) -> void {
        had_collision = false;
        last_impact = 0.0;
        if (colliders.is_empty()) {
            return;
        }

        const double pos_x = state->x;
        const double pos_z = state->z;
        double best_dist = 1e30;
        double best_nx = 0.0, best_nz = 0.0;
        godot::Dictionary best_box;

        for (int i = 0; i < colliders.size(); i++) {
            const godot::Dictionary box = colliders[i];
            if (!box.has("pos") || !box.has("size")) {
                continue;
            }
            const godot::Vector3 centre = box["pos"];
            const godot::Vector3 half = static_cast<godot::Vector3>(box["size"]) * 0.5;

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

        state->x = static_cast<float>(pos_x + best_nx * radius);
        state->z = static_cast<float>(pos_z + best_nz * radius);

        const int32_t kind = best_box.has("kind")
            ? static_cast<int32_t>(static_cast<int64_t>(best_box["kind"])) : 0;
        float impact = 0.0f;
        if (collide(state, static_cast<float>(best_nx), static_cast<float>(best_nz), kind, &impact) != 0) {
            had_collision = true;
            last_impact = impact;
        }
    }
};

} // namespace fdv

#endif // FDV_VEHICLE_SIM_H
