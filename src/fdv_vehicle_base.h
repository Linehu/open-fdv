#ifndef FDV_VEHICLE_BASE_H
#define FDV_VEHICLE_BASE_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/core/binder_common.hpp>

#include <type_traits>

namespace godot {

// C++20 concept: an arithmetic scalar (used to type-check speed storage).
template <typename T>
concept ArithmeticScalar = std::is_arithmetic_v<T>;

// Thin Godot-facing shell around the vehicle simulation.
//
// The real vehicle architecture (composition over inheritance) lives elsewhere;
// this class only exposes a minimal property/method surface so the GDExtension
// registers and runs end-to-end. It is intentionally NOT the vehicle design.
class FdvVehicleBase : public Node3D {
    GDCLASS(FdvVehicleBase, Node3D)

    // km/h -> m/s conversion factor.
    static constexpr double KMH_TO_MPS = 1.0 / 3.6;

    double speed_kmh = 0.0;

    static_assert(KMH_TO_MPS > 0.0, "KMH_TO_MPS must be positive");
    static_assert(ArithmeticScalar<decltype(speed_kmh)>, "speed_kmh must be an arithmetic scalar");

protected:
    static void _bind_methods();

public:
    FdvVehicleBase() = default;
    ~FdvVehicleBase() = default;

    void set_speed_kmh(double p_speed_kmh);
    [[nodiscard]] double get_speed_kmh() const;

    // Exposed to GDScript for smoke-testing the binding.
    [[nodiscard]] double speed_mps() const;
};

} // namespace godot

#endif // FDV_VEHICLE_BASE_H
