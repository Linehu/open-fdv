#include "fdv_vehicle_base.h"

#include <godot_cpp/core/class_db.hpp>

auto fdv::VehicleBase::_bind_methods() -> void {
    godot::ClassDB::bind_method(godot::D_METHOD("set_speed_kmh", "speed_kmh"), &VehicleBase::set_speed_kmh);
    godot::ClassDB::bind_method(godot::D_METHOD("get_speed_kmh"), &VehicleBase::get_speed_kmh);
    ADD_PROPERTY(godot::PropertyInfo(godot::Variant::FLOAT, "speed_kmh"), "set_speed_kmh", "get_speed_kmh");

    godot::ClassDB::bind_method(godot::D_METHOD("speed_mps"), &VehicleBase::speed_mps);
}

auto fdv::VehicleBase::set_speed_kmh(double p_speed_kmh) -> void {
    speed_kmh = p_speed_kmh;
}

auto fdv::VehicleBase::get_speed_kmh() const -> double {
    return speed_kmh;
}

auto fdv::VehicleBase::speed_mps() const -> double {
    return speed_kmh * KMH_TO_MPS;
}
