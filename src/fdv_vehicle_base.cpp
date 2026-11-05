#include "fdv_vehicle_base.h"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void FdvVehicleBase::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_speed_kmh", "speed_kmh"), &FdvVehicleBase::set_speed_kmh);
    ClassDB::bind_method(D_METHOD("get_speed_kmh"), &FdvVehicleBase::get_speed_kmh);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed_kmh"), "set_speed_kmh", "get_speed_kmh");

    ClassDB::bind_method(D_METHOD("speed_mps"), &FdvVehicleBase::speed_mps);
}

void FdvVehicleBase::set_speed_kmh(double p_speed_kmh) {
    speed_kmh = p_speed_kmh;
}

double FdvVehicleBase::get_speed_kmh() const {
    return speed_kmh;
}

double FdvVehicleBase::speed_mps() const {
    return speed_kmh * KMH_TO_MPS;
}
