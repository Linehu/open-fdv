#ifndef FDV_REGISTER_TYPES_H
#define FDV_REGISTER_TYPES_H

#include <godot_cpp/core/class_db.hpp>

namespace fdv {

void initialize_module(godot::ModuleInitializationLevel p_level);
void terminate_module(godot::ModuleInitializationLevel p_level);

} // namespace fdv

#endif // FDV_REGISTER_TYPES_H
