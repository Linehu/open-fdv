# Code Style — open-fdv

The C++ style for this project. It is deliberately distinct from the author's C style (8-space indent, `/* */` comments, Allman braces); this file governs all C++ (Godot GDExtension) code here.

## Indentation

- One indent level is 4 spaces, expanded from tabs (expandtab). Never tabs, never 8 spaces.
- Each nested block adds another 4 spaces.

## Braces

- K&R: the opening brace stays on the same line as the class/function/control statement; the closing brace sits on its own line.

## Comments

- Use `//` line comments, never `/* */`.

## Naming

| Category | Style | Example |
|---|---|---|
| Classes | PascalCase | `VehicleBase` |
| Methods / variables / members | snake_case | `speed_mps`, `speed_kmh`, `set_speed_kmh` |
| Constants | SCREAMING_SNAKE_CASE | `KMH_TO_MPS` |
| Parameters | `p_` prefix | `p_speed_kmh`, `p_level` |
| Out-params | `r_` prefix | `r_initialization` |
| Header guards | `#ifndef NAME` … `#endif // NAME` | never `#pragma once` |

## Modern C++ (default, not decoration)

- Target **C++20**.
- Use **trailing return types** with the `auto` placeholder always present: `auto f(...) -> T` (`-> void`, `-> double`). A bare `f(...) -> void;` does not compile.
- Mark pure functions and getters `[[nodiscard]]`.
- Use `constexpr` for compile-time constants.
- Use `concept` for template constraints: `template <typename T> concept ArithmeticScalar = std::is_arithmetic_v<T>;`
- Enforce invariants with `static_assert`, always with a message.
- Prefer STL (`std::vector`/`std::array`/`std::optional`/`std::variant`/`std::span`) over raw arrays and manual memory.
- Prefer strong return types (`std::optional`/`std::variant`/Result) over `bool` plus out-params.

## Composition over inheritance

- Avoid is-a hierarchy hell. Godot/base-class inheritance stays at the boundary as a thin shell; business logic is composed from components.

## Godot GDExtension specifics

- Keep the module in a custom namespace (`namespace fdv`), not `namespace godot`; fully qualify every `godot::` type and never write `using namespace godot`.
- Write godot-cpp macros bare, never `godot::`-prefixed: `GDCLASS`, `GDREGISTER_CLASS`, `ADD_PROPERTY` expand to text beginning `::godot::`, so a prefix becomes `godot::::godot::` and fails to compile.
- Pass the `GDCLASS` inherits type fully qualified: `GDCLASS(VehicleBase, godot::Node3D)`.
- Write raw `GDExtension*` C types bare (global typedefs from `gdextension_interface.h`, not inside `godot::`).
- `D_METHOD` is the exception: it expands to `StringName(...)` with no leading `::`, so `godot::D_METHOD(...)` compiles and stays qualified.
- Expose tunables as ClassDB properties via `ADD_PROPERTY`.

## Class layout

- Order members: private data → `protected: _bind_methods()` → `public:` constructor/destructor/methods.

## Verification

- Rebuild and confirm no warnings.
- Confirm no tabs remain: `grep -rnP '\t' src/` returns nothing.
