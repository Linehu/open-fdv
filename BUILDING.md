# Building open-fdv

Godot 4 GDExtension (C++) module. Runtime wiring lives in `fdv.gdextension`.

## Prerequisites

- **C++20 compiler** — GCC 12+ or Clang 16+ (tested with GCC 14.2).
- **SCons 4.x** — the build driver (`scons`).
- **Python 3.8+** — SCons runs on it (tested with 3.13).
- **Git** — to fetch godot-cpp.
- **Godot 4.4.x editor** — only needed to *run* the extension inside Godot.
  The C++ library itself builds without a Godot binary because godot-cpp ships a
  pre-generated `gdextension/extension_api.json` + `gdextension_interface.h`.

No other Python packages are required: godot-cpp's binding generator uses only
the standard library.

## 1. Get godot-cpp (submodule, pinned to Godot 4.4.1)

```sh
git submodule update --init --recursive
```

If the submodule is missing, add it manually:

```sh
git submodule add https://github.com/godotengine/godot-cpp godot-cpp
git -C godot-cpp checkout godot-4.4.1-stable
```

## 2. Build

From the repository root:

```sh
scons platform=linux target=template_debug -j$(nproc)
```

This produces:

- `godot-cpp/bin/libgodot-cpp.linux.template_debug.x86_64.a` — bindings (static).
- `bin/libfdv.linux.template_debug.x86_64.so` — this extension.

Release build: `scons platform=linux target=template_release -j$(nproc)`.

## 3. Use in a Godot project

Copy `bin/` and `fdv.gdextension` into a Godot 4.4 project and open it.
`FdvVehicleBase` (extends `Node3D`) is then available with the `speed_kmh`
property and `speed_mps()` method.

## Notes

- `compatibility_minimum` in `fdv.gdextension` is `4.4`; keep the godot-cpp
  tag in sync with your Godot version.
- Regenerate the extension API only when changing Godot versions:
  `godot --dump-extension-api --dump-gdextension-interface`, then copy the two
  outputs into `godot-cpp/gdextension/`.
