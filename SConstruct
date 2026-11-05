#!/usr/bin/env python
# SCons build script for the open-fdv GDExtension.
#
# This imports the godot-cpp build environment (which compiles the bindings and
# the static libgodot-cpp library), then builds our shared library from src/.

env = SConscript("godot-cpp/SConstruct")

# Clone so our toolchain tweaks (C++20) don't disturb godot-cpp's own targets.
env = env.Clone()

# Add the extension source include path and force C++20 for our own sources
# (godot-cpp itself defaults to C++17).
env.Append(CPPPATH=["src/"])
env.Append(CXXFLAGS=["-std=c++20"])

sources = Glob("src/*.cpp")

if env["platform"] == "macos":
    library = env.SharedLibrary(
        "bin/libfdv.{}.{}.framework/libfdv.{}.{}".format(
            env["platform"], env["target"], env["platform"], env["target"]
        ),
        source=sources,
    )
else:
    library = env.SharedLibrary(
        "bin/libfdv{}{}".format(env["suffix"], env["SHLIBSUFFIX"]),
        source=sources,
    )

Default(library)
