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

# The shared rules core lives one level up in shared/. It is plain C, compiled
# straight into this library rather than linked separately, so the Godot build
# has no runtime dependency beyond its own .dll -- the same sources the Unity
# build compiles to fd_core.dll and calls through P/Invoke.
#
# FD_CORE_STATIC must be defined: without it fd_core.h declares the functions
# as dllimport, the compiler emits __imp_ symbols, and the link fails with every
# core function undefined. The macro says "the implementation is already in this
# binary", not "there is a DLL next door to import from".
env.Append(CPPPATH=["../shared/include"])
env.Append(CFLAGS=["-std=c11"])
env.Append(CPPDEFINES=["FD_CORE_STATIC"])

sources = Glob("src/*.cpp") + Glob("../shared/src/*.c")

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
