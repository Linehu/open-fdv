# open-fdv — Vehicle Architecture Design

> Status: design proposal. Scope: Godot 4 GDExtension (C++20, godot-cpp).
> This document is design only; it creates no code and touches nothing outside docs/.

## 0. Purpose and principles

This document defines the architecture of the open-fdv vehicle module: a GDExtension that gives Godot 4 a data-driven, composable vehicle simulation. It is built first for Fatal-Delivery, an open world delivery driving game, so the priorities are tunable handling, stable physics at speed, many vehicles on screen, and a clean separation that lets the simulation run in unit tests with no Godot at all.

Non-negotiable principles, applied throughout:

1. **Godot at the boundary only.** Godot types and inheritance appear only in the thin node shell and in the physics backend. Every component is pure C++20 with no Godot dependency.
2. **Composition over inheritance.** There is no `Vehicle` base and no `Car -> Sedan -> SportsCar` chain. A vehicle is an aggregate of components owned by value.
3. **Strong types and Result.** No `bool success, T& out` pairs. Use `Result<T, VehicleError>`, `std::optional`, and `std::variant`.
4. **Properties and invariants.** Everything tunable is a ClassDB property; every invariant is enforced by `static_assert`/`concept` at compile time and by assertions in setters at runtime.
5. **The simulation is a pure function.** `VehicleCore::step` maps (inputs, world sample, prior state) to (new state, commands). This is the property that makes headless tests and deterministic replay possible.

The one unavoidable boundary, stated explicitly: to be a Godot object a class MUST inherit a Godot base (`Node`, `Node3D`, `RigidBody3D`, `RefCounted`, ...). We accept exactly one such inheritance at the shell (`OpenFdvVehicle3D : godot::RigidBody3D`) and one abstract interface for physics. No internal logic inherits from Godot.

## 1. Component decomposition

A vehicle is decomposed into nine parts. Each is an algorithm object with no hidden state: it reads a slice of `VehicleState` and writes its outputs. Persistent values (engine rpm, gear, wheel angular velocities, suspension travel) live in a single `VehicleState` owned by `VehicleCore`, never inside a component.

### 1.1 Summary

| Component | Responsibility | Key inputs | Key outputs | Owned by |
|---|---|---|---|---|
| Chassis | Inertia, mass, CoM, geometry, mounting points | (config) | `ChassisSpec`, wheel positions | `VehicleCore` (the `RigidBody3D` is the body) |
| Engine | Torque vs rpm, idle, rev limiter, throttle | throttle, feedback rpm, clutch state | torque, rpm, power | `VehicleCore` |
| Gearbox | Gear ratios, clutch slip, shift policy | gear, input/output rpm, clutch | output torque, constrained input rpm | `VehicleCore` |
| Differential | Torque split across wheels | input torque, wheel speeds | per-wheel torque | `VehicleCore` |
| Wheel (tire) | Longitudinal + lateral force from slip | drive/brake torque, steer, normal load, slip | contact forces, slip ratio/angle | `VehicleCore` (array of 4) |
| Suspension | Spring + damper along travel | contact hit, wheel velocity, travel | spring/damper force, normal load | `VehicleCore` (array of 4) |
| Steering | Ackermann geometry, rate limit, assist | steer input, speed | per-wheel steer angle | `VehicleCore` |
| Brakes | Bias, handbrake, ABS | brake input, wheel lock state | per-wheel brake torque | `VehicleCore` |
| Aerodynamics | Drag + downforce | speed, attitude | drag force, downforce | `VehicleCore` |

### 1.2 Chassis

Responsibility: define the rigid body — mass, inertia tensor, center of mass, wheelbase, track, and the geometric positions where wheels, suspension, and aero forces attach. The chassis has no `step`; it is the frame the rest of the simulation is bolted to.

- Inputs: none (configuration).
- Outputs: `ChassisSpec` (static) and wheel mounting positions `std::array<Vec3, 4>` computed from wheelbase, track, and ride height.
- Ownership: the `ChassisSpec` is owned by `VehicleCore`; the physics body is the `RigidBody3D` that the shell extends. The shell pushes mass, inertia, and center of mass into the Godot body on `_ready` and whenever a property changes.

### 1.3 Engine

Responsibility: convert a throttle demand into crank torque using a torque curve, subject to rotational inertia, an idle controller, and a soft rev limiter. A turbo/boost model is deferred.

- Inputs: `EngineInput { throttle, feedback_rpm, clutch_engaged }`.
- Outputs: `EngineOutput { torque, rpm, power, fuel_rate_gs }`.
- Ownership: composed by value in `VehicleCore`.

The crank speed is NOT free. When the clutch is engaged, `feedback_rpm` is the wheel speed reflected through the gearbox, so the engine either accelerates the car or is dragged down by it. When the clutch is disengaged, the crank integrates from net torque against its own inertia. See Section 4.

### 1.4 Gearbox (transmission)

Responsibility: apply gear ratios and model the clutch as a torque-limited coupling between crank and output shaft. Owns the shift policy (manual or automatic) and reports clutch slip.

- Inputs: `GearboxInput { gear, input_rpm, output_rpm, clutch_engagement, shift_up, shift_down }`.
- Outputs: `GearboxOutput { constrained_input_rpm, output_torque, gear_ratio, clutch_slip }`.
- Ownership: composed by value in `VehicleCore`; the current gear is part of `VehicleState`.

### 1.5 Differential (drivetrain)

Responsibility: split the single transmission output across the driven wheels. Three behaviors behind a variant: open (equal torque, free speed split), locked (equal speed), limited-slip (bias ratio caps the torque difference).

- Inputs: `DifferentialInput { input_torque, input_rpm, wheel_speeds[4] }`.
- Outputs: `DifferentialOutput { per_wheel_torque[4] }`.
- Ownership: composed by value in `VehicleCore`. AWD vehicles add a center split plus front and rear differentials; the first slice is RWD with a single open differential.

### 1.6 Wheel (tire)

Responsibility: turn drive torque, brake torque, steer angle, and normal load into ground reaction forces using a slip-based tire model (Pacejka-lite to start). This produces the forces that actually move the chassis.

- Inputs: `WheelInput { drive_torque, brake_torque, steer_angle, normal_load, camber, surface_grip, wheel_velocity, chassis_velocity, contact }`.
- Outputs: `WheelOutput { longitudinal_force, lateral_force, angular_velocity, slip_ratio, slip_angle }`.
- Ownership: `std::array<WheelSim, 4>` owned by `VehicleCore`.

### 1.7 Suspension

Responsibility: spring and damper force along the suspension travel axis. The raycast that finds the ground contact is performed by the backend; the spring math stays pure.

- Inputs: `SuspensionInput { contact: optional<RaycastHit>, travel, travel_velocity }`.
- Outputs: `SuspensionOutput { spring_force, damper_force, total_force, compressed }`.
- Ownership: `std::array<SuspensionSim, 4>` owned by `VehicleCore`.

### 1.8 Steering

Responsibility: map a normalized steer input to per-wheel angles via Ackermann geometry, with steering-rate limiting and speed sensitivity so high-speed inputs do not flip the car.

- Inputs: `SteeringInput { steer, speed, steer_rate }`.
- Outputs: per-wheel steer angle (`std::array<Angle, 4>`).
- Ownership: composed by value in `VehicleCore`.

### 1.9 Brakes

Responsibility: map brake and handbrake demand to per-wheel brake torque with front/rear bias and an optional ABS assist that releases a wheel about to lock.

- Inputs: `BrakeInput { brake, handbrake, speed, wheel_lock_state }`.
- Outputs: `std::array<Torque, 4>` of brake torque (sign opposite to drive torque).
- Ownership: composed by value in `VehicleCore`.

### 1.10 Aerodynamics

Responsibility: drag against the velocity vector and speed-dependent downforce distributed to the wheels (which increases grip).

- Inputs: `AeroInput { speed, angular_velocity, air_density, attitude }`.
- Outputs: `AeroOutput { drag_force, downforce, downforce_per_wheel[4] }`.
- Ownership: composed by value in `VehicleCore`.

### 1.11 Ownership diagram

```text
OpenFdvVehicle3D : godot::RigidBody3D      (boundary inheritance, thin shell)
  |
  +-- ofdv::VehicleCore core_               (composition, owned by value)
  |     |
  |     +-- ChassisSpec        chassis_     (config)
  |     +-- EngineSim          engine_
  |     +-- GearboxSim         gearbox_
  |     +-- Differential       differential_
  |     +-- SteeringSim        steering_
  |     +-- BrakeSim           brakes_
  |     +-- AeroSim            aero_
  |     +-- std::array<WheelSim, 4>      wheels_
  |     +-- std::array<SuspensionSim, 4> suspensions_
  |     +-- VehicleState       state_       (all mutable sim state)
  |
  +-- std::unique_ptr<IPhysicsBackend> backend_  (pluggable; injected)
```

## 2. The thin Godot shell

The shell is `OpenFdvVehicle3D`, and it extends exactly one Godot class: `godot::RigidBody3D`. Rationale: `RigidBody3D` is a real dynamic body (mass, inertia, velocity) and it exposes `_integrate_forces`, the correct hook for applying forces inside the physics step.

The shell has three jobs and nothing more:

1. Own the `VehicleCore` (composition).
2. Map ClassDB properties to `VehicleCore` spec/state, and push body parameters such as mass and center of mass into the `RigidBody3D`.
3. Bridge Godot physics to the core: capture body state, sample suspension rays, run the core, apply the returned commands.

It contains no vehicle behavior: no torque math, no gear logic, no tire model lives here.

```cpp
class OpenFdvVehicle3D : public godot::RigidBody3D {
    GDCLASS(OpenFdvVehicle3D, godot::RigidBody3D)

    ofdv::VehicleCore          core_;
    ofdv::GodotRigidBodyBackend backend_;
    ofdv::VehicleCommands      pending_commands_;
    ofdv::DriverInput          input_;
    ofdv::SuspensionRays       rays_;

protected:
    static void _bind_methods();
    void _ready() override;
    void _physics_process(double delta) override;
    void _integrate_forces(godot::PhysicsDirectBodyState3D* state) override;

public:
    // Inspector surface: setters validate invariants, then write into core_.
    void set_mass(double v);                   double get_mass() const;
    void set_engine_max_torque(double v);      double get_engine_max_torque() const;
    void set_gear_ratios(godot::PackedFloat32Array v);
    godot::PackedFloat32Array get_gear_ratios() const;
    // ... one setter/getter pair per property (Section 6)
};
```

Tick delegation:

```cpp
void OpenFdvVehicle3D::_physics_process(double delta) {
    ofdv::ChassisState chassis = backend_.capture_state(this);  // from RigidBody3D
    ofdv::WorldSample  world   = backend_.sample(rays_);        // raycast, safe outside the step

    pending_commands_ = core_.step(float(delta), input_, world, chassis); // pure
}

void OpenFdvVehicle3D::_integrate_forces(godot::PhysicsDirectBodyState3D* state) {
    backend_.apply(state, pending_commands_);  // forces applied in-step, ordered
}
```

The split between `_physics_process` and `_integrate_forces` is deliberate: raycasts are discouraged inside `_integrate_forces`, so they happen in `_physics_process`, while force application happens in `_integrate_forces` for correct ordering relative to the physics solver. The cost is a one-tick latency between sampling and applying; this is acceptable and is the standard Godot vehicle pattern. See Section 7 for the alternative.

## 3. The physics interface abstraction

The core never calls Godot directly. It talks to the world through a narrow interface with two sides:

- **Sampling:** the backend turns the world into a `WorldSample` (chassis pose and velocity, suspension contact hits, surface grip).
- **Applying:** the backend consumes a `VehicleCommands` (a flat list of forces and torques) and pushes them into the physics body.

This keeps the same component logic reusable across backends: the Godot `RigidBody3D` backend for the game, a headless deterministic backend for unit tests, and later a fixed-point backend for networked replay.

```cpp
// Core-side math is Godot-free; the backend converts to and from godot::Vector3.
struct Vec3 { double x = 0.0, y = 0.0, z = 0.0; };

struct RaycastHit    { Vec3 point; Vec3 normal; Distance distance; };
struct SuspensionRay { Vec3 origin; Vec3 direction; Distance max_length; };
using SuspensionRays = std::array<SuspensionRay, 4>;

struct ChassisState {
    Vec3 position;
    Vec3 linear_velocity;
    Vec3 angular_velocity;        // body-frame rad/s (kept minimal here)
    Vec3 forward; Vec3 up; Vec3 right;   // orientation basis
};

struct WorldSample {
    ChassisState chassis;
    std::array<std::optional<RaycastHit>, 4> suspension_hits;
    std::array<double, 4> surface_grip;    // 0..1 friction modifier per wheel
};

struct ForceCommand  { Force force; Vec3 world_point; };
struct TorqueCommand { Torque torque; };

struct VehicleCommands {
    std::vector<ForceCommand>  forces;    // suspension, tire, aero forces
    std::vector<TorqueCommand> torques;   // wheel and drive torques
};

class IPhysicsBackend {
public:
    virtual ~IPhysicsBackend() = default;
    virtual ChassisState capture_state(const void* body) = 0;
    virtual WorldSample  sample(const SuspensionRays& rays) = 0;
    virtual void         apply(const void* body, const VehicleCommands& commands) = 0;
};
```

Concrete backends:

- `GodotRigidBodyBackend` — the production backend. `capture_state` reads `RigidBody3D`/`PhysicsDirectBodyState3D`; `sample` casts `PhysicsRayQueryParameters3D` via `PhysicsDirectSpaceState3D`; `apply` calls `apply_force`/`apply_torque` on the direct body state. All `Vec3` conversions happen only here.
- `HeadlessBackend` — a flat, deterministic backend used by tests. `sample` reads pre-scripted contacts; `apply` accumulates commands into a buffer so tests assert on the exact forces emitted. No Godot at all.

A `concept` enforces the contract so a wrong backend fails at compile time:

```cpp
template <typename B>
concept PhysicsBackendLike = requires(B& b, const SuspensionRays& r,
                                      const VehicleCommands& c, const void* body) {
    { b.capture_state(body) } -> std::same_as<ChassisState>;
    { b.sample(r) }            -> std::same_as<WorldSample>;
    { b.apply(body, c) }       -> std::same_as<void>;
};
```

Alternative considered and rejected: driving the Godot built-in `VehicleBody3D`/`VehicleWheel3D` nodes. Those bake in a specific raycast model, hide the math, and would make the shared component logic fight the node instead of driving it. We keep a custom `RigidBody3D` + raycast vehicle and expose the tunables ourselves.

## 4. Data flow

The simulation is an ordered pipeline inside `VehicleCore::step`. Order matters because each stage consumes the outputs of earlier stages, and the feedback loop (wheel speed -> crank speed) closes across a tick.

```text
DriverInput
   |
   +--> SteeringSim ---------------------> per-wheel steer angle
   |
   +--> SuspensionSim (x4) --------------> suspension force + normal load   (uses WorldSample contacts)
   |
   +--> AeroSim -------------------------> drag + downforce
   |
   +--> EngineSim  <-- feedback rpm --+   (reverse path: wheels -> gearbox -> crank)
   |        |                          |
   |        v                          |
   +--> GearboxSim (clutch) ----------> output torque + constrained crank rpm
   |        |
   |        v
   +--> Differential ------------------> per-wheel drive torque
   |
   +--> BrakeSim ----------------------> per-wheel brake torque
   |
   +--> WheelSim (x4) -----------------> longitudinal + lateral tire forces   (uses steer, normal load, drive/brake torque)
   |
   v
VehicleCommands  (forces + torques)  --->  backend.apply()
   |
   v
VehicleState (integrate: wheel angular velocities, suspension travel, crank rpm)
```

The feedback loop is the heart of the powertrain. In one direction, engine torque flows down to the wheels and accelerates the car. In the other direction, wheel speed is reflected back up through the gear ratio to constrain the crank speed. Concretely, `GearboxSim` computes:

```text
reflected_wheel_rpm = wheel_rpm * gear_ratio * final_drive_ratio

if clutch engaged:
    constrained_crank_rpm = reflected_wheel_rpm
    output_torque = clamp(engine_torque, -clutch_capacity, +clutch_capacity)
else:
    crank integrates freely from engine_torque - engine_inertia_loss
    output_torque = 0
```

Strong types make the direction of each quantity unambiguous (an `Rpm` can never be silently assigned to a `Torque`), which is where most hand-rolled vehicle bugs live.

All inputs are normalized (`double` in `[0,1]` or `[-1,1]`), and every physical quantity carries a unit tag, so a `Force` from the suspension and a `Torque` from the brakes cannot be confused.

## 5. Type and API sketch

Representative signatures only — not implementations.

### 5.1 Unit-tagged scalars

```cpp
template <typename Tag, typename Rep = double>
struct Quantity {
    Rep value{};
    constexpr explicit Quantity(Rep v = {}) : value(v) {}
    constexpr friend Quantity operator+(Quantity a, Quantity b) { return Quantity{a.value + b.value}; }
    constexpr friend Quantity operator-(Quantity a, Quantity b) { return Quantity{a.value - b.value}; }
    constexpr friend Quantity operator*(Quantity a, Rep s) { return Quantity{a.value * s}; }
    constexpr friend Quantity operator/(Quantity a, Rep s) { return Quantity{a.value / s}; }
    constexpr friend Rep operator/(Quantity a, Quantity b) { return a.value / b.value; }  // ratio
    constexpr auto operator<=>(const Quantity&) const = default;
};

struct RpmTag{}; struct TorqueTag{}; struct ForceTag{}; struct SpeedTag{};
struct AngleTag{}; struct DistanceTag{}; struct PowerTag{};

using Rpm      = Quantity<RpmTag>;      // rotational speed
using Torque   = Quantity<TorqueTag>;   // N.m
using Force    = Quantity<ForceTag>;    // N
using Speed    = Quantity<SpeedTag>;    // m/s
using Angle    = Quantity<AngleTag>;    // rad
using Distance = Quantity<DistanceTag>; // m
using Power    = Quantity<PowerTag>;    // W

constexpr Rpm to_rpm(double rad_s)      { return Rpm{rad_s * 30.0 / 3.141592653589793}; }
constexpr double to_rad_s(Rpm rpm)      { return rpm.value * 3.141592653589793 / 30.0; }
constexpr Speed to_speed_ms(double kmh) { return Speed{kmh * 1000.0 / 3600.0}; }
constexpr Power power_from(Torque t, Rpm r) { return Power{t.value * to_rad_s(r)}; }
```

### 5.2 Result

```cpp
enum class VehicleError { None, GearOutOfRange, ClutchDisengaged, NoContact, InvalidSpec, DivideByZero };

template <typename T>
class Result {
    std::variant<T, VehicleError> data_;
public:
    static Result ok(T v)              { return Result{std::in_place_index<0>, std::move(v)}; }
    static Result err(VehicleError e)  { return Result{std::in_place_index<1>, e}; }

    [[nodiscard]] bool has_value() const      { return std::holds_alternative<T>(data_); }
    [[nodiscard]] const T& value() const      { return std::get<T>(data_); }
    [[nodiscard]] T value_or(T fallback) const { return has_value() ? value() : std::move(fallback); }
    [[nodiscard]] VehicleError error() const  { return std::get<VehicleError>(data_); }
    [[nodiscard]] explicit operator bool() const { return has_value(); }
};
```

### 5.3 Variant for differential mode and gear

```cpp
enum class Gear : int8_t { Reverse = -1, Neutral = 0, G1 = 1, G2 = 2, G3 = 3, G4 = 4, G5 = 5, G6 = 6 };

struct OpenDiff {};
struct LockedDiff {};
struct LimitedSlipDiff { double bias_ratio; };   // must be >= 1

using DifferentialMode = std::variant<OpenDiff, LockedDiff, LimitedSlipDiff>;
```

### 5.4 Concepts

```cpp
template <typename C, typename In, typename Out>
concept Component = requires(C& c, float dt, const In& in, VehicleState& s) {
    { c.step(dt, in, s) } -> std::same_as<Out>;
};

static_assert(Component<EngineSim, EngineInput, EngineOutput>);
static_assert(Component<WheelSim, WheelInput, WheelOutput>);
```

### 5.5 Component signatures

```cpp
struct DriverInput {
    double throttle{0.0}; double brake{0.0}; double handbrake{0.0};
    double steer{0.0};                        // -1..1, left negative
    bool clutch_pressed{false};
    Gear gear{Gear::Neutral};
    bool shift_up{false}; bool shift_down{false};
};

struct TorqueCurvePoint { Rpm rpm; Torque torque; };

struct EngineSpec {
    std::vector<TorqueCurvePoint> torque_curve;   // sorted, strictly increasing rpm
    Rpm idle_rpm; Rpm redline_rpm; Rpm rev_limit_rpm;
    double rotational_inertia_kgm2{0.0};
    double throttle_response{5.0};                // 1/s smoothing
};

struct EngineInput  { double throttle{0.0}; Rpm feedback_rpm; bool clutch_engaged; };
struct EngineOutput { Torque torque; Rpm rpm; Power power; double fuel_rate_gs; };

class EngineSim {
public:
    // Stateless: reads rpm from VehicleState, writes the new rpm back.
    EngineOutput step(float dt, const EngineInput& in, VehicleState& s) const;
    static Result<EngineSpec> validate(const EngineSpec& spec);
};
```

`WheelSim` and `SuspensionSim` follow the same shape (stateless `step`, `validate` returning `Result`, and a `Spec` struct).

### 5.6 VehicleCore

```cpp
class VehicleCore {
public:
    // The pure transition: (inputs, world, chassis) -> commands, mutating state in place.
    VehicleCommands step(float dt, const DriverInput& in,
                         const WorldSample& world, const ChassisState& chassis);

    // Whole-vehicle invariant validation, run on load and on property change.
    Result<void> validate() const;

    VehicleSpec& spec();                 // grouped inspector specs
    const VehicleState& state() const;
};
```

`Result<void>` is a `Result` whose success carries no payload; it is how `validate()` reports a broken spec (wrong gear count, redline below idle, negative mass) instead of a bool.

## 6. Property and editor surface

Everything tunable is a ClassDB property on the shell; the shell pushes it into the matching `Spec`. Godot property hints give the inspector a usable slider with units.

| Property | Type | Hint / range | Invariant (asserted) |
|---|---|---|---|
| mass | float | 1..100000 kg | > 0 |
| inertia_tensor | Vector3 | — | each component > 0 |
| center_of_mass | Vector3 | — | within body bounds |
| wheelbase / track | float | m | > 0 |
| engine_max_torque | float | 0..10000 Nm | >= 0 |
| engine_max_power | float | 0..5000 kW | >= 0 |
| engine_idle_rpm / redline_rpm | float | rpm | idle < redline |
| torque_curve | Curve | — | non-empty, monotonic rpm |
| gear_ratios | PackedFloat32Array | — | size >= 2, all > 0 |
| final_drive_ratio | float | 0.1..10 | > 0 |
| transmission_type | enum (Manual/Auto) | — | — |
| differential_type | enum (Open/Locked/LSD) | — | — |
| lsd_bias_ratio | float | 1..10 | >= 1 |
| suspension_stiffness | float | N/m | > 0 |
| damper_compression / rebound | float | N.s/m | >= 0 |
| suspension_travel | float | m | > 0 |
| ride_height | float | m | > 0 |
| wheel_radius | float | m | > 0 |
| wheel_mass | float | kg | > 0 |
| tire_grip | float | 0..2 | > 0 |
| tire_stiffness_long / lat | float | — | > 0 |
| max_steer_angle | float | deg | > 0 |
| ackermann_percent | float | 0..1 | in [0,1] |
| brake_torque | float | Nm | >= 0 |
| brake_bias_front | float | 0..1 | in [0,1] |
| handbrake_torque | float | Nm | >= 0 |
| drag_coefficient | float | 0..2 | >= 0 |
| downforce_coefficient | float | 0..10 | >= 0 |

Enforcement has three layers:

1. **Compile time** — `static_assert`/`concept` catch type errors (for example a `Torque` passed where a `Force` is expected) and struct-layout assumptions.
2. **Setters** — every setter runs `ERR_FAIL_COND_MSG` (or clamps where clamping is the intended UX) before writing, so the editor cannot produce an invalid spec.
3. **validate()** — `VehicleCore::validate()` returns `Result<void>` and is invoked on `_ready` and on property change; it rejects cross-field invariants (redline > idle, gear count matches the transmission, ride height <= suspension travel).

Representative registration and setter:

```cpp
void OpenFdvVehicle3D::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_engine_max_torque", "value"), &OpenFdvVehicle3D::set_engine_max_torque);
    ClassDB::bind_method(D_METHOD("get_engine_max_torque"), &OpenFdvVehicle3D::get_engine_max_torque);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "engine_max_torque",
        PROPERTY_HINT_RANGE, "0,10000,0.1,suffix:Nm"),
        "set_engine_max_torque", "get_engine_max_torque");
}

void OpenFdvVehicle3D::set_engine_max_torque(double v) {
    ERR_FAIL_COND_MSG(v < 0.0, "engine_max_torque must be >= 0");
    core_.spec().engine_max_torque = v;
}
```

(On Godot 4.3+, `_validate_property` can annotate invalid values in the inspector; on 4.2 we rely on setters only.)

## 7. Open questions and trade-offs

1. **Custom raycast vehicle vs Godot `VehicleBody3D`.** Decision: custom `RigidBody3D` + raycast. `VehicleBody3D` bakes in a model we cannot control and would make the shared, testable component logic pointless. Trade-off: we re-implement suspension rays ourselves.
2. **Raycast timing.** Sampling in `_physics_process` (safe, one-tick latency) vs `_integrate_forces` (immediate, discouraged by Godot). Decision: sample in `_physics_process`, apply in `_integrate_forces`.
3. **Tire fidelity.** Coulomb + slip curve (Pacejka-lite) vs full Magic Formula. Decision: start with a lite model behind the `WheelSim` interface so it can be upgraded without touching anything else.
4. **Sub-stepping.** Stiff suspension at 60 Hz physics can go unstable. Decision: `VehicleCore::step` supports N sub-steps (default 2), configurable per vehicle.
5. **Determinism and own math types.** Using `ofdv::Vec3` instead of `godot::Vector3` inside the core keeps it Godot-free and testable, at the cost of a small conversion at the boundary. Decision: accept the cost.
6. **Engine-drivetrain coupling.** Rigid kinematic lock vs clutch-slip + inertia. Decision: clutch-slip model (Section 4); a locked mode is a special case with a very high clutch capacity.
7. **Automatic gearbox.** Simple rpm-threshold shift vs torque-demand policy. Decision: manual + a simple rpm-threshold automatic first; torque-demand later.
8. **Threading.** Single-threaded first; a worker pool over N vehicles is a later optimization that the pure `step` makes straightforward.

### Recommended first implementation slice

**Milestone 0 — foundations:** `Vec3`, `Quantity`, `Result`, `VehicleState`, `VehicleCommands`, `IPhysicsBackend`, `GodotRigidBodyBackend`, `HeadlessBackend`, and the `OpenFdvVehicle3D` shell. Prove it with a headless test that steps a body and asserts on the emitted forces.

**Milestone 1 — suspension + rolling:** four raycast suspensions plus a longitudinal tire (Coulomb). The car settles under gravity, rolls, and stops.

**Milestone 2 — steering + brakes:** Ackermann steering and brake bias. The car can be driven and turned at low speed on flat ground.

**Milestone 3 — powertrain:** engine torque curve, manual + simple automatic gearbox, RWD open differential. The car accelerates to a stable top speed.

**Deferred:** aerodynamics, turbo/boost, ABS/traction control, AWD and limited-slip differentials, full tire model, replay determinism, networking.

## 8. Proposed source layout (documentation only)

This is a target layout for the build worker to follow; no files are created by this document.

```text
src/
  core/          # Godot-free simulation (Vec3, Quantity, Result, components, VehicleCore)
    math.hpp
    quantity.hpp
    result.hpp
    state.hpp
    spec.hpp
    engine.hpp
    gearbox.hpp
    differential.hpp
    wheel.hpp
    suspension.hpp
    steering.hpp
    brakes.hpp
    aero.hpp
    vehicle_core.hpp
  backend/       # physics boundary
    backend.hpp          # IPhysicsBackend + concepts
    godot_backend.hpp    # GodotRigidBodyBackend (only file importing godot-cpp physics)
    headless_backend.hpp # test backend
  node/          # thin shell (Godot boundary)
    open_fdv_vehicle_3d.hpp
    open_fdv_vehicle_3d.cpp
    register.cpp
```

That is the whole design. It is deliberately small at the boundary and rich in the core, so the simulation can be proven in unit tests before the first Godot scene loads.
