# 3D Physics Engine

Writing a 3D rigid-body physics engine from scratch in C++.

## Bringup
Run `raylib` renderer:
```sh
cmake -S . -B build -DPHYS_BUILD_SANDBOX=ON
cmake --build build --target phys_sandbox
./build/phys_sandbox
```

The sandbox uses a muted object palette, a one-unit ground grid, and a compact
object/contact-count overlay. Contact normals are shown in pale gold; dark sphere
markers make rotation visible. It requests 4x MSAA where supported. Press `Esc` to quit.
Shapes and box outlines follow the full collider pose, including body rotation
and collider-local offsets and rotations.

The overlay uses [IBM Plex Sans](https://github.com/IBM/plex/tree/78cd4223d8de9fcb78cba84eadecb269c56093c5),
bundled under the [SIL Open Font License](assets/fonts/LICENSE.txt). CMake copies
the font and license to `assets/fonts` beside the executable; keep that directory
with the executable when moving it. No system font installation is needed.

For benchmarking:
```sh
cmake -S . -B build/release-bench -DCMAKE_BUILD_TYPE=Release \
  -DPHYS_BUILD_SANDBOX=OFF -DPHYS_BUILD_BENCHMARKS=ON
cmake --build build/release-bench --target phys_collision_bench -j 4
./build/release-bench/phys_collision_bench
```

See [performance history](docs/performance.md) for Debug/Release baselines,
the measured sweep-and-prune gains, methodology, and reproduction commands.
The default headless scaling scene is a seeded 50/50 mix of spheres and boxes
over one static floor. Every size runs 240 warmup steps and 1,200 measured steps
at 120 Hz: two simulated seconds of warmup plus ten measured seconds.
Body-count labels exclude the additional floor.
The CLI is `[measured_steps] [sap|grid|tree] [mixed|spheres] [warmup_steps]`;
counts are exact at every size, without adaptive caps. For example,
`phys_collision_bench 1200 tree` uses the default mixed scene and warmup with
the tree backend; `phys_collision_bench 10 tree spheres 0` explicitly selects
a short, unwarmed sphere-only run.
The uniform grid and persistent dynamic AABB tree are experimental; SAP remains
the engine default. `phys_broadphase_compare` compares all three backends on
static and moving AABB layouts, including tree maintenance and initial construction.
For optimized, symbolized CPU sampling of moving spheres and settled box stacks,
see [CPU sampling](docs/performance.md#cpu-sampling).

## Sleeping and waking

Sleeping is opt-in and works with all three broadphase backends:

```cpp
world.setSleepingEnabled(true);
if (auto *body = world.getBody(handle)) {
    bool sleeping = body->isSleeping();
    body->applyLinearImpulse({1, 0, 0}); // Automatically wakes the body.
}
```

Contact-connected dynamic bodies sleep together after every member stays at or
below 0.05 units/s linear speed and 0.05 rad/s angular speed for 0.5 simulation
seconds without a wake request. Sleeping zeroes their velocities and skips
integration and constraint solving. Their colliders and contact geometry remain
in collision detection and `world.contacts()`.

Nonzero forces/impulses, velocity setters, and `body->wakeUp()` wake the body
immediately and its connected island on the next step. Contact with an awake
body also wakes an island before solving. Body property changes, collider edits,
support removal, and gravity changes are detected as wake events. A shared static
floor does not connect independent sleeping islands.

`world.setSleepingEnabled(false)` wakes all bodies and restores always-active
simulation. `lastStepStats()` reports awake/sleeping dynamic-body counts and the
number of solved manifolds. Compare the settled-box workload with
`phys_cpu_profile boxes 5` and `phys_cpu_profile boxes 5 --sleep`; see
[sleeping measurements](docs/performance.md#sleeping-islands).

## Pipeline

```text
PhysicsWorld::step(dt)
    |
    v
Integrate forces into linear / angular velocities
    |
    v
Update collider world transforms and AABBs
    |
    v
Broad phase -> candidate pairs
    |
    v
Narrow phase -> contact manifolds
    |
    v
Constraint solver -> corrected velocities
    |
    v
Integrate positions / orientations
```

See [architecture](docs/architecture.md),
[collision pipeline](docs/collision_pipeline.md), and
[solver notes](docs/solver_notes.md) for intended boundaries and open decisions.

## Roadmap

| Stage | Scope |
| --- | --- |
| V0 | Basic math, rigid bodies, gravity, semi-implicit Euler; then a simple debug renderer |
| V1 | Spheres, AABBs, sphere-sphere contacts, basic impulse response |
| V2 | Oriented boxes, SAT, angular dynamics, inertia, friction, multi-point manifolds |
| V3 | Improved broad phase, sequential impulses, stable stacking, instrumentation |
| V4 | Evaluate continuous collision detection and other differentiators; GJK + EPA and an experimental dynamic AABB tree are implemented |
