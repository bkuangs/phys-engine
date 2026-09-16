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
cmake -S . -B build -DPHYS_BUILD_BENCHMARKS=ON
cmake --build build --config Debug
```

See [performance history](docs/performance.md) for Debug/Release baselines,
the measured sweep-and-prune gains, methodology, and reproduction commands.

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
| V4 | Choose a differentiator: GJK + EPA, continuous collision detection, or a dynamic AABB tree |
