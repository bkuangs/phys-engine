# 3D Physics Engine

Writing a 3D rigid-body physics engine from scratch in C++.

## Bringup
Run `raylib` renderer:
```sh
cmake -S . -B build -DPHYS_BUILD_SANDBOX=ON
cmake --build build --target phys_sandbox
./build/phys_sandbox
```

For benchmarking:
```sh
cmake -S . -B build -DPHYS_BUILD_BENCHMARKS=ON
cmake --build build --config Debug
```

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
