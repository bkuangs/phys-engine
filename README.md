# phys-engine

A from-scratch 3D rigid-body physics engine project in C++20.

**Status: skeleton only.** The full intended directory layout is present, but no
math, simulation, collision detection, or solver behavior is implemented.
Headers reserve names with forward declarations or document future APIs;
source files contain no engine definitions. The sandbox and benchmark
executables only print that they are unimplemented.

## Architecture

The planned centerpiece is an explicit simulation pipeline:

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

Rigid bodies describe motion and mass properties; colliders describe geometry
and local placement. Collision detection produces contact geometry, not motion
response. The solver consumes that geometry, while the world orchestrates the
step. Debug rendering and instrumentation belong outside the core library.

See [architecture](docs/architecture.md),
[collision pipeline](docs/collision_pipeline.md), and
[solver notes](docs/solver_notes.md) for intended boundaries and open decisions.

## Build

Requires CMake 3.20 or newer and a C++20 compiler. The scaffold has no external
dependencies or dependency downloads. Build targets use portable CMake and
standard C++ for macOS, Linux, and Windows.

```sh
cmake -S . -B build
cmake --build build --config Debug
```

For single-configuration generators, select a configuration at configure time:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
```

Run the headless entry point with `./build/phys_sandbox`. Multi-configuration
generators place it under `build/Debug/`; Windows adds the `.exe` suffix.
There is no renderer, simulation loop, or interactive control handling yet.

| CMake option | Default | Purpose |
| --- | --- | --- |
| `PHYS_BUILD_SANDBOX` | `ON` | Build the headless sandbox entry point |
| `PHYS_BUILD_BENCHMARKS` | `OFF` | Build the two benchmark entry points |
| `BUILD_TESTING` | `ON` | Enable CTest and the test directory |

The library target is `phys`, with the alias `phys::phys`. It publishes
`include/` and the C++20 requirement to consumers. It has no usable physics API
or engine symbols yet.

## Tests and benchmarks

CTest is wired up, but **no tests are registered yet**:

```sh
ctest --test-dir build -C Debug --output-on-failure
```

Add tests under `tests/math`, `tests/collision`, `tests/dynamics`, and
`tests/solver`, registering executable targets in `tests/CMakeLists.txt`.
The initial testing approach is dependency-free CTest executables; passing
placeholder tests are deliberately not included.

```sh
cmake -S . -B build -DPHYS_BUILD_BENCHMARKS=ON
cmake --build build --config Debug
```

This adds `phys_broadphase_bench` and `phys_collision_bench`. Neither measures
performance yet; they are entry points for future reproducible workloads.

## Layout

| Path | Intended responsibility |
| --- | --- |
| `include/phys/core` | Scalar types, constants, assertions |
| `include/phys/math` | Vectors, matrices, quaternions, transforms |
| `include/phys/dynamics`, `src/dynamics` | Rigid bodies, mass properties, integration |
| `include/phys/collision`, `src/collision` | Shapes, queries, broad/narrow phase, manifolds |
| `include/phys/solver`, `src/solver` | Contact constraints and iterative impulse solving |
| `include/phys/world`, `src/world` | Ownership and simulation orchestration |
| `apps/sandbox` | Headless driver now; physics debugger and scenes later |
| `tests` | Focused numerical, geometry, and physical-invariant tests |
| `benchmarks` | Broad-phase and collision-query performance workloads |
| `assets/meshes` | Future debug/demo meshes |
| `docs` | Architecture and algorithm design notes |
| `third_party` | Reserved for explicitly chosen dependencies; currently empty |

## Planned progression

All items below are future work, not supported features.

| Stage | Scope |
| --- | --- |
| V0 | Basic math, rigid bodies, gravity, semi-implicit Euler; then a simple debug renderer |
| V1 | Spheres, AABBs, sphere-sphere contacts, basic impulse response |
| V2 | Oriented boxes, SAT, angular dynamics, inertia, friction, multi-point manifolds |
| V3 | Improved broad phase, sequential impulses, stable stacking, instrumentation |
| V4 | Choose a differentiator: GJK + EPA, continuous collision detection, or a dynamic AABB tree |

An initial all-pairs broad phase belongs with the first collision pipeline;
V3 improves that baseline rather than postponing candidate generation.
Reserved SAT, GJK, EPA, and convex-hull files do not commit to implementing
them before the earlier stages work.

Future evidence should include physical-invariant tests, comparable benchmark
results, and captures of real debug visualization. No feature claims,
performance numbers, or demo GIFs are included before those exist.

## License

No license has been selected. This repository intentionally has no `LICENSE`
file.
