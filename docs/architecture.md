# Architecture

This document describes the intended design, not implemented behavior.
The repository currently contains build plumbing and declarations only.

## Boundaries

| Module | Responsibility |
| --- | --- |
| Core | Shared numerical types, constants, and invariant policy |
| Math | Geometry operations with no simulation dependencies |
| Dynamics | Body state, force/torque accumulation, mass/inertia, integration |
| Collision | Geometry, bounds, candidate generation, contact generation |
| Solver | Velocity response from contact constraints |
| World | Simulation ownership, lifetime management, and stage ordering |
| Sandbox | Scenes, debug visualization, and user controls |
| Benchmarks | Reproducible workloads and timing outside the engine algorithms |

The library must not depend on the sandbox or a rendering backend. Collision
queries must not apply impulses. The solver must not discover collisions.
The world coordinates those responsibilities rather than implementing them.

## Body and collider separation

A rigid body represents motion: pose, linear/angular velocity, accumulated
force/torque, and mass/inertia properties. A collider represents shape and local
placement relative to a body. Do not introduce shape-specific rigid-body
subclasses.

The design should leave room for several colliders on one body without
requiring compound-body support in the first implementation. Body/collider
ownership, stable references, creation, and removal semantics must be chosen
before exposing a usable world API. Raw pointers into a growing vector are not
a safe lifetime contract.

## Step pipeline

The desired orchestration is:

```text
step(dt)
    integrate forces / torques into velocities
    update collider world transforms and bounds from current body poses
    compute broad-phase candidate pairs
    generate narrow-phase contact manifolds
    prepare and solve contact constraints, updating velocities
    integrate corrected velocities into positions / orientations
```

Force integration and pose integration are separate stages of semi-implicit
Euler. The collision stage sees the current poses, not a continuous sweep.
Fast objects can therefore tunnel; continuous collision detection is a later,
explicit extension rather than an implied guarantee.

Before implementation, decide when accumulators are cleared and how cached
collider bounds are refreshed or invalidated after pose integration. Queries
and debug drawing must not accidentally expose stale bounds as current state.

The caller should eventually advance the world with a fixed simulation step;
wall-clock accumulation and rendering interpolation belong in the application.
Timestep validation and substep policy are still undecided.

## Deliberately open decisions

| Decision | Resolve before |
| --- | --- |
| Scalar precision, units, handedness, numerical tolerances | Math implementation |
| Matrix layout and multiplication convention; quaternion component/composition order | Rotation and transform implementation |
| Normalization and degenerate-input policy | Numerical API implementation |
| Static/dynamic body representation; mass/inverse-mass invariants | Rigid-body implementation |
| Body/collider storage, handle validity, deletion, and ownership | World implementation |
| Material ownership and friction/restitution combination | Contact response implementation |
| Contact normal direction, coordinate spaces, penetration sign | Narrow-phase implementation |
| Collision filtering, pair ordering, same-body/static-static policy | Broad-phase implementation |
| Rendering backend and external dependency policy | Visual sandbox implementation |

Forward declarations reserve names without settling data layouts, inheritance,
method signatures, or ownership. Do not replace missing algorithms with
success-shaped stubs.

## Instrumentation

Future instrumentation should expose counts and stage timings without coupling
physics to rendering: bodies, candidate pairs, manifolds, contact points, solver
iterations, and per-stage cost. Benchmarks should use fixed seeds and disclose
workload, build configuration, hardware, and algorithm settings.

The eventual debugger should support pause, single-step, and reset, and display
collider wireframes, AABBs, normals, contacts, centers of mass, angular
velocities, and candidate pairs. None of these controls or overlays exist yet.
