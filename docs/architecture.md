# Architecture

Design principles behind the engine.

## Boundaries

| Module | Responsibility |
| --- | --- |
| Core | Shared numerical types, constants, and invariant policy |
| Math | Geometry operations with no simulation dependencies |
| Dynamics | Body state, force/torque accumulation, mass/inertia, integration |
| Collision | Geometry, bounds, candidate generation, contact generation |
| Solver | Velocity response from contact constraints |
| World | Simulation ownership, lifetime management, and stage ordering |

## Body vs. Collider

We define a clean boundary between `RigidBody` and `Collider`:
- **Rigid Body:** Physical object state and behavior
- **Collider:** Geometric shape used for collision detection

The rigid body moves; the collider defines the space that moves with it. For example, a rolling ball has 
a rigid body containing its mass, position, velocity, and rotation. It also has a sphere collider used to detect contact with the floor.  

The collider doesn't independently move or have velocity. Its world transform is derived from the rigid body’s transform plus the collider’s local transform.

## Step

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

## Hot Path

Efficient, real-time coding principles are a major point of interest and learning here.

`PhysicsWorld::step()` is the simulation "hot path":
integration, broad phase, narrow phase, contact generation, and solving. Body
and collider creation is usually a cold path, so it may favor simpler, clearer
allocation and ownership. The step path should favor predictable work:

- reserve body, pair, manifold, and contact buffers during initialization;
- reuse storage and target zero allocations per step;
- avoid file I/O, logging, and unnecessary locking;
- prefer contiguous iteration and explicit handling of inactive entries;
- measure stage costs and allocation counts rather than assuming improvements.

The important distinction is not whether a loop is hot, but whether its work is
unbounded or unpredictable. Future performance studies may compare array-of-
structures with structure-of-arrays storage, measure parallel narrow-phase
scaling, and test determinism by repeating fixed-seed simulations and comparing
their resulting state.

The eventual debugger should support pause, single-step, and reset, and display
collider wireframes, AABBs, normals, contacts, centers of mass, angular
velocities, and candidate pairs. None of these controls or overlays exist yet.
