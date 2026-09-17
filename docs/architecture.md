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

### Opt-in sleeping islands

`PhysicsWorld::setSleepingEnabled(true)` adds sleep/wake orchestration around the
existing pipeline. The world joins contact-connected dynamic bodies using
slot-indexed disjoint sets. Static bodies remain collision participants but
never join two dynamic islands through a common floor.

Before velocity integration, the previous contact graph propagates wake requests
and support edits to connected bodies. After narrowphase, the current graph
wakes any island touched by an awake body or a changed static body. Bodies newly
woken in this second pass receive the same step's gravity/forces before solving;
already-awake bodies are not integrated twice. Candidate and manifold ordering
remain unchanged.

Each dynamic body must remain below the linear/angular sleep thresholds for
0.5 simulation seconds; an island uses its least-quiet member's time. The
thresholds are 0.05 world units/s and 0.05 rad/s. Sleep zeroes velocities, skips
integration and solving, and retains warm-start cache entries for later waking.
Internal solver impulses do not reset quiet timers; external forces, impulses,
velocity setters, and explicit `wakeUp()` requests do.

Body properties and collider geometry are publicly mutable. While sleeping is
enabled, the world compares them with end-of-step snapshots to detect edits
without treating mutable getter calls as wake requests. Lifetime operations wake
affected contacts before removing their supports; generation checks prevent
recycled slots from inheriting stale contact identities. Gravity changes wake
all bodies.

This implementation still recomputes bounds, broadphase pairs, and narrowphase
contacts for sleeping bodies. Contact geometry remains available to queries and
debug drawing; skipped manifolds apply no impulses that step. It does not add
parallel solving, CCD, or a sleeping-contact geometry cache. Disabling sleeping
wakes every body immediately and bypasses island/snapshot processing.

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
