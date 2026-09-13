# Solver notes

The solver is not implemented. These notes define the intended direction
without committing to an untested API.

## Inputs and responsibilities

The solver consumes contact manifolds and body state, constructs constraints,
and updates linear/angular velocities. It does not perform collision
detection, integrate external forces, or own the simulation loop.

The eventual contact response should account for relative velocity at the
contact point, inverse mass, and world-space inverse inertia. A force changes
momentum over time; an impulse changes it directly. Off-center forces and
impulses can affect angular motion as well as translation.

## Planned sequential impulse pass

```text
Prepare contact constraints and effective masses
Optionally warm start from persistent contact impulses
Repeat for a configured iteration count:
    solve normal impulses with a nonnegative accumulated impulse
    solve tangential impulses under a Coulomb friction limit
Write back any persistent solver state
```

Begin with simple normal impulse response before adding friction, multiple
contacts, and caching. Sequential impulse solving is the intended iterative
extension, not implemented functionality.

## Decisions to resolve during implementation

| Topic | Required contract |
| --- | --- |
| Restitution | Material combination and low-speed bounce threshold |
| Friction | Material combination, tangent basis, and impulse limits |
| Penetration stabilization | Slop, bias policy, and control of injected energy |
| Inertia | Conversion of local inverse inertia into world space |
| Static bodies | Zero inverse mass/inertia and immovable-state rules |
| Warm starting | Contact identity, cache lifetime, and timestep changes |
| Convergence | Iteration budget and observable solver statistics |
| Sleeping | Explicit later scope, not a substitute for stable contacts |

Stable stacking requires consistent contact generation and solver behavior.
Iteration count alone does not fix incorrect normals, stale inertia, or
unstable manifolds.

## Future tests

For isolated bodies with no external forces, check linear momentum
conservation under internal contact impulses. Check angular momentum with
consistent contact points and inertia. Use an appropriate numerical tolerance,
not bitwise equality.

Test expected restitution in controlled two-body collisions. Energy behavior
depends on restitution, friction, stabilization, and the integrator: do not
claim that all simulations conserve kinetic energy. Test frictionless elastic
cases separately from intentionally dissipative cases.

Add stationary-contact, off-center impulse, friction-ramp, and stack-stability
cases as those behaviors are introduced. There are no solver tests yet.
