# Collision pipeline

All algorithms and contracts below are planned; the collision module is empty.

```text
Body poses + collider local transforms
    -> world-space collider transforms
    -> conservative AABBs
    -> broad-phase candidate pairs
    -> narrow-phase shape queries
    -> contact manifolds
    -> solver input
```

## Broad phase

Start with all-pairs AABB overlap. It is an intentionally simple O(n^2)
baseline, not the final scalability story. A candidate means only that a
narrow-phase query may be necessary.

Define unique-pair ordering, touching-bound semantics, collision filtering,
same-body exclusion, and static-static handling before implementing pair
generation. Bounds must conservatively contain transformed shapes.

Later compare sweep and prune, a dynamic AABB tree, or another justified
spatial structure against the same workloads and baseline pair results.
Report false-positive counts alongside timings; faster execution is not useful
if valid pairs are lost.

## Narrow phase

Implement simple shape pairs before general convex algorithms:

1. Sphere-sphere.
2. Sphere-plane and axis-aligned box overlap.
3. Oriented box-box using SAT.
4. Convex-convex using GJK and EPA, if chosen as the differentiator.

The proposed tree has no plane shape file. Add one only when plane support is
implemented. An AABB bounding volume and a physical box shape are distinct
concepts even when an axis-aligned box query shares overlap logic.

SAT must progress beyond a boolean overlap result to contact generation.
GJK answers convex intersection queries; EPA extracts penetration information
from a suitable intersecting simplex. They are separate stages with explicit
degeneracy, iteration-limit, and failure contracts still to be designed.
GJK plus EPA alone does not provide a stable multi-point contact manifold.

## Contacts and manifolds

A manifold will identify the two participants, a consistently oriented normal,
and a small set of contact points with penetration information. The proposed
starting capacity is four contacts, but no representation is fixed yet.

Before writing response code, document the normal direction, penetration sign,
contact coordinate space, point meaning, and behavior for touching shapes.
Persistent contact identifiers and cached impulses are future solver needs,
not fields to add speculatively to this scaffold.

Collision detection produces geometry. It must not mutate body velocities or
perform ad hoc reflection.

## Future tests and benchmarks

Tests should cover separation, touching, penetration depth, coincident centers,
rotated shapes, degenerate inputs, pair symmetry, and the chosen contact
conventions. Compare broad-phase results against the all-pairs reference.

Broad-phase workloads should distinguish sparse, clustered, and densely
overlapping distributions. Collision workloads should isolate shape-pair
queries from world stepping, scene setup, and visualization. Neither benchmark
entry point currently contains workloads or timing code.
