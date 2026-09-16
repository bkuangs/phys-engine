# Collision Pipeline

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

The current broad phase uses all-pairs AABB overlap. It is an intentionally
simple O(n^2) baseline, not the final scalability story. A candidate means
only that a narrow-phase query may be necessary.

Pairs are generated once using stable collider-slot ordering, and colliders
belonging to the same body are excluded. Bounds are refreshed from the
current body poses before pair generation and conservatively contain the
transformed shapes. Collision filtering and explicit static-static rejection
remain future policy work.

Later compare sweep and prune, a dynamic AABB tree, or another justified
spatial structure against the same workloads and baseline pair results.
Report false-positive counts alongside timings; faster execution is not useful
if valid pairs are lost.

## Narrow phase

The current narrow phase implements these shape pairs:

1. Sphere-sphere.
2. Sphere-box, including axis-aligned boxes.
3. Oriented box-box contact generation using SAT-based overlap axes and
   support-face candidates.

Simple shape pairs were implemented before considering general convex
algorithms. GJK and EPA remain possible future extensions rather than required
parts of the current pipeline.

The box-box query progresses beyond a boolean overlap result and produces up
to four contact points. A full reference/incident-face clipping algorithm is
not yet used. If GJK and EPA are added later, GJK will answer convex
intersection queries and EPA will extract penetration information from a
suitable intersecting simplex; both need explicit degeneracy, iteration-limit,
and failure contracts. GJK plus EPA alone would not provide a stable
multi-point contact manifold.

An AABB bounding volume and a physical box shape are distinct concepts even
when an axis-aligned box query shares overlap logic.

## Contacts and manifolds

A manifold identifies the two participants, a consistently oriented normal,
material response values, and up to four contact points with penetration
information. Contact anchors are stored in each collider's local coordinate
space and converted to body-local anchors by the world before solving.

The normal points from participant A toward participant B. Penetration is a
nonnegative overlap depth, and touching shapes are generally not reported by
the current strict intersection tests. The solver uses the local anchors to
reconstruct contact offsets and maintains a heuristic persistent cache of
normal and tangent impulses for warm starting. Cache matching currently uses
body handles and local-anchor proximity rather than stable feature IDs.

Collision detection produces geometry. It must not mutate body velocities or
perform ad hoc reflection.

## Future tests and benchmarks

Tests cover separation, penetration depth, coincident centers, sphere and box
contact conventions, rotated shapes, friction response, and static-floor
stability. Additional coverage should include touching semantics, degenerate
inputs, pair symmetry, multi-point box manifolds, and explicit broad-phase
comparison against an all-pairs reference.

Broad-phase and collision benchmarks now exist, but their workload coverage
should be expanded. Broad-phase workloads should distinguish sparse, clustered,
and densely overlapping distributions. Collision workloads should isolate
shape-pair queries from world stepping, scene setup, and visualization, and
report false positives alongside timings. Benchmark runs should disclose
workload, build configuration, hardware, and algorithm settings.
