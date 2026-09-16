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

The default broad phase uses single-threaded, single-axis sweep-and-prune
(SAP). AABBs and their original input indices are stored together in contiguous
scratch records, then sorted by minimum X each step. The sweep reads these
records directly rather than following sorted indices into an unsorted array.
The sweep checks later entries until their minimum X exceeds the current maximum X,
then stops; the remaining AABB axes reject false candidates. Touching bounds
are included. Dense overlap along X can still require quadratic scanning.
A candidate means only that a narrow-phase query may be necessary.

`PhysicsWorld::step()` calls `BroadPhase::findCandidatePairs()` and maps its
input indices back to live collider slots. Pairs are generated once and sorted
into the original collider-slot order to keep solver ordering deterministic;
colliders belonging to the same body are excluded. Bounds are refreshed from the
current body poses before pair generation and conservatively contain the
transformed shapes. Collision filtering and explicit static-static rejection
remain future policy work.

The `possiblePairs` statistic still counts all eligible collider pairs before
spatial rejection, using per-body counts rather than an all-pairs loop.
The brute-force implementation is retained as a test oracle, and the recorded
naive/Release reports remain performance baselines.

Later compare persistent SAP endpoints, a dynamic AABB tree, or another justified
spatial structure against the same workloads and baseline pair results.
Report false-positive counts alongside timings; faster execution is not useful
if valid pairs are lost.

### Experimental uniform grid

Set `PhysicsWorld::broadPhaseAlgorithm` to
`BroadPhaseAlgorithm::UniformGrid` to opt into the grid prototype. SAP remains
the default; both backends return the same canonical, sorted AABB-overlap pairs.
The grid uses the upper median of positive maximum AABB side lengths as its
cell width. An all-point/empty input uses a width of one.

Each AABB is inserted into every cell it overlaps, including cells touched at
its maximum boundary. Cell coordinates use `floor`, including for negative
positions. Contiguous cell/index records are sorted, pairs within each cell are
checked, and the output is sorted and deduplicated.

An AABB spanning more than 64 cells, or outside the signed 32-bit cell-coordinate
range, goes into an explicit overflow list instead of expanding into cells.
Overflow AABBs are checked against regular objects and each other without
dropping any pairs. This bounds cell storage, but many overflow objects can
make the comparison path expensive. Wide-size workloads demonstrate this
limitation; a large floor alone is much less costly.

### Broad-phase measurements

`StepStats` separates active-bound/index collection and final slot mapping/body
filtering from the `BroadPhaseStats` returned by the geometry query. The latter
times record construction, record sorting, the sweep, and output-pair sorting.
The sweep timer includes overlap checks, pair emission, and output-vector
growth. Timers are sampled at phase boundaries, never per pair. AABB
recomputation precedes the broadphase timer; minor call/destruction overhead is
included in the parent time but not assigned to an individual phase.

`xWindowComparisons` counts pairs whose X intervals overlap, including touching
intervals but excluding the probe that terminates each scan. It is accumulated
once per outer sweep iteration. `aabbPairs` counts full AABB overlaps before
same-body filtering; `candidatePairs` counts the pairs left after that filter.
The first two counts include same-body pairs, unlike `possiblePairs`.
For grid queries, the same four timing fields measure cell-width selection and
entry construction, cell-entry sorting, bucket/overflow comparison, and output
sorting/deduplication. `xWindowComparisons` is zero; `gridComparisons` counts
actual AABB tests, including repeat tests in shared cells and overflow checks.
`gridEntries`, `gridOverflowAabbs`, and `gridCellSize` expose storage amplification
and the overflow policy. `aabbPairs` counts unique output pairs after deduplication.

The optional query statistics are overwritten on every call, including empty
queries. Benchmarks print mean phase durations and explicitly label the work
counts as belonging to the last simulated step.

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
not yet used. Support-face vertices must lie inside the other box before
projection, so lifted corners do not become artificial support points. Each
accepted vertex gets its own penetration depth rather than sharing the deepest
overlap across the face. Pairs with no contained support-face vertices retain
the single-point overlap fallback.

If GJK and EPA are added later, GJK will answer convex
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
stability. Broad-phase coverage compares exact ordered pairs with an all-pairs
reference for empty, touching, degenerate, dense, random, and moving bounds.
World-level checks cover same-body filtering, collider-local transforms,
inactive slots, and body/collider slot reuse.
Both backends also match brute force for large floors, negative cell boundaries,
extreme coordinates, and degenerate bounds. Grid-specific checks cover the exact
64-cell threshold and duplicate suppression. A mixed sphere/rotating-box world
is stepped with each backend and compared for matching poses and velocities.
Additional coverage should include narrow-phase touching semantics, degenerate
inputs, pair symmetry, and multi-point box manifolds.

Broad-phase and collision benchmarks now exist, but their workload coverage
should be expanded. Broad-phase workloads should distinguish sparse, clustered,
and densely overlapping distributions. Collision workloads should isolate
shape-pair queries from world stepping, scene setup, and visualization, and
report false positives alongside timings. Benchmark runs should disclose
workload, build configuration, hardware, and algorithm settings.

`phys_broadphase_compare` isolates the two broadphase queries on uniform,
clustered, mixed-size-with-floor, and wide-size AABB layouts. Its inputs are
static between samples, and exact output equality is checked outside the timed
query. These query timings must not be confused with the full-world sphere
timings from `phys_collision_bench` and `phys_broadphase_bench`.
