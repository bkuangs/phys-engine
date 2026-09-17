# Performance history

Measurements from September 16-17, 2026, on an Apple M2 running macOS. The main
algorithmic gain so far is replacing all-pairs broadphase with single-threaded,
single-axis sweep-and-prune (SAP). At 10,000 bodies, the recorded Release mean
step time fell from **197.83 ms to 13.40 ms**, approximately **14.76x faster**.
That is a substantial improvement, but still above the 8.33 ms budget for 120 Hz.
The subsequent [contiguous-record experiment](#contiguous-record-experiment)
improved SAP further using a fresh, repeated comparison.
Opt-in [uniform-grid](#uniform-grid-prototype) and
[dynamic-tree](#dynamic-aabb-tree-experiment) prototypes are evaluated below;
SAP remains the engine default.
The current default scaling benchmark is now the
[sustained mixed scene](#sustained-mixed-scene-scaling), not the short sphere-only
workload used by the early reports. Results from different workload protocols
must not be treated as before/after optimization comparisons.

## Comparable checkpoints

| Checkpoint | Source |
| --- | --- |
| Debug, all-pairs broadphase | `fd8f5a1` |
| Release, all-pairs broadphase | `fd8f5a1` |
| Release, X-axis SAP | `9b9bc30` |
| Release, contiguous-record SAP | `9b9bc30` + layout patch (uncommitted when measured) |

The baseline reports were committed in `0455cd0`. The SAP report was captured
before its implementation was committed in `9b9bc30`, so its original header
still describes the working tree as `0455cd0 + uncommitted single-axis SAP changes`.
The relevant measurements are summarized below; the superseded raw reports were
removed during benchmark cleanup.

An older naive run is historical context, not the denominator for these
comparisons: it did not record build type, hardware, or source revision, so its
raw output was removed.

## Workload and measurement

The initial three checkpoint runs use the existing headless sphere-field benchmark:

- Seed 42, sphere radius 0.5, with the spawn volume scaling with body count.
- Fixed simulation timestep of 1/120 second.
- 10 measured steps for 100, 500, 1,000, and 2,500 bodies; 100 steps for 5,000;
  25 steps for 10,000.
- A fresh world for each body count, with no warmup.
- Full `PhysicsWorld::step()` timing, including integration, broadphase,
  narrowphase, and solving, but excluding world creation and rendering.

Release uses Apple Clang 17.0.0.17000604 with
`-O3 -DNDEBUG -std=c++20 -arch arm64`. Both benchmark executables used
the same sphere workload for those captures; the broadphase executable is not an isolated SAP
microbenchmark.

These are single, short runs, not a controlled statistical study. System load,
cache state, and CPU scheduling can affect the results. In particular,
percentiles from 10 samples are coarse. Speedups below are ratios of the
reported, rounded means, not guaranteed improvements for every scene.

## Measured gains

Mean full-step time in milliseconds:

| Bodies | Debug all-pairs | Release all-pairs | Release SAP | SAP speedup over Release all-pairs |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0.28 | 0.04 | 0.06 | 0.67x |
| 500 | 2.21 | 0.61 | 0.34 | 1.79x |
| 1,000 | 7.58 | 2.17 | 0.83 | 2.61x |
| 2,500 | 47.70 | 13.31 | 2.37 | 5.62x |
| 5,000 | 137.53 | 52.77 | 3.42 | 15.43x |
| 10,000 | 547.02 | 197.83 | 13.40 | 14.76x |

At 100 bodies, the SAP run was slower, not faster. Setup/sorting overhead and
measurement noise matter at that scale; these runs do not isolate their shares.

The 120 Hz deadline results show where the improvement was practically useful:

| Bodies | Release all-pairs misses | Release SAP misses | SAP p95 step |
| ---: | ---: | ---: | ---: |
| 2,500 | 10 / 10 | 0 / 10 | 3.09 ms |
| 5,000 | 100 / 100 | 0 / 100 | 4.71 ms |
| 10,000 | 25 / 25 | 25 / 25 | 21.48 ms |

The 5,000-body SAP run stayed below budget even at its 5.65 ms maximum. The
10,000-body run still missed every deadline and had a 39.83 ms maximum.

## How we achieved the gains

### 1. Establish an optimized baseline

The Debug-to-Release change did not alter the algorithm or source revision.
Compiler optimization reduced the 10,000-body mean from 547.02 ms to 197.83 ms
(2.77x). This made Release the appropriate reference for subsequent engine
optimizations, rather than attributing compiler gains to SAP.

### 2. Stop examining every possible pair

The original world loop considered every pair of active colliders on different
bodies before checking AABB overlap. For 10,000 one-collider bodies, that means
49,995,000 possible pairs per step.

The initial SAP implementation in [broadphase.cpp](../src/collision/broadphase.cpp):

1. Sorts an index array by each AABB's minimum X, breaking ties by input index.
2. Scans later entries only until their minimum X exceeds the current maximum X.
3. Applies the full AABB overlap check to those remaining entries.
4. Canonicalizes each pair and sorts the output into the original index order.

This prunes spatially separated pairs before doing a full overlap check.
It remains single-threaded and sorts afresh each step; persistent endpoints
and incremental sorting are not implemented. The initial index array was
subsequently replaced by contiguous records, as described below.
Dense X overlap can still cause quadratic scanning, and output ordering adds
a sort over the emitted pairs.

### 3. Wire the optimization into the actual simulation

Previously, `PhysicsWorld::step()` contained its own all-pairs loop and did not
use `BroadPhase::findCandidatePairs()`. Updating the helper alone would not have
changed the sandbox or benchmark.

The world now collects active bounds, calls SAP, maps the returned indices back
to collider slots, and removes same-body pairs. The `possiblePairs` statistic
is computed using per-body collider counts in a linear pass, rather than
keeping a hidden all-pairs loop just to count pairs.

`possiblePairs` is still the theoretical eligible-pair count, not the number
of overlap checks SAP actually performs. At 10,000 bodies it remains 49,995,000;
the last-step candidate count remains 2,969. The last-step candidate counts
matched the Release baseline at all six body counts.

### 4. Preserve collision behavior

Touching AABBs remain candidates, same-body pairs remain excluded, and final
pair ordering matches the original collider-slot traversal. Stable ordering
avoids changing the sequential impulse solver's processing order simply because
objects sort differently along X.

[Broadphase regressions](../tests/collision/broadphase_tests.cpp) compare exact
ordered output against brute force for boundary cases, dense and random
layouts, and moving bounds. World-level cases cover local transforms,
same-body filtering, removed colliders, and reused body/collider slots.
Collision and solver regressions also passed in Debug and Release.

## Costs after the initial SAP implementation

At 10,000 bodies, the recorded Release stage means were:

| Stage | All-pairs | SAP |
| --- | ---: | ---: |
| Broadphase | 196.60 ms | 11.84 ms |
| Narrowphase | 0.25 ms | 0.44 ms |
| Solver | 0.87 ms | 0.89 ms |

Broadphase improved by approximately 16.60x, but still accounts for 88.4% of
the SAP step. Narrowphase and solver code were not optimized in this change;
their timing differences are not evidence of an intended algorithmic change.
Threading narrowphase is therefore not the first priority for this workload.

The simple SAP integration also adds four temporary allocations per step.
At 10,000 bodies, the reported average rose from 16 to 20. The layout experiment
below leaves allocation policy unchanged; buffer reuse remains a potential
follow-up. The later phase profile separates sorting, scanning, and filtering
to identify the remaining cost.

These measurements do not cover box-box contact costs, stacking, or renderer
performance. The earlier box-contact correctness fix should not be credited
with speedups in this sphere-only workload.

## Contiguous-record experiment

The sweep now sorts records containing both an AABB and its original input
index. Previously, every inner-loop iteration used a sorted index to fetch an
AABB from the original, unsorted array. Reading the sorted records directly
makes the scan sequential in memory and removes that extra lookup.

The X-axis choice, full AABB overlap checks, tie-breaking, final pair order,
world filtering, and allocation policy are unchanged. Records contain more
bytes than indices and cost more to move during sorting; the measurements
include that tradeoff, rather than timing only the scan.

For this experiment, the indexed executable was preserved before rebuilding.
Both variants used the same Release compiler flags and benchmark argument `10`.
Three runs per variant were executed sequentially in interleaved order:
indexed/contiguous, contiguous/indexed, indexed/contiguous. Each run creates the
same fresh seeded worlds. No warmup was added.

The table reports the **median of three per-run mean step times**, not a pooled
median or percentile of individual steps:

| Bodies | Fresh indexed SAP | Contiguous records | Speedup |
| ---: | ---: | ---: | ---: |
| 100 | 0.04 ms | 0.03 ms | 1.33x |
| 500 | 0.25 ms | 0.16 ms | 1.56x |
| 1,000 | 0.46 ms | 0.40 ms | 1.15x |
| 2,500 | 1.58 ms | 1.38 ms | 1.14x |
| 5,000 | 3.43 ms | 2.94 ms | 1.17x |
| 10,000 | 11.86 ms | 8.97 ms | 1.32x |

At 10,000 bodies, the median per-run broadphase mean fell from 9.64 ms to
7.89 ms. Across the three runs, deadline misses fell from 75/75 to 28/75.
The improvement does not establish reliable 120 Hz operation: the contiguous
runs still had p95 step times of 11.65-11.78 ms.

Sample counts, last-step candidate counts, possible-pair counts, and reported
allocation counts matched across every run. The broadphase brute-force
equivalence and world/solver regressions also passed in Debug and Release.
There are still 20 reported allocations per step at 10,000 bodies; this is a
layout optimization, not buffer reuse.

The comparison used fresh indexed controls rather than treating the earlier
single-run 13.40 ms result as a controlled before measurement. These are still
short runs on a shared machine, so the observed ratios should not be treated as
universal speedups.

## Broadphase phase profile

The next measurement adds phase-boundary timers and work counters without
changing the collision algorithm. A control executable from `a6b5dd6` was
preserved, then three control runs and three instrumented runs were interleaved
using the same Release configuration, seed, and sample argument `10`.
The profile captured per-run timings and counts summarized below.

At 10,000 bodies, the instrumented full-step median run mean was 9.04 ms and
the broadphase median run mean was 7.97 ms. The detailed timings were:

| Broadphase component | Median of three per-run means |
| --- | ---: |
| Collect active bounds/indices and count eligible pairs | 0.0300 ms |
| Build contiguous sweep records | 0.0228 ms |
| Sort records by X | 0.5038 ms |
| Sweep, check overlaps, and emit pairs | 7.2686 ms |
| Sort output pairs into deterministic order | 0.1375 ms |
| Map slots and filter same-body pairs | 0.0124 ms |

These independently computed medians and rounded totals need not sum exactly.
The sweep accounts for approximately **91% of broadphase time** and **80% of
the whole step**. Record sorting is about 6% of broadphase time; setup and
filtering are much smaller.

The final step examined **2,284,736 X-overlapping pairs**, of which only
**2,969** passed the full AABB test. Since this benchmark gives each sphere its
own body, all 2,969 also survived body filtering. That is about **770 X-window
comparisons per emitted pair**: **99.87%** of those pairs were rejected on the
remaining axes. These are last-step counts, whereas the durations above
summarize all measured steps.

This identifies the sweep's candidate volume as the primary target, rather
than scratch-buffer setup or sorting alone. Persistent sorting cannot remove
the millions of remaining overlap checks. The sweep timing also includes
output-vector growth, so it should not be interpreted as a hardware-level
breakdown of comparisons versus allocation.

The control's median run mean was 9.23 ms, versus 9.04 ms with instrumentation.
Individual control means ranged from 8.91 to 9.62 ms; instrumented means ranged
from 8.98 to 9.33 ms. Their overlap does not establish a speedup or precisely
measure instrumentation overhead, but the repeated phase results consistently
identify the sweep as dominant.

Phase counters are checked against brute force, including touching intervals,
empty/reset cases, and same-body filtering. Existing pair, contact, and
allocation counts matched the control throughout the benchmark. No per-pair
clock calls or additional allocation-tracking buffers were introduced.

## Uniform-grid prototype

The grid is an explicit alternative to SAP, not a replacement default.
It chooses its cell width from typical AABB size, inserts each AABB into all
overlapped cells, sorts contiguous cell/index records, and deduplicates the
resulting pairs. A 64-cell-per-AABB limit and an overflow comparison path keep
large floors or extreme coordinates from causing unbounded cell expansion.
See the [collision pipeline](collision_pipeline.md#experimental-uniform-grid)
for the precise policy.

The full-world comparison uses three interleaved runs per backend with the same
seed, Release build, and argument `10`. These are medians of per-run mean step
times:

| Bodies | Fresh SAP | Uniform grid | SAP time / grid time |
| ---: | ---: | ---: | ---: |
| 100 | 0.03 ms | 0.06 ms | 0.50x |
| 500 | 0.16 ms | 0.29 ms | 0.55x |
| 1,000 | 0.43 ms | 0.63 ms | 0.68x |
| 2,500 | 1.65 ms | 1.73 ms | 0.95x |
| 5,000 | 3.11 ms | 3.00 ms | 1.04x |
| 10,000 | 9.44 ms | 7.10 ms | 1.33x |

At 10,000 bodies, last-step AABB tests fell from 2,284,736 to 30,485, roughly
a 75-fold reduction, with the same 2,969 unique candidate pairs. However, the
grid creates 80,000 cell records, and sorting them took a median run mean of
4.7663 ms out of 6.04 ms of broadphase time. The bottleneck shifted toward
building/sorting the spatial index, rather than disappearing.

The grid missed 20/75 deadlines versus SAP's 72/75 in this fresh comparison.
Its per-run p95 step times were 9.96-10.08 ms: an average below 8.33 ms does not
establish consistent 120 Hz performance. At 10,000 bodies, reported allocations
also increased from 20 to 24 per step. Small scenes were slower with the grid.

### Other layouts

The separate layout comparison times standalone broadphase queries on static
AABBs, not full simulation steps. Ten queries per backend are interleaved for
each layout, with exact ordered pair equality checked outside the timed region.
At 10,000 AABBs:

| Layout | SAP query mean | Grid query mean |
| --- | ---: | ---: |
| Uniform, unit-sized AABBs | 7.99 ms | 6.31 ms |
| Eight clusters | 27.79 ms | 18.94 ms |
| Rotated mixed-size boxes plus a large floor | 12.42 ms | 6.85 ms |
| Rotated cubes with a wide size distribution | 21.51 ms | 115.67 ms |

The mixed-floor layout had only one overflow object. The wide-size layout
had 1,990 overflow AABBs, causing 17,967,931 grid-path checks versus SAP's
4,159,465. The overflow path preserves correctness and bounded cell storage,
but costs roughly the number of overflow objects times the total object count.

These results support keeping the grid available for experiments while
retaining SAP as the default. It helps large, similarly sized populations and
some clustered/mixed layouts, but is not an engine-wide win. Cheaper grid
indexing and better handling of heterogeneous sizes are possible future work;
neither is claimed as an improvement already achieved.

Both backends match brute force for boundary, random, degenerate, oversized,
and extreme-coordinate inputs. A mixed sphere/rotating-box world on a large
floor also evolves with matching poses and velocities under both backends.
Default SAP behavior, pair ordering, and same-body filtering are retained.

## Dynamic AABB tree experiment

The profiling/grid work was committed as `80d60c2`, then the persistent-tree
experiment was developed on `experiment/dynamic-aabb-tree`. The tree remains
opt-in through the `tree` backend; the default was not changed.

The implementation retains an index-based node pool between steps, tracks
collider-slot generations, and expands leaf bounds by 10% of each axis extent
(minimum padding 0.01). Proxies are reinserted when tight bounds escape that
padding or shrink substantially. Height-balancing rotations also consider
spatial area. Tight leaf bounds are checked before emitting canonical pairs,
so fat bounds do not change the candidate set.

### Refining the first version

The first correct version was not a performance win. It restarted a root
query for every leaf and used height-only rotations. At 10,000 bodies its
median run mean was 17.14 ms. Spatially aware rotations reduced that to
13.13 ms, but traversal remained dominant.

The final version traverses overlapping subtree pairs once, starting at
root/root. Within-subtree and cross-subtree work partition the unordered
leaf pairs; overlapping distinct subtrees are split by area. This avoids
repeated ancestor traversal without duplicating emitted pairs.

These were separate experimental batches, not an interleaved comparison of all
three tree implementations. Their useful results are summarized here; the
superseded raw reports were removed. The final batch below contains fresh SAP
and grid controls.

### Full-world results, including maintenance

The [final full-world report](../benchmarks/results/tree-release.txt) uses
three interleaved runs per backend, seed 42, argument `10`, and the same Release
configuration. Each run starts with a fresh world and empty tree. Initial
construction is included in the first step and in the reported means.

Step columns are medians of per-run means; the last column is the median
first-step time. All values are milliseconds:

| Bodies | SAP | Grid | Tree | Tree first step |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0.03 | 0.06 | 0.03 | 0.07 |
| 500 | 0.17 | 0.29 | 0.16 | 0.31 |
| 1,000 | 0.42 | 0.63 | 0.37 | 0.67 |
| 2,500 | 1.51 | 1.76 | 1.13 | 1.93 |
| 5,000 | 2.93 | 3.04 | 2.28 | 4.37 |
| 10,000 | 9.33 | 7.10 | 5.11 | 9.61 |

At 10,000 bodies, the tree was approximately 1.83x faster than SAP and 1.39x
faster than the grid in this batch. Tree maintenance averaged 0.8602 ms and
paired traversal 3.0115 ms, using medians of the per-run means. Broadphase
overall was 4.05 ms.

The tree missed 4/75 deadlines, compared with SAP's 30/75 and the grid's
20/75. Tree per-run p95 step times were 7.43-7.56 ms, but the median first
step was 9.61 ms. Cold construction and occasional spikes still prevent a
claim of guaranteed 120 Hz operation.

The final step visited 245,464 node pairs, checked 6,785 tight leaf pairs,
and emitted the same 2,969 AABB-overlap pairs as SAP/grid. Tree height was 16
for 10,000 proxies. Node-pair visits are a different unit from the single-node
visits in the earlier per-leaf-query reports, so their counts are not directly
comparable. Each run performed 10,000 initial insertions and 32,580 subsequent
reinsertions; maintenance was not omitted from the measurements.

### Moving and mixed-size layouts

The [final layout report](../benchmarks/results/tree-layouts-release.txt)
compares static and moving/resizing AABBs. It times initial construction,
incremental maintenance, and querying together. Motion and reference-pair
generation are outside timing for all backends. Each tree persists across
the ten samples rather than being rebuilt each time.

At 10,000 AABBs, the moving-case mean maintenance-plus-query times were:

| Layout | SAP | Grid | Tree |
| --- | ---: | ---: | ---: |
| Uniform | 8.32 ms | 6.33 ms | 4.41 ms |
| Eight clusters | 27.26 ms | 19.30 ms | 11.74 ms |
| Mixed rotated boxes and a large floor | 13.00 ms | 6.91 ms | 6.67 ms |
| Wide-size rotated cubes | 22.29 ms | 118.83 ms | 16.48 ms |

The tree avoids the grid's expensive oversized-object comparison list, but
is not uniformly faster: at 1,000 AABBs, the moving wide-size query was
1.05 ms versus SAP's 0.71 ms. The mixed-floor advantage over the grid at
10,000 AABBs is small enough to warrant more samples. These query benchmarks
are not complete mixed-shape simulation steps.

Exact pair order and evolving world states match the existing backends.
Coverage includes fat-bound false positives, movement inside padding,
resizing, deletion/reuse, backend switching, tree copy/move behavior, and
invalid proxy operations. Address/undefined-behavior sanitizer runs cover
the tree and world integration stress cases. These results support further
evaluation on this branch, not an automatic default switch.

## Compact-node trials (not adopted)

The completed tree experiment was committed as `ddfc6a5` before these trials.
Two smaller layouts were then compared with preserved Release executables from
that commit. Each version ran three times in interleaved order, with the same
seed, sample argument `10`, fat bounds, topology decisions, and paired traversal.

Compiler record-layout output confirmed the original node is 88 bytes on this
arm64 build. The two trial layouts were:

| Trial | Storage design | Original 10,000-body step | Trial step |
| --- | --- | ---: | ---: |
| Hot/cold split | 32-byte traversal node, 8-byte per-node metadata, 40-byte payload per live leaf | 4.97 ms | 5.02 ms |
| Compact inline | 72-byte node, with bounds and 32-bit child links in its first 32 bytes | 4.91 ms | 5.27 ms |

Step times are medians of three per-run means. Each row has its own fresh
control batch; the trial timings should not be ranked as if they came from
one shared three-way run.

At 10,000 leaves, the hot/cold split reduced logical node/leaf storage by roughly
32%, excluding vector capacity slack and container overhead. The inline version
reduced per-node size by about 18%. Both retained
the public `size_t` proxy/user-index types and checked narrowing to internal
32-bit indices.

The memory reductions did not translate into a convincing overall throughput
gain. The hot/cold split improved the 5,000-body mean from 2.25 to 2.10 ms and
some standalone query layouts, but its 10,000-body result was effectively flat.
It also introduced additional pools and payload-relocation bookkeeping.
The simpler inline layout regressed the 10,000-body mean in its batch and
was generally flat or slightly slower across the moving query layouts.

Pair counts, node-pair visits, leaf checks, heights, and reinsertion counts
matched exactly across the versions, so these differences were not produced
by pruning different work. The trials passed the existing geometry/lifecycle
and sanitizer cases. Additional coverage for copy assignment, proxy reuse,
full-width user indices, and invalid high-bit proxy IDs is retained.

**Neither layout was adopted.** The production tree was restored exactly to
`ddfc6a5`; it still uses the original 88-byte node on this platform. The
experiments do not establish node footprint as the main limiting factor, and
short-run differences on a shared machine should not be overstated.

The rejected trial outputs were removed after recording the methodology and
results above.

## Surface-area caching trials (not adopted)

The next experiment cached each node's fat-bound surface area in a `double`,
reusing it during paired traversal and insertion-cost calculations. The cache
was refreshed on creation, reinsertion, and internal refit. Existing member
ordering was otherwise retained, with the new value next to its fat bounds.
Compiler layout output showed node size increasing from 88 to 96 bytes.

Both trials used preserved uncached executables from `ddfc6a5`, three
interleaved runs per version, seed 42, and argument `10`. Total step timings
include maintenance, initial construction, and cache initialization.

The initial eager-refit version reduced the 10,000-body query mean from
2.9331 to 2.6438 ms, but maintenance rose from 0.8434 to 1.0490 ms. The
full-step result was nearly flat: 4.98 to 4.92 ms. At 5,000 bodies, the
step mean increased from 2.19 to 2.31 ms.

A second version skipped cache recomputation when a refit's fat bounds were
unchanged, while still updating height. Its fresh comparison was:

| Bodies | Uncached step | Guarded-cache step | Uncached maintenance | Cached maintenance | Uncached query | Cached query |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 5,000 | 2.23 ms | 2.31 ms | 0.6822 ms | 0.8684 ms | 1.2810 ms | 1.1673 ms |
| 10,000 | 5.05 ms | 4.91 ms | 0.8693 ms | 1.0323 ms | 2.9698 ms | 2.6492 ms |

These are medians of per-run means, not pooled step percentiles. The two body
counts also use different run lengths: 100 steps at 5,000 bodies and 25 at
10,000. Their maintenance/query ratios differ; the comparisons within each
row use identical workloads.

Static standalone queries generally showed roughly 1.06-1.11x speedups at 10,000
AABBs in the guarded trial. Moving queries had smaller, mixed changes;
uniform moving queries were slightly slower. Both versions preserved exact
pair counts, traversal counts, heights, reinsertion counts, and allocation
counts. The geometry/lifecycle and sanitizer cases passed.

**The cache was not adopted.** It improved queries but increased maintenance
and node storage, without a convincing overall moving-world win. The results
do not isolate extra arithmetic, writes, and larger-record cache effects.
The original uncached implementation was restored exactly to `ddfc6a5`.
A CPU sampling profile is a better next investigation than another speculative
layout or cache change.

The rejected trial outputs were removed after recording the methodology and
results above.

## CPU sampling

The rejected-experiment findings were committed as `118e337` before sampling.
At this capture, the engine used the uncached tree implementation from `ddfc6a5`.
Sampling was captured on September 17, 2026, using macOS `/usr/bin/sample`;
Instruments' `xctrace` CLI was unavailable.

The dedicated `phys_cpu_profile` runner uses `-O3 -DNDEBUG -g
-fno-omit-frame-pointer` and a dSYM bundle. It does not link the benchmark
allocation-counting override or record per-step timing vectors. The original
sphere factory is shared with the existing benchmarks rather than duplicated.
No engine algorithms were changed for profiling.

### Workloads and capture protocol

The launcher samples only the PIDs it creates, sequentially. Each capture
requests 15 seconds at a 1 ms interval while the driver runs for 20 seconds.
The raw call graphs contain 11,494 sphere-workload and 11,554 box-workload
main-thread samples. The driver's scalar timing summaries cover its entire
20-second run, not exactly the sampling interval.
Simulation advances at a fixed 1/120-second timestep in an unthrottled loop;
the driver does not sleep to enforce real-time pacing.

| Workload | Setup | Behavior during capture |
| --- | --- | --- |
| Moving spheres | 10,000 spheres, seed 42, 120 Hz | Repeat fresh 25-step batches so long profiling runs do not turn into a different long-fall workload. Tree startup remains in each batch. |
| Settled box stacks | 512 dynamic unit boxes in 256 two-high stacks, one static floor, 120 Hz | Warm up before sampling, then continuously step the same world. |

The box scene settled after 240 warmup steps. Its initial maximum linear and
angular speeds were about 1.3e-8 and 1.4e-8; it had 512 manifolds and 2,048
contact points. The measured run retained exactly 512 manifolds throughout,
performed zero tree reinsertions, and remained below the velocity thresholds.
An initial eight-high fixture did not settle under the existing solver settings
and was excluded; no solver tuning was used to force a settled profile.

Driver logs are retained for the
[sphere](../benchmarks/results/profiling/cpu-tree-spheres.run.txt) and
[box](../benchmarks/results/profiling/cpu-tree-boxes.run.txt) workloads. The
large machine-specific call-graph dumps were removed after summarizing them
below.

### What the samples show

Percentages below use all main-thread samples as their denominator.
Inclusive function shares count a stack only once, even when optimized
symbolication repeats the same function name. Self samples exclude children.
The warm-start row is a subset of the solver, not an additional stage.

| Sample attribution | Moving spheres | Settled boxes |
| --- | ---: | ---: |
| Tree query and output ordering, inclusive | 61.7% | 0.9% |
| Tree maintenance functions, inclusive | 16.0% | 0.2% |
| Solver, inclusive | 15.9% | 92.5% |
| Narrowphase, inclusive | 1.1% | 5.5% |
| Warm-start cache scan, self samples at solver lines 230-232 | 6.9% | 38.9% |
| Identified allocation/reclaim/memmove helper self samples | 0.3% | 0.1% |

The timing summaries corroborate the different bottlenecks. Sphere steps
averaged 5.3577 ms, with 3.1611 ms of tree traversal and 0.8553 ms of solving.
Box steps averaged 2.9137 ms, with 2.6992 ms of solving and only 0.0237 ms of
tree traversal. These are instrumented diagnostic runs, not a new A/B speedup
claim or a comparison of equivalent workloads.

These samples identified **indexing the warm-start cache by
ordered body pair**, retaining the existing local-anchor matching and solver
processing order, as the clearest next small optimization. The implementation
then scanned the entire previous cache for each contact point. That loop was
substantial work, especially for the 2,048-point resting-contact workload.
The subsequent indexing change and its measurements are described below.

**Sleeping/awake islands** are a separate, larger opportunity: the box scene
keeps spending almost all its CPU time solving contacts despite negligible
motion. Sleeping colliders must remain available to active-body collision
queries and wake correctly. The subsequent opt-in implementation is measured
under [sleeping islands](#sleeping-islands).

For the sphere workload, samples concentrate in the node-pair traversal loop
and its child-stack paths. Optimized source-line attribution does not establish
cache misses, branch mispredictions, or a specific SIMD opportunity; instruction
and hardware-counter analysis would be needed to distinguish those causes.
Named inverse-inertia samples also do not include all inlined matrix work, so
their small standalone symbol count should not be treated as a complete cost.

Allocation/copy helpers were not dominant in these captures. Buffer reuse can
still improve predictability, but allocation counts alone do not justify making
it the first throughput optimization. Some optimized helpers remain labeled
`<deduplicated_symbol>`; all sample shares are statistical estimates.

### Repeating the sampling run

On macOS, with Python 3 and the command-line developer tools:

```sh
cmake -S . -B build/cpu-profile \
  -DCMAKE_BUILD_TYPE=Release \
  '-DCMAKE_CXX_FLAGS_RELEASE=-O3 -DNDEBUG -g -fno-omit-frame-pointer' \
  -DPHYS_BUILD_SANDBOX=OFF \
  -DPHYS_BUILD_BENCHMARKS=ON \
  -DBUILD_TESTING=OFF
cmake --build build/cpu-profile --target phys_cpu_profile -j 4
dsymutil build/cpu-profile/phys_cpu_profile
python3 benchmarks/sample_cpu.py \
  build/cpu-profile/phys_cpu_profile \
  profiling-results/next-cpu-run
```

Choose a fresh output directory; existing reports are not overwritten. The
runner's `--wait` handshake lets setup/warmup finish before sampling starts,
and the launcher waits for both child processes to exit or terminates its own
children on failure. It does not sample unrelated processes.
The driver can also run directly as `phys_cpu_profile spheres 20` or
`phys_cpu_profile boxes 20`, without the sampler.

## Warm-start body-pair indexing

CPU sampling motivated a targeted change to the solver, rather than a new solver
algorithm. Previous cache entries are kept in ordered-body-pair groups. A
manifold uses `std::equal_range` to locate its group once, then its points scan
only that group's cached anchors.

The key includes both body indices and generations, and preserves A/B direction.
Only the private cache is reordered; live manifolds and the eight solver
iterations retain their original order. Stable sorting preserves the first
matching anchor within a pair, not just the nearest or first exact match.
An `is_sorted` fast path avoids stable-sort work and temporary allocation when
the newly generated cache is already ordered.

For M cached points and P current manifolds, lookup no longer scans all M
entries for every current point. It does roughly P binary range lookups plus
the anchor checks within each matching body pair. Cache ordering costs a
linear sortedness check in the common case, or a stable sort otherwise.
Groups can still be large when many colliders share the same body pair.

### Full-step comparison

The [comparison report](../benchmarks/results/warmstart-index.txt) records three
interleaved runs per version against executables preserved from `db9ff26`.
No CPU sampler ran during these timing comparisons.

| Workload | Before: full step | Indexed: full step | Before: solver | Indexed: solver |
| --- | ---: | ---: | ---: | ---: |
| 10,000 moving spheres | 4.99 ms | 4.62 ms | 0.81 ms | 0.49 ms |
| 512 settled boxes | 2.7770 ms | 1.7216 ms | 2.5736 ms | 1.5180 ms |

These are medians of per-run means. The sphere runs use the existing Release
benchmark, seed 42, argument `10`, and include cold starts. The box runs use
three five-second intervals after warmup, with the same optimized/symbolized
driver on both sides. The two workloads use different build flags, so compare
versions within each row rather than comparing rows.

The settled-box full step improved by about **38%**, with solver time down
about **41%**. The 10,000-sphere step improved by about **7.4%**, with solver
time down about **39.5%**. Smaller sphere cases were mostly flat or noisy.

Sphere pair counts, tree traversal counts, heights, and allocation counts
matched the baseline. Both box versions remained settled with exactly 512
manifolds and no tree reinsertions. These common workloads needed no cache
reordering allocation; less orderly body/collider mappings can require
`stable_sort` scratch space.

### Sampling after the change

New 15-second captures use the same profiling protocol. The retained
[sphere](../benchmarks/results/profiling/cpu-warmstart-index-spheres.run.txt)
and [box](../benchmarks/results/profiling/cpu-warmstart-index-boxes.run.txt)
driver logs record the workloads; the machine-specific call graphs were removed
after analysis.

The old linear cache-scan hotspot disappeared. In the box capture, the new
binary-search callsite accounts for roughly 0.9% of self samples, with additional
small costs in matching and optimized helpers. Most remaining solver samples
are in impulse/friction work. Source-line attribution is approximate and is
not a complete accounting of every inlined cache operation.

The solver still occupies about 88% of box CPU samples, but that fraction is
of a much faster step: the sampled driver mean fell from 2.9137 to 1.7951 ms.
The unprofiled paired measurements above are the primary speed comparison.
Raw sample line numbers refer to their captured revisions.

Regression coverage explicitly exercises interleaved and reversed body pairs,
near-anchor first-match priority ahead of a later exact match, and generation
changes when either body slot is reused. The previous restitution, friction,
angular-response, floor, and world/backend checks continue to pass.

**This optimization is retained.** It changes neither the iteration budget
nor the physical matching rules, and directly removes the sampled search work.

## Sleeping islands

The `experiment/sleeping-islands` branch adds opt-in, contact-connected sleeping
on top of the indexed solver. All members of a dynamic island must remain at or
below 0.05 units/s linear speed and 0.05 rad/s angular speed for 0.5 simulation
seconds without wake requests. Static floors do not connect independent islands.
Sleeping bodies skip integration and constraint solving, but remain in
broadphase/narrowphase and keep their contact geometry and warm-start cache.
See [architecture](architecture.md#opt-in-sleeping-islands) for wake ordering
and mutation handling.

The [comparison report](../benchmarks/results/sleeping-release.txt) records three
interleaved runs for a preserved pre-change executable, the new executable with
sleeping disabled, and that same executable with sleeping enabled. All use the
Release flags `-O3 -DNDEBUG`, Apple M2, and the dynamic-tree backend. No CPU
sampler or allocation-counting hook ran during timing.

Step times below are medians of three per-run means, in milliseconds:

| Workload | Before change | Sleeping disabled | Sleeping enabled |
| --- | ---: | ---: | ---: |
| 512 settled boxes | 1.7553 | 1.8131 | 0.2372 |
| 10,000 moving spheres | 4.8844 | 4.8794 | 5.1632 |

The primary same-executable box comparison improves full-step time by about
**7.64x (86.9% less time)**. Solver time falls from 1.5947 to 0.0153 ms.
All three sleeping runs retain 512 manifolds / 2,048 contact points, keep all
512 dynamic bodies asleep, solve zero manifolds, and perform no tree
reinsertions. Remaining cost is primarily narrowphase (0.1650 ms); retaining
full collision detection intentionally leaves this work in place.

Each box run warms for 240 steps before its five-second timed interval. The
sleeping driver's warmup and end-of-run checks require every box to be asleep,
not just near-zero mean velocity. The non-sleeping runs show some shared-machine
load variation; the before/disabled difference does not establish a regression
or speedup from the disabled feature.

The moving-sphere control repeats fresh 25-step worlds for three seconds per
run. No sphere sleeps in these short batches. Enabling island construction and
mutation snapshots costs about **5.8%** in this workload, which is why sleeping
remains opt-in rather than an unconditional throughput improvement.

These are unpaced, fixed-1/120-second simulations and report mean step costs,
not a paced deadline or worst-case-latency guarantee. Reproduce the comparison
using the same executable:

```sh
cmake --build build/release-bench --target phys_cpu_profile -j 4
./build/release-bench/phys_cpu_profile boxes 5
./build/release-bench/phys_cpu_profile boxes 5 --sleep
./build/release-bench/phys_cpu_profile spheres 3
./build/release-bench/phys_cpu_profile spheres 3 --sleep
```

`--sleep` can be combined with the existing `--wait` sampling handshake.
Without it, the profiling workloads remain always active.

## Sustained mixed-scene scaling

The default scaling benchmark now uses a seeded, balanced mixture of radius-0.5
spheres and unit boxes above a static floor. Objects start in two shallow,
jittered layers without overlap. The mix is shuffled with seed 42 and contains
equal numbers of spheres and boxes at every default scale. Body-count labels
refer to dynamic bodies; the floor is one additional static body.

Every scale runs **240 warmup steps followed by 1,200 measured steps at 120 Hz**:
two simulated seconds plus ten measured seconds. Explicit step-count arguments
apply exactly to every body count. The former 5,000/10,000-body caps are removed.
Warmup is excluded from percentiles, deadline counts, stage means, and allocation
counts. Cold first-step cost and total warmup step time are reported separately.
The `first` value under measured step timing is the first step after warmup.
Step timings use the engine's existing `StepStats.totalMs`; scene creation,
health checks, sample recording, and reporting are outside those timings.

The floor has a 32-unit margin around the spawn footprint. Its thickness is
`min(1, 0.5 * BodyLimits::maxSize / floorArea)`, keeping its top at y=0 while
respecting the engine's volume limit. An initial 16-unit margin was rejected
when a rolling body left the floor in the 500-body case; the incomplete run was
removed because it was not a valid performance baseline.

The benchmark checks tracked bodies after warmup and after measurement. Non-finite
state, invalid bounds, or objects completely outside/below the floor fail the run
instead of yielding a misleading success. It also reports final floor penetration,
linear/angular speeds, measured contact load and maximum contact penetration.
These endpoint checks are not a proof of perfect constraint stability.

The allocator scope now surrounds only `PhysicsWorld::step()`, excluding the
benchmark's sample-vector growth and reporting. These remain C++ allocation-hook
counts, not a count of every possible allocator API or worker-thread allocation.
Sleeping mode, mean awake/sleeping bodies, and mean solved manifolds are explicit.

### First long-run results

These runs were recorded on `experiment/sleeping-islands` after coordinating
with the concurrent sleeping work. Sleeping was **disabled** for both runs.
The same source SHA-256 was recorded and verified unchanged across each build
and measurement, so the results do not mix source revisions during a run.

| Dynamic bodies | SAP mean | SAP p95 | Tree mean | Tree p95 |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0.19 ms | 0.22 ms | 0.21 ms | 0.23 ms |
| 500 | 1.19 ms | 1.32 ms | 1.20 ms | 1.28 ms |
| 1,000 | 2.33 ms | 2.46 ms | 2.60 ms | 2.73 ms |
| 2,500 | 6.25 ms | 6.59 ms | 6.93 ms | 7.41 ms |
| 5,000 | 13.10 ms | 13.89 ms | 13.75 ms | 14.50 ms |
| 10,000 | 27.32 ms | 29.90 ms | 30.94 ms | 34.97 ms |

All cases measured 1,200 steps and passed the population/floor checks. SAP and
tree produced identical contact-load and scene-health results. At 10,000
dynamic bodies, the run averaged 16,137.58 manifolds and 23,740 contact points.
SAP spent 17.68 ms in the solver, 3.90 ms in narrowphase, and 5.56 ms in broadphase;
the tree spent 17.64, 3.91, and 9.22 ms respectively.

This is far more contact-heavy than the earlier short free-falling-sphere
workload. It also shows that the tree is not universally fastest: SAP was faster
for this broad, shallow mixed scene. Neither backend met the 8.33 ms deadline
at 5,000 or 10,000 bodies; both missed all 1,200 measured deadlines there.
The 2,500-body runs missed 15 deadlines with SAP and 32 with the tree.

These are single long sweeps per backend, not multi-seed confidence intervals
or real-time guarantees. Occasional shared-machine spikes remain visible in
the raw reports. The longer sample count improves percentile usefulness but
does not make this one scene representative of every simulation.

Raw reports:
[default SAP](../benchmarks/results/mixed-sustained-sap-release.txt) and
[explicit tree](../benchmarks/results/mixed-sustained-tree-release.txt).
Historical short sphere reports are retained and are not directly comparable
to this new baseline.

### Sustained mixed scene with sleeping enabled

The sleeping feature and mixed-scene benchmark were committed as `b93adca`.
A trailing `--sleep` selector now enables sleeping before warmup; without it,
the scaling runner retains its always-active default.

The [sleeping comparison](../benchmarks/results/mixed-sleeping-comparison.txt)
uses the same executable for fresh off/on runs with SAP and the tree. Every
size runs 240 warmup and 1,200 measured steps on the same seeded mixed scene.
Execution order was SAP off, SAP on, tree on, tree off. The source fingerprint
was unchanged throughout all four sweeps.

| Dynamic bodies | SAP off | SAP on | Tree off | Tree on | Mean bodies asleep |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 100 | 0.19 ms | 0.21 ms | 0.21 ms | 0.19 ms | 12.4% |
| 500 | 1.15 ms | 1.05 ms | 1.20 ms | 1.18 ms | 12.2% |
| 1,000 | 2.30 ms | 2.30 ms | 2.58 ms | 2.68 ms | 7.1% |
| 2,500 | 6.25 ms | 6.18 ms | 6.83 ms | 6.80 ms | 6.4% |
| 5,000 | 13.00 ms | 13.01 ms | 13.79 ms | 13.61 ms | 6.0% |
| 10,000 | 27.06 ms | 27.43 ms | 30.83 ms | 34.32 ms | 5.3% |

These are measured step means from one sustained sweep per mode/backend, not
repeated-run medians. Both enabled backends had the same mean sleeping counts.
All runs passed scene-health checks and accounted for every dynamic body as
awake or sleeping.

This does not reproduce the large gain from the fully settled box-only scene.
At 10,000 bodies, an average of 9,474.30 bodies remained awake. SAP's solved
manifolds fell only from 16,137.58 to 15,637.40, and solver time from 17.56 to
17.21 ms. Most solving work remained, while sleeping added island bookkeeping.
Collision geometry was deliberately retained, so narrowphase work did not vanish.

The tree also did more traversal work with sleeping: its last-step node-pair
visits increased from 689,361 to 1,130,707, with similar mean candidate counts.
Query time rose from 8.1572 to 11.2061 ms. Sleeping changes trajectories and
proxy-update history, so an unchanged body count does not imply an unchanged
tree shape or traversal cost.

Both backends still missed all 1,200 deadlines at 10,000 bodies. Sleeping is
therefore workload-dependent: highly effective for settled independent islands,
but not a general speedup when most of this mixed scene remains awake. No
thresholds or iteration counts were tuned to manufacture a gain, and the default
remains sleeping disabled. Small timing differences remain load-sensitive.

## Caching solver invariants

The next optimization caches inverse mass and world-space inverse inertia once
per participating body per `solve()` call. A second layer caches each contact's
angular response to a unit normal or friction-axis impulse. Iterations scale
these responses instead of repeating cross products and matrix-vector products.
This is automatic, single-threaded preparation, not a persistent cache or SIMD
rewrite. The eight iterations, contact/axis order, clamping, restitution,
warm-start matching, and sleeping thresholds are unchanged.

The [comparison report](../benchmarks/results/solver-cache-release.txt) separates
the original solver at `0e0da25`, body-only caching, and the retained combined
body/contact-response cache. All use the same Release flags and workload code.

### Full-step results

The mixed-scene rows are individual matched runs: 240 warmup plus 1,200 measured
steps, seed 42, sleeping disabled. A focused driver calls the existing
`runBenchmark()` for the selected sizes/backends. The box row is the median of
three interleaved three-second runs after 240 warmup steps, also with sleeping
disabled. All values are milliseconds:

| Workload | Original step | Body-only cache | Body + contact cache | Step-time reduction |
| --- | ---: | ---: | ---: | ---: |
| 2,500 mixed, tree | 6.81 | 6.30 | 5.31 | 22.0% |
| 10,000 mixed, tree | 31.22 | 28.73 | 25.80 | 17.4% |
| 10,000 mixed, SAP | 27.27 | 24.93 | 21.73 | 20.3% |
| 512 settled boxes, tree | 1.7172 | 1.5636 | 1.1924 | 30.6% |

At 2,500 mixed bodies, solver time falls from 4.33 to 2.86 ms. At 10,000 it falls
from 17.86 to 12.12 ms with the tree, and from 17.63 to 12.09 ms with SAP.
The settled-box solver falls from 1.5139 to 0.9883 ms (34.7% less time), while
retaining 512 solved manifolds and 2,048 contact points.

The 2,500-body tree run's p95 improves from 7.12 to 5.56 ms, but it still records
7/1,200 steps over the 8.33 ms budget, versus 16/1,200 originally. All 10,000-body
runs still miss every deadline. These are unpaced simulations on a shared
machine, not a real-time guarantee or a multi-seed performance study.

A sleeping-enabled 2,500-body tree comparison improves from 6.77 to 5.40 ms.
The already fully sleeping 512-box control still solves zero contacts and does
not allocate the new body table; its small timing difference is not attributed
to this optimization. A short moving-sphere control changes from 4.7762 to
4.6033 ms, with solver time from 0.5106 to 0.3332 ms, but broadphase remains
dominant there.

### Numerical behavior and memory tradeoff

Body-only caching retains the original arithmetic ordering. A one-off
differential probe used 128 seeded four-body contact systems, four successive
solves each, including warm starts, rotated unequal-inertia boxes, two friction
axes, and mass changes. All 27,108 recorded velocity/impulse scalars matched
exactly for body-only caching. Combined contact-response caching produced a
maximum absolute difference of 1.34e-5 and a maximum scaled difference of
5.364e-6, using `abs(new - old) / max(1, abs(old))`.

Factoring `I^-1 * cross(r, axis * impulse)` into a cached response times the
impulse changes floating-point rounding. Those small local differences can
accumulate into different long trajectories and sleep decisions. At 10,000
mixed bodies, mean manifolds change from 16,137.58 to 16,177.59 and contact
points from 23,740.00 to 23,810.79. Final maximum floor penetration changes
from 0.0055 to 0.0148; maximum contact penetration over the measured interval
changes from 0.0863 to 0.0864. All measured cases remain finite and within the
floor, with final floor penetration below 0.02. No tolerances, iteration counts,
or sleep thresholds were relaxed to obtain the gain.

The solver regression compares uncached public impulse math with the prepared
solver across orientation, mass, static-state, and body-generation changes.
Existing friction, restitution, stacking, warm-start identity, backend-equivalence,
and sleep/wake regressions also remain covered.

On this arm64 build, the temporary body table costs 48 bytes per world body slot
and one additional allocation on a solve with prepared manifolds. Prepared
contact-point storage grows from 68 to 140 bytes, and the four-point prepared
manifold from 296 to 584 bytes. This is an arithmetic-for-memory tradeoff, not
an allocation optimization. The table is discarded each solve, avoiding stale
pose/mass data and additional public-body cache invalidation machinery.

Reproduce with the standard runners, comparing against `0e0da25` in a separate
build:

```sh
cmake --build build/release-bench --target phys_collision_bench phys_cpu_profile -j 4
./build/release-bench/phys_collision_bench 1200 tree mixed 240
./build/release-bench/phys_collision_bench 1200 sap mixed 240
./build/release-bench/phys_collision_bench 1200 tree mixed 240 --sleep
./build/release-bench/phys_cpu_profile boxes 3
```

**Both cache layers are retained.** The body-only layer is simpler and numerically
identical in the differential probe; the contact-response layer gives the larger
additional reduction while preserving the solver equations and processing order.

## Inlining the cross product

`Math3d::cross()` now has its unchanged definition inside the class in
`math_utils.hpp`, making it implicitly inline and visible to callers. Its
out-of-line definition was removed from `math_utils.cpp`; no other math helpers,
compiler flags, or solver equations changed. Release remains `-O3 -DNDEBUG`
without LTO.

Before this change, the repeated solver loop could call the cross-product helper
six times per contact point per iteration. After rebuilding, the Release solver
object contains no out-of-line cross-product symbol, and disassembly of the
Release profiling executable contains no calls to it.

The [comparison report](../benchmarks/results/inline-cross-release.txt) uses a
fresh preserved baseline from `331c3d3`, which already includes both solver
cache layers. Mixed rows are matched individual runs with 240 warmup and 1,200
measured steps. The box row is the median of three interleaved three-second
runs after 240 warmup steps. Sleeping is disabled in this table; times are ms:

| Workload | Before | Inline cross | Step-time reduction |
| --- | ---: | ---: | ---: |
| 2,500 mixed, tree | 5.37 | 4.89 | 8.9% |
| 10,000 mixed, tree | 25.76 | 24.33 | 5.6% |
| 10,000 mixed, SAP | 21.63 | 19.84 | 8.3% |
| 512 settled boxes, tree | 1.1829 | 1.0244 | 13.4% |

The corresponding solver times change from 2.89 to 2.42 ms, 12.13 to 10.52 ms,
12.03 to 10.28 ms, and 0.9792 to 0.8200 ms. Broadphase and narrowphase timings
are mostly unchanged. The fully sleeping box control is effectively flat
(0.2343 vs 0.2345 ms), while the short sphere control changes from 4.6651 to
4.5814 ms.

All 27,108 velocity/impulse scalars in the same seeded differential probe used
for solver caching match exactly in this build. Mixed-scene candidate/contact
loads, reported final state/penetration, traversal counts, and allocation counts
also match between versions. No new runtime buffers were added.

The 2,500-body tree run still misses 5/1,200 deadlines, and both 10,000-body runs
miss all deadlines. These unpaced, shared-machine measurements establish a mean
cost reduction, not a worst-case or real-time guarantee. Reproduce using the
same commands as the solver-cache comparison, with `331c3d3` as the baseline.

## Reproducing the Release workload

Build the selected source revision in a separate directory to leave the
sandbox's Debug build intact:

```sh
cmake -S . -B build/release-bench \
  -DCMAKE_BUILD_TYPE=Release \
  -DPHYS_BUILD_SANDBOX=OFF \
  -DPHYS_BUILD_BENCHMARKS=ON \
  -DBUILD_TESTING=ON
cmake --build build/release-bench --target phys_collision_bench -j 4
./build/release-bench/phys_collision_bench
```

The shared scaling CLI is:

```text
[measured_steps=1200] [sap|grid|tree] [mixed|spheres] [warmup_steps=240] [--sleep]
```

The scaling executable uses these defaults. To compare backends, keep the scene,
warmup, and measured steps identical:

```sh
./build/release-bench/phys_collision_bench 1200 sap mixed 240
./build/release-bench/phys_collision_bench 1200 grid mixed 240
./build/release-bench/phys_collision_bench 1200 tree mixed 240
./build/release-bench/phys_collision_bench 1200 sap mixed 240 --sleep
./build/release-bench/phys_collision_bench 1200 tree mixed 240 --sleep
./build/release-bench/phys_collision_bench 1200 tree spheres 240
cmake --build build/release-bench --target phys_broadphase_compare -j 4
./build/release-bench/phys_broadphase_compare 10
```

`phys_collision_bench --sleep` enables sleeping with the default scene and
step counts. The flag must be last and is applied before warmup.
`phys_broadphase_compare` remains the separate query-only comparison, and
`phys_cpu_profile spheres` retains the original profiling workload.

These commands benchmark the currently checked-out code. Use `fd8f5a1` for the
recorded all-pairs implementation and `9b9bc30` for the original indexed SAP,
with the same compiler and workload, when repeating the comparison. The
contiguous-record results were measured on the subsequent layout patch, before
it was committed. Historical commands use their recorded revisions and old
sample-cap behavior; `10 tree spheres 0` now means exactly ten measured steps
at every size, not the former adaptive counts. Increasing the sample argument
also changes how long each world evolves, so compare matching protocols.
