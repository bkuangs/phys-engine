# Performance history

Measurements from September 16, 2026, on an Apple M2 running macOS. The main
algorithmic gain so far is replacing all-pairs broadphase with single-threaded,
single-axis sweep-and-prune (SAP). At 10,000 bodies, the recorded Release mean
step time fell from **197.83 ms to 13.40 ms**, approximately **14.76x faster**.
That is a substantial improvement, but still above the 8.33 ms budget for 120 Hz.
The subsequent [contiguous-record experiment](#contiguous-record-experiment)
improved SAP further using a fresh, repeated comparison.
An opt-in [uniform-grid prototype](#uniform-grid-prototype) is evaluated below;
SAP remains the engine default.

## Comparable checkpoints

| Checkpoint | Source | Raw report |
| --- | --- | --- |
| Debug, all-pairs broadphase | `fd8f5a1` | [Debug baseline](../benchmark-results-current.txt) |
| Release, all-pairs broadphase | `fd8f5a1` | [Release baseline](../benchmark-results-release.txt) |
| Release, X-axis SAP | `9b9bc30` | [SAP results](../benchmark-results-sap-release.txt) |
| Release, contiguous-record SAP | `9b9bc30` + layout patch (uncommitted when measured) | [Layout comparison](../benchmark-results-contiguous-release.txt) |

The baseline reports were committed in `0455cd0`. The SAP report was captured
before its implementation was committed in `9b9bc30`, so its original header
still describes the working tree as `0455cd0 + uncommitted single-axis SAP changes`.
The raw reports are retained unchanged.

The older [naive report](../benchmark-results-naive.txt) is historical context,
not the denominator for these comparisons: it does not record build type,
hardware, or source revision.

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
`-O3 -DNDEBUG -std=c++20 -arch arm64`. Both benchmark executables currently run
the same sphere workload; the broadphase executable is not an isolated SAP
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

The [comparison report](../benchmark-results-contiguous-release.txt) records
the summary and all per-run measurements. It uses fresh indexed controls,
rather than treating the earlier single-run 13.40 ms result as a controlled
before measurement. These are still short runs on a shared machine, so the
observed ratios should not be treated as universal speedups.

## Broadphase phase profile

The next measurement adds phase-boundary timers and work counters without
changing the collision algorithm. A control executable from `a6b5dd6` was
preserved, then three control runs and three instrumented runs were interleaved
using the same Release configuration, seed, and sample argument `10`.
The [profile report](../benchmark-results-broadphase-profile-release.txt)
contains all per-run timings and counts.

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

The [full-world comparison](../benchmark-results-grid-release.txt) uses three
interleaved runs per backend with the same seed, Release build, and argument
`10`. These are medians of per-run mean step times:

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

The separate [layout comparison](../benchmark-results-grid-layouts-release.txt)
times standalone broadphase queries on static AABBs, not full simulation steps.
Ten queries per backend are interleaved for each layout, with exact ordered
pair equality checked outside the timed region. At 10,000 AABBs:

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
./build/release-bench/phys_collision_bench 10
```

The algorithm argument is optional and defaults to `sap`. To compare the
experimental backend, use the same sample argument:

```sh
./build/release-bench/phys_collision_bench 10 sap
./build/release-bench/phys_collision_bench 10 grid
cmake --build build/release-bench --target phys_broadphase_compare -j 4
./build/release-bench/phys_broadphase_compare 10
```

`phys_broadphase_bench` accepts the same optional `sap`/`grid` argument as
`phys_collision_bench`; both run the full-world sphere workload.

These commands benchmark the currently checked-out code. Use `fd8f5a1` for the
recorded all-pairs implementation and `9b9bc30` for the original indexed SAP,
with the same compiler and workload, when repeating the comparison. The
contiguous-record results were measured on the subsequent layout patch, before
it was committed. Increasing the sample argument also changes how long each
world evolves, so compare runs with matching
arguments rather than mixing sample counts.
