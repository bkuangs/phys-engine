# Performance history

Measurements from September 16, 2026, on an Apple M2 running macOS. The main
algorithmic gain so far is replacing all-pairs broadphase with single-threaded,
single-axis sweep-and-prune (SAP). At 10,000 bodies, the recorded Release mean
step time fell from **197.83 ms to 13.40 ms**, approximately **14.76x faster**.
That is a substantial improvement, but still above the 8.33 ms budget for 120 Hz.
The subsequent [contiguous-record experiment](#contiguous-record-experiment)
improved SAP further using a fresh, repeated comparison.

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
follow-up. Separating sorting, scanning, and filtering timings would help
identify the remaining cost.

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

These commands benchmark the currently checked-out code. Use `fd8f5a1` for the
recorded all-pairs implementation and `9b9bc30` for the original indexed SAP,
with the same compiler and workload, when repeating the comparison. The
contiguous-record results were measured on the subsequent layout patch, before
it was committed. Increasing the sample argument also changes how long each
world evolves, so compare runs with matching
arguments rather than mixing sample counts.
