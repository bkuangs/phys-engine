# Performance

For a concise history of retained improvements, see
[merged performance optimizations](performance_optimizations.md). Detailed
measurements remain in [`benchmarks/results`](../benchmarks/results).

## Current benchmark

The primary workload is a seeded mixed scene with equal numbers of spheres and
boxes over a static floor. It uses the default SAP broadphase, disables
sleeping, warms up for 240 steps, and measures Release builds on an Apple M2.

The latest stable three-run profile is:

| Dynamic bodies | Full step | Broadphase | Narrowphase | Solver | Approximate rate |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 5,000 | 6.8936 ms | 1.5849 ms | 1.8535 ms | 3.3259 ms | 145 Hz |
| 10,000 | 14.7683 ms | 3.9595 ms | 3.4684 ms | 7.1810 ms | 68 Hz |

These are medians of three per-run means, not worst-case guarantees. At 5,000
bodies the mean is below the 8.33 ms budget for 120 Hz. At 10,000 bodies it is
not.

## Current bottlenecks

At both target sizes, the solver remains roughly half of the complete step.
Its fixed eight velocity iterations are the largest solver subphase.

SAP sweep-and-emit is the largest broadphase subphase. Pair ordering previously
cost about 0.5 ms at 5,000 bodies and 1.1 ms at 10,000; stable counting passes
reduced that to about 0.05 and 0.10 ms.

Narrowphase is also substantial. Box-box candidates account for most sampled
narrowphase time, so it is the clearest future parallel workload.

## Broadphase guidance

- **SAP:** Default. Strong general-purpose performance and deterministic output.
- **Dynamic tree:** Opt-in. Best for many persistent, incrementally moving
  objects, but not universally faster.
- **Uniform grid:** Opt-in. Useful for uniformly sized objects; large or
  irregular objects can make it substantially slower.

All backends return the same lexicographically ordered candidate pairs.

## Measurement rules

- Compare matching source revisions, compiler flags, scenes, seeds, warmup, and
  step counts.
- Use Release builds (`-O3 -DNDEBUG`); Debug timing is not representative.
- Interleave control and experimental runs to reduce machine-load bias.
- Compare behavior and stability as well as elapsed time.
- Do not add percentages from separate experiments; each uses its own baseline.
- Treat subphase timers as attribution tools, not independent wall-clock totals.

## Reproduce

```sh
cmake -S . -B build/release-bench \
  -DCMAKE_BUILD_TYPE=Release \
  -DPHYS_BUILD_SANDBOX=OFF \
  -DPHYS_BUILD_BENCHMARKS=ON \
  -DBUILD_TESTING=ON
cmake --build build/release-bench -j 4
```

Run the sustained mixed workload:

```sh
./build/release-bench/phys_collision_bench 1200 sap mixed 240
```

Compare broadphase backends:

```sh
./build/release-bench/phys_collision_bench 1200 grid mixed 240
./build/release-bench/phys_collision_bench 1200 tree mixed 240
./build/release-bench/phys_broadphase_compare 10
```

Run focused CPU-profile workloads:

```sh
./build/release-bench/phys_cpu_profile mixed5k 20
./build/release-bench/phys_cpu_profile mixed10k 20
./build/release-bench/phys_cpu_profile boxes 20
```

Append `--sleep` to a scaling or profile command to enable sleeping before
warmup.

## Detailed reports

| Area | Reports |
| --- | --- |
| Current SAP | [linear pair ordering](../benchmarks/results/sap-linear-pair-ordering-release.txt), [redundant X checks](../benchmarks/results/sap-redundant-x-release.txt) |
| Current solver | [derived tangent velocity](../benchmarks/results/derived-tangent-velocity-release.txt), [coupled friction](../benchmarks/results/coupled-friction-release.txt), [hot-subphase profile](../benchmarks/results/mixed-sap-hot-subphases.txt) |
| Earlier solver work | [invariant caching](../benchmarks/results/solver-cache-release.txt), [inline cross product](../benchmarks/results/inline-cross-release.txt), [warm-start indexing](../benchmarks/results/warmstart-index.txt) |
| Runtime behavior | [workspace reuse](../benchmarks/results/workspace-reuse-release.txt), [sleeping islands](../benchmarks/results/sleeping-release.txt) |
| Broadphase | [dynamic tree](../benchmarks/results/tree-release.txt), [tree layouts](../benchmarks/results/tree-layouts-release.txt) |
| Sustained baselines | [SAP](../benchmarks/results/mixed-sustained-sap-release.txt), [tree](../benchmarks/results/mixed-sustained-tree-release.txt), [sleeping comparison](../benchmarks/results/mixed-sleeping-comparison.txt) |
