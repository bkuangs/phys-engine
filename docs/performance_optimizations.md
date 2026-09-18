# Merged performance optimizations

This is the short version of the optimizations retained on `main`. Measurements
come from controlled Release comparisons at the time each change was made.
Different rows use different workloads and baselines, so their gains should not
be added together. Detailed protocols and measurements remain in
[`benchmarks/results`](../benchmarks/results).

## 1. Sweep-and-prune broadphase

- **Problem:** The engine compared every object with every other object. At
  10,000 bodies, that meant almost 50 million possible pairs per step.
- **Solution:** Sort objects along X and stop scanning once objects are too far
  apart to collide.
- **Gain:** The original 10k workload dropped from 197.83 ms to 13.40 ms, about
  14.8x faster.

## 2. Contiguous SAP records

- **Problem:** The SAP loop followed sorted indices back into an unsorted AABB
  array, causing scattered memory reads.
- **Solution:** Sort records containing both the AABB and its original index so
  the sweep reads memory sequentially.
- **Gain:** The 10k workload dropped from 11.86 ms to 8.97 ms, about 24% faster.

## 3. Dynamic AABB tree

- **Problem:** SAP still sorts and scans objects every frame, even when much of
  the world changes only slightly.
- **Solution:** Keep a spatial tree between frames and update only objects that
  move outside their padded bounds.
- **Gain:** On the original 10k-body benchmark, SAP took 9.33 ms and the tree
  took 5.11 ms, making the tree about 1.8x faster.
- **Caveat:** The tree remains optional because SAP is faster for some smaller
  or simpler scenes.

## 4. Indexed warm-start lookup

- **Problem:** For each current contact, the solver searched through the entire
  previous-contact cache for a match.
- **Solution:** Group cached contacts by body pair, then search only the
  relevant group.
- **Gain:** Settled boxes improved by about 38%; the 10k-sphere workload
  improved by 7.4%.

## 5. Sleeping islands

- **Problem:** Completely settled stacks were still integrated and solved every
  frame.
- **Solution:** Put whole connected groups of resting bodies to sleep and wake
  them together when disturbed.
- **Gain:** A settled 512-box scene dropped from 1.81 ms to 0.24 ms, about 7.6x
  faster.
- **Caveat:** Sleeping is opt-in because tracking sleep state adds roughly 5.8%
  overhead when nothing can sleep.

## 6. Cached solver calculations

- **Problem:** During eight solver iterations, the engine repeatedly calculated
  body inertia and contact-response math that did not change.
- **Solution:** Calculate those values once before the iterations and reuse
  them.
- **Gain:** Mixed workloads improved by roughly 17-22%; settled boxes improved
  by about 31%.

## 7. Inlined cross-product math

- **Problem:** The hottest solver loop repeatedly called a tiny cross-product
  function instead of compiling it directly into the loop.
- **Solution:** Make its definition visible to the optimizer so the calls
  disappear.
- **Gain:** About 6-13% faster, depending on the workload.

## 8. Reusable step workspaces

- **Problem:** Every simulation step created and destroyed temporary vectors
  for broadphase, tree traversal, contacts, and solver preparation.
- **Solution:** Keep private scratch buffers in each world and reuse their
  capacity.
- **Gain:** Removed approximately 94-98% of allocation churn.
- **Important:** Average step time remained effectively flat, but allocation
  behavior became much more predictable.

## 9. Coupled friction solve

- **Problem:** The solver handled two friction directions separately, repeating
  velocity reads and body updates. The old square clamp could also apply too
  much total friction.
- **Solution:** Solve both directions together as one 2D problem and clamp the
  result to a circular friction limit.
- **Gain:** The velocity loop improved by 25-27%, producing a 9-11% full-step
  improvement.
- **Behavior:** This intentionally changes trajectories slightly because the
  circular friction limit is more physically correct.

## 10. Specialized SAP overlap check

- **Problem:** SAP checked X overlap again even though sorting and the sweep
  window had already proven it.
- **Solution:** Check only Y and Z inside the hot loop.
- **Gain:** The SAP sweep improved by 11-13%, producing a 1.9-3.0% full-step
  improvement with exact behavior preserved.

## 11. Derived post-normal tangent velocity

- **Problem:** Every solver iteration recalculated both bodies' contact
  velocities after applying the normal impulse.
- **Solution:** Cache how a normal impulse affects each friction direction and
  derive the post-normal tangent velocity from the first calculation.
- **Gain:** The velocity loop improved by 27-28%, producing an 8-10% mixed-scene
  improvement and an 18.5% settled-box improvement.
- **Behavior:** The equations are unchanged, but reordered floating-point math
  causes small long-run trajectory differences.

## 12. Linear SAP pair ordering

- **Problem:** SAP used comparison sorting to restore deterministic order for
  roughly 12,000-25,000 candidate pairs per mixed step.
- **Solution:** Use two reusable stable counting passes for dense output, with
  `std::sort` retained as a sparse-output fallback.
- **Gain:** Pair ordering improved by about 90%, broadphase by 18-20%, and full
  steps by 3.6-4.5%.
- **Behavior:** Candidate order and simulation results remain exact.

## 13. Deterministic parallel narrowphase

- **Problem:** Contact generation for independent candidate pairs ran serially,
  consuming about 1.8 ms at 5,000 bodies and 3.4 ms at 10,000.
- **Solution:** Use persistent world-owned workers and per-candidate result
  slots, then compact contacts in their original order.
- **Gain:** Four workers reduced narrowphase time by about 66% and full-step
  time by 15-17%.
- **Caveat:** Parallel narrowphase is opt-in; one worker remains the default,
  and small candidate sets stay serial.
- **Behavior:** Contact order and fixed-step simulation output remain exact.
