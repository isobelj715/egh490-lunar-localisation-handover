# LeGO-LOAM: modifications and configuration

Fork of `RobustFieldAutonomyLab/LeGO-LOAM`, branch `egh490`.

Two main commits on top of upstream `master` (`896a7a9`):

| commit | date | what |
|---|---|---|
| `d31f6a9` | 11 May 2026 | Noetic build fixes, frame and node names, segmentation parameters |
| `dbcc1c2` | 27 Oct 2026 | `scan_logger.h` instrumentation, HDL-32E sensor config, timestamp tolerance, eignThre set to 0/0 |

---

## 1. Instrumentation — `include/scan_logger.h`

New file, added for this project. Writes one CSV row per scan without altering
pipeline behaviour. Output directory comes from the `LEGO_LOG_DIR` environment
variable; note it resolves relative to the process working directory, which for
a ROS node is `~/.ros` unless set explicitly.

Logged per scan:

- the full 6×6 information matrix from the map stage, reduced to eigenvalue
  summaries: `map_full_lambda_min/max/trace/cond/logdet`, and the same for the
  rotation block, the translation block and both Schur complements
- `is_degenerate`, `map_ran`, `iter_count`, `n_corr`, `residual_rms_init`
- feature counts: `n_corner_sharp`, `n_corner_less`, `n_surf_flat`,
  `n_surf_less`, `n_corr_corner`, `n_corr_surf`, `n_segmented`
- bearing, elevation and range histograms of the matched features, each as
  occupancy fraction, normalised entropy and largest empty arc
- scatter moments (linearity, planarity, sphericity) for corner and surf sets

`pipeline/testbed/derive_structure.py` turns the raw log into `structure.csv`,
which is what every analysis script reads.

**Overhead was measured, not assumed:** three runs with logging on and three
with it off gave 0.101 m and 0.103 m ATE, inside normal run-to-run variation,
with no dropped scans.

**A column absent from the first row is dropped for the whole run.** The logger
freezes its schema on the first write, so `declare()` must be called for every
field up front. This bit once already.

---

## 2. The degeneracy threshold (`eignThre`)

LeGO-LOAM checks the conditioning of each optimisation and suppresses updates
in directions whose eigenvalue falls below a threshold. The threshold is a
**literal in the source**, in two places:

| file | array | upstream value | what it guards |
|---|---|---|---|
| `src/mapOptmization.cpp` | `eignThre[6]` | 100 | map stage — scan-to-map refinement |
| `src/featureAssociation.cpp` | `eignThre[3]`, `eignThre[6]` | 10 | odometry stage — scan-to-scan |

Because it is compiled in, it appears in no launch file, no parameter server and
no run metadata. **`git_sha` in a `meta_*.json` does not identify which threshold
produced that run.** `pipeline/testbed/brake.sh set <map> <feature>` edits both
files and rebuilds; result directories carry the setting by naming convention,
`m0f10` meaning map 0, feature 10.

**As committed both are 0**, i.e. no suppression. That is a starting state, not
the configuration behind any particular result.

Configurations used:

| condition | map | feature |
|---|---|---|
| stock | 100 | 10 |
| no brake | 0 | 10 |
| threshold sweep | 5, 10, 25, 50, 100 | 10 |

**What the threshold does to the results.** At L0 and L1 the check never fires;
at L2 and L3 it fires on every mapped scan, on all four runs. Removing it at L2
increases end-of-run error by between 39× and 178× depending on the trajectory.
At L3 the map-stage check still helps, but the **odometry-stage** check does
too — setting the feature stage to 0 at L3 made September's error worse, 14.5 m
against 3.1 m, so `featureAssociation.cpp` should be left as shipped.

**The `is_degenerate` column is λ_min thresholded at that hard-coded 100, not an
independent measurement.** Do not present it as corroboration of λ_min.

---

## 3. Sensor configuration

`utility.h` holds every sensor variant as a commented-out block. Exactly one is
live and changing it requires `catkin_make`.

**Currently active: Velodyne HDL-32E**, the testbed sensor.

```cpp
N_SCAN        = 32
ang_res_x     = 360.0 / Horizon_SCAN
ang_res_y     = 1.330          // measured median spacing
ang_bottom    = 30.67 + 0.1
useCloudRing  = true           // ring field present and monotonic
```

With `useCloudRing = true` the ring field assigns rows, so `ang_res_y` and
`ang_bottom` no longer drive row indexing — but `ang_res_y` still feeds
`segmentAlphaY`, so it continues to affect segmentation. See §4.

---

## 4. Segmentation parameters

Changed from upstream in `d31f6a9` and unchanged since, so identical across
every run.

| parameter | upstream | ours | effect |
|---|---|---|---|
| `segmentTheta` | 60° | 20° | **Relaxed.** In `labelComponents()` a neighbour joins the cluster when the computed angle *exceeds* the threshold, and that angle shrinks as the range step grows — so lowering it admits pairs with bigger depth jumps |
| `segmentValidPointNum` | 5 | 3 | Relaxed — smaller clusters accepted |
| `segmentValidLineNum` | 3 | 2 | Relaxed — fewer rings required per cluster |

Clusters of 30+ points are always valid; below that they need at least
`segmentValidPointNum` points spanning `segmentValidLineNum` rings. All three
changes push the same way: more small clusters from rocks and crater rims
survive instead of being discarded as noise.

The comparison angle derives from the sensor's angular resolution via
`segmentAlphaY`, so identical parameter values give different effective
behaviour on sensors with different beam spacing. For the HDL-32E it is 1.33°.

---

## 5. Build fixes — Ubuntu 20.04 / ROS Noetic

| file | change |
|---|---|
| `CMakeLists.txt` | `-std=c++11` → **`-std=c++17`**|
| | `find_package(Boost ... serialization thread timer chrono)` and `${Boost_LIBRARIES}` linked into `mapOptmization`, needed by newer GTSAM |
| `include/utility.h` | `<opencv/cv.h>` → `<opencv2/opencv.hpp>`; added `PCL_NO_PRECOMPILE`, `<cstdint>`, Eigen headers |
| | `extern const string` → `inline const string` (C++17 ODR) |
| | `uint16_t ring` → `std::uint16_t ring` in `PointXYZIR` |
| all four nodes | leading-slash frame IDs (`"/camera_init"`) → relative; tf2 in Noetic rejects leading slashes |
| all four nodes | `ros::init` name changed from the shared `"lego_loam"` to each node's own |
| `src/mapOptmization.cpp` | timestamp tolerance 0.005 s → **0.15 s** in all three checks in `run()` |

---

## 6. Other configuration worth stating

- **`loopClosureEnableFlag = false`.** All results are pure odometry plus
  scan-to-map refinement over a 50 m keyframe radius, with no pose-graph loop
  closure.
- `sensorMinimumRange = 1.0` — everything within 1 m is discarded.
- `surroundingKeyframeSearchRadius = 50.0` exceeds the whole testbed, so every
  keyframe is always in the local map.
- `run.launch` hardcodes `use_sim_time = true`. With no `/clock` publisher,
  `rospy.Time.now()` returns 0, which breaks the live-publisher path.
