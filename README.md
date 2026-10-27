# EGH490 — LiDAR-only localisation for a lunar rover

Honours capstone, QUT, 2026. Evaluates LeGO-LOAM as a LiDAR-only odometry
method for a planetary rover, using recordings from the Yandiwanba
lunar-analogue testbed.

Author: Isobel Jones  ·  Supervisor: A/Prof Thierry Peynot

## What the project does

Structure is removed from recorded LiDAR scans in controlled steps, offline, so
the rover's real motion is identical at every level and any difference in the
estimate is the algorithm responding to the scene. Drift is then measured
against how well-conditioned the remaining scene is, using the smallest
eigenvalue of LeGO-LOAM's map-stage information matrix (λ_min), logged per scan
by a custom `scan_logger.h` in the fork.

Four testbed recordings are cropped into filtered levels L0–L3, measuring how
far LeGO-LOAM drifts as the scene is reduced toward bare regolith, and how much
of that the algorithm's built-in degeneracy check recovers.

## Layout

| path | contents |
|---|---|
| `src/` | LeGO-LOAM fork (submodule) |
| `pipeline/testbed/` | cropping, filtering, replay and logging — produces `results/testbed/` |
| `analysis/testbed/` | turns results into the report's figures and tables |
| `results/testbed/` | per-run outputs: trajectories, metadata, per-scan structure logs |
| `report/` | the figures and tables that appear in the final report |
| `docs/` | pipeline details and LeGO-LOAM modifications |
| `data/` | raw recordings — **not in git**, see `data/README.md` |

## Build

Ubuntu 20.04, ROS Noetic.

```bash
git clone --recurse-submodules git@github.com:isobelj715/?? catkin_ws
cd catkin_ws
catkin_make
source devel/setup.bash
```

The sensor configuration is compiled in via
`src/LeGO-LOAM/LeGO-LOAM/include/utility.h`, currently set for the testbed's
32-beam Velodyne HDL-32E.

## Reproducing the results

Restore the recordings into `data/` first — see `data/README.md`.

```bash
pipeline/testbed/crop_all.sh          # build the L1/L2/L3 bags from each recording
pipeline/testbed/reprocess_lim.sh     # replay them, write results/testbed/
analysis/testbed/report_tables.py     # regenerate the report tables
```

`docs/pipelines.md` has the full chain, including the numbered `step0`–`step6`
scripts that record how the campaign was actually run.

##  Important notes

**Replay speed.** Bags must be replayed at 0.3×. At full rate the VM drops
scans and the result looks like a structure effect — two identical runs
differed by metres at full rate, against 0.012 m at 0.3×.

**`eignThre` is compiled in.** The degeneracy threshold is a literal in
`mapOptmization.cpp` (map stage) and `featureAssociation.cpp` (odometry stage),
so it is not recorded in run metadata and `git_sha` will not tell you which
build produced a result. Set it with `pipeline/testbed/brake.sh` and keep track
yourself. Upstream default is map 100 / feature 10; result directories carry the
setting by naming convention, `m0f10` meaning map 0, feature 10.

**Segmentation is not upstream.** `segmentTheta` 20°, `segmentValidPointNum` 3,
`segmentValidLineNum` 2, all relaxed from the defaults so small clusters survive
in sparse terrain. Unchanged since May 2026 and identical across every run.

**Loop closure is off** (`loopClosureEnableFlag = false`). All results are pure
odometry plus scan-to-map refinement — no pose-graph correction. In an arena the
rover revisits constantly, a reader will assume otherwise unless told.

**Noise floor.** Repeated replays of identical input differ by about 0.012 m,
because the map stage processes whatever scans have arrived and depends on CPU
scheduling. Nothing smaller than that is a real effect.

