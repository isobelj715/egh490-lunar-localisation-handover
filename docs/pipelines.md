# Pipelines

The testbed structure-filtering chain, which is what every number in the final
report comes from.

Restart LeGO-LOAM between every replay. It keeps keyframes, the pose graph and
TF history in memory, so replaying into a live node makes timestamps jump
backwards and corrupts the SLAM state.

---

## The chain

Structure is removed from recorded scans offline, so the rover's real motion is
identical at every level and any difference in the estimate is the algorithm
responding to the scene.

```
data/<run>.bag  +  data/<run>_tf.bag        a testbed recording and its tf
  │
  ├─ pipeline/testbed/merge_halfscans.py    only for the 20 Hz half-scan bags
  │
  ├─ pipeline/testbed/measure_arena.py      box and yaw from an accumulated .pcd
  │                                            → results/testbed/arena_geometry.txt
  │
  ├─ pipeline/testbed/crop_all.sh           → <run>_level1/2/3.bag
  │     wraps crop_run.sh → crop_map_frame.py      + results/testbed/crop_logs/*.log
  │
  ├─ pipeline/testbed/brake.sh set <map> <feature>
  │     edits eignThre in mapOptmization.cpp and featureAssociation.cpp, rebuilds
  │
  ├─ pipeline/testbed/reprocess_lim.sh      replay harness, 3 phases
  │     wraps run_ablation.sh → record_run.py
  │     → results/testbed/<run>/est_raw_L*.txt, meta_L*.json, logs/L*/
  │
  ├─ pipeline/testbed/derive_structure.py   raw per-scan log → structure.csv
  │
  └─ analysis/testbed/report_tables.py      → report/tables/
     analysis/testbed/*.py                  → report/figs/
```

**Filtered levels.** L0 is the unmodified recording. L1–L3 remove progressively
more structure by cropping in the map frame: L1 to the outer walls, L2 to the
retaining-wall footprint, L3 adds a height ceiling that removes the overhead
crane.

**The L3 ceiling is ZHI = 0.0**, which is 0.652 m above the regolith, because the
map origin is the rover's start pose rather than the ground. 
The crop logs record what each crop actually removed.

**Replay at 0.3×.** At full rate the VM drops scans and the result looks like a
structure effect.

**`eignThre` is compiled in** and is not recorded in run metadata, so `git_sha`
will not tell you which build produced a result. Directory suffixes carry it by
convention: `m0f10` means map stage 0, feature stage 10. Upstream default is
map 100 / feature 10.

**Noise floor 0.012 m.** Repeated replays of identical input differ by about
that much, because the map stage processes whatever scans have arrived and
depends on CPU scheduling. Measured by `step4_repeats.sh`, reproducible from the
stored trajectories with `analysis/testbed/noise_floor.py`. Nothing smaller is a
real effect.

**Null control.** A run where the crop removes almost nothing should not move.
If it does, the change is coming from re-cropping and replaying rather than from
the removed points.

---

## The numbered steps

`pipeline/testbed/step0`–`step6` are the record of how the campaign was run,
each answering a specific validity question.

| script | question | writes |
|---|---|---|
| `step0_recrop.sh` | were scans silently dropped during cropping? | the level bags |
| `step1_sensitivity.sh` | does L3's result depend on where the box edge sits? | `results/testbed/levels` |
| `step2_sweep_walls.sh` | what happens between L1's λ_min and L2's? | `results/testbed/sweep` |
| `step3_refilter.sh` | walidate wall-filtering results using control conditions. | `results/testbed/refilter` |
| `step4_repeats.sh` | what is the noise floor? | `results/testbed/rep03` |
| `step5_final.sh` | every condition re-run at a rate the pipeline sustains | `results/testbed/final` |
| `step6_nobrake.sh` | is the drift from missing structure, or from the brake? | `results/testbed/nobrake` |

`step2`, `step3` and `step4` use `pipeline/testbed/filter_walls.py`, a RANSAC
vertical-plane filter with two dials — `keep_fraction` for random decimation and
`keep_sector_deg` for sector retention. Pass a different seed per repeat, or
every repeat shares one decimation pattern and they are not independent samples.

---

## Known issues

1. **One sensor config at a time.** `utility.h` holds each sensor variant as a
   commented-out block and switching means a `catkin_make`.
