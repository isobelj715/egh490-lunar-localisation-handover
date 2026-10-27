# data/

Raw recordings and external datasets. **Nothing in this directory is tracked in
git** except this file — see `.gitignore`. The pipeline reads from here, so the
recordings have to be restored before anything in `pipeline/` or `analysis/`
will run.

## Where the recordings live

In my shared document folder in the project Teams group, under `Source recordings`.

Copy everything listed below into this directory and extract the archives in
place, so the files end up with their original names. **Filenames are
important**: `crop_all.sh`, `crop_run.sh` and `reprocess_lim.sh` locate bags
by name.
 
| in the Teams folder | extracts to |
|---|---|
| `egh490_tf_bags.zip` | all the `*_tf.bag` companion files |
| `2026-09-16-07-03-07.zip` | `2026-09-16-07-03-07.bag` |
| `rosbag2_2026_08_31-17_09_53.7z` | a ROS 2 bag directory |
| `rosbag2_2026_08_31-17_12_49.7z` | a ROS 2 bag directory |
 
The remaining `.bag` files are uploaded loose and dont need extraction (just copy loose into this /data directory).

## Source recordings

Irreplaceable. Each is a single testbed or simulator session and cannot be
regenerated from anything in this repository.

| file | size | session | notes |
|---|---|---|---|
| `2026-09-16-07-03-07.bag` | 1.3 G | 16 Sep 2026, 99 s | loop — **reported** |
| `rectangle_2026-10-07-06-53-20.bag` | 1.4 G | 7 Oct 2026 | rectangle — **reported** |
| `ellipse1_2026-10-07-06-56-59.bag` | 953 M | 7 Oct 2026 | ellipse 1 — **reported** |
| `zigzag_2026-10-07-07-01-17.bag` | 748 M | 7 Oct 2026 | zigzag — **reported** |
| `ellipse2_2026-10-07-06-59-01.bag` | 883 M | 7 Oct 2026 | ellipse 2, recorded but not reported |

Recorded by Clayder Alejandro Gonzalez Cadenillas. Sensor: Velodyne HDL-32E, 32
beams, 10 Hz, mounted 0.652 m above the regolith.

The four marked **reported** are the runs behind every number in the final
report. Approximately 6 GB with their tf bags. 

A sixth recording from the same session, `ribbon_2026-10-07-07-03-34.bag`
(851 M), exists on the original VM but is **not in the archive**. It was not
used in any reported result.

### Companion tf bags — archive these too

Each recording has a tf-only bag covering the same time window, replayed
alongside the point-cloud bag so `crop_map_frame.py` can transform points into
the map frame. They are inputs to the pipeline; nothing in this repository
produces them, so without them **no crop can be rebuilt**.
 
All of them are in `egh490_tf_bags.zip`:
 
| recording | tf bag |
|---|---|
| `2026-09-16-07-03-07.bag` | `yandiwanba3_tf.bag` (2.7 M) |
| `rectangle_…`, `ellipse1_…`, `ellipse2_…`, `zigzag_…` | `<run>_tf.bag` |
| the August pilot passes | `pass1_tf.bag`, `pass2_tf.bag` |
 
`reprocess_lim.sh` holds the authoritative bag/tf pairing for each run, along
with its yaw and crop box.

## Derived files — regenerate 

Everything below is produced by scripts in this repository from the source
recordings above. They are excluded from git and should be left out of any
archive; rebuilding them is faster than moving them.

| pattern | produced by |
|---|---|
| `*_level1.bag`, `*_level2.bag`, `*_level3.bag` | `pipeline/testbed/crop_run.sh` |
| `*_lim_level3.bag` | `pipeline/testbed/crop_all.sh` (ZHI = 0.0) |
| `level1.bag`, `level2.bag`, `level3.bag` | September session crops |
| `september_lim_level3.bag`, `september_v2_level2.bag` | September session crops |
| `*_merged.bag` | `pipeline/testbed/merge_halfscans.py` |
| `maps/*.pcd`, `maps/*.npy`, `arena_map.pcd` | `pipeline/testbed/accumulate_map.py` |
| `pcd/` | LeGO-LOAM map output |

To rebuild the filtered levels for one recording:

    cd ~/catkin_ws/data
    RUN=rectangle BAG=rectangle_2026-10-07-06-53-20.bag TF=rectangle_tf.bag \
        ../pipeline/testbed/crop_run.sh

## Rebuilding everything from scratch

1. Restore the source recordings and their tf bags into this directory.
2. `pipeline/testbed/crop_all.sh` — regenerates the L1/L2/L3 bags.
3. `pipeline/testbed/reprocess_lim.sh` — replays them and writes `results/testbed/`.
4. `analysis/testbed/report_tables.py` — regenerates the report tables.

Replays must run at 0.3× or scans are dropped. See `docs/pipelines.md`.
