
### 1. Per-run summary  (from meta_L0.json and est_raw_L0.txt)

| run       | session    | shape            | path L0 [m] | duration [s] | parked open [s] | parked close [s] | mean speed [m/s] | poses L0 |
|-----------|------------|------------------|-------------|--------------|-----------------|------------------|------------------|----------|
| september | 2026-09-16 | open traverse    | --          | --           | 4.8             | 4.8              | 0.50             | --       |
| rectangle | 2026-10-07 | closed rectangle | 38.20       | 102.9        | 16.0            | 15.6             | 0.53             | 258      |
| ellipse1  | 2026-10-07 | closed ellipse   | 21.65       | 72.9         | 18.0            | 16.0             | 0.55             | 183      |
| zigzag    | 2026-10-07 | open zigzag      | 12.94       | 56.8         | 14.8            | 11.6             | 0.42             | 143      |

### 2. Median lambda_min per level  (mapped scans only; eignThre = 100)

| run       | L0 median lambda_min | L1 median lambda_min | L2 median lambda_min | L3 median lambda_min |
|-----------|----------------------|----------------------|----------------------|----------------------|
| september | 438.2 (over)         | 387.9 (over)         | 27.3                 | 9.1                  |
| rectangle | 376.9 (over)         | 336.1 (over)         | 28.4                 | 6.5                  |
| ellipse1  | 277.8 (over)         | 262.4 (over)         | 22.3                 | 4.1                  |
| zigzag    | 363.0 (over)         | 338.2 (over)         | 19.7                 | 2.5                  |

### 3. Scans on which the threshold fires  (% of MAPPED scans)

| run       | L0 fires [%] | L1 fires [%] | L2 fires [%] | L3 fires [%] |
|-----------|--------------|--------------|--------------|--------------|
| september | 0.0          | 0.0          | 100.0        | 100.0        |
| rectangle | 0.0          | 0.0          | 100.0        | 100.0        |
| ellipse1  | 0.0          | 0.0          | 100.0        | 100.0        |
| zigzag    | 0.0          | 0.0          | 100.0        | 100.0        |

   base for those percentages:

| run       | L0 mapped scans | L1 mapped scans | L2 mapped scans | L3 mapped scans |
|-----------|-----------------|-----------------|-----------------|-----------------|
| september | 239             | 239             | 239             | 239             |
| rectangle | 239             | 239             | 239             | 239             |
| ellipse1  | 179             | 159             | 159             | 179             |
| zigzag    | 139             | 119             | 119             | 139             |

### 4. Path length per level, and L3 against L0

| run       | L0 [m] | L1 [m] | L2 [m] | L3 [m] | L3 / L0 |
|-----------|--------|--------|--------|--------|---------|
| september | --     | --     | --     | 68.04  | --      |
| rectangle | 38.20  | 38.22  | 34.88  | 60.22  | 1.58x   |
| ellipse1  | 21.65  | 21.68  | 25.40  | 60.31  | 2.79x   |
| zigzag    | 12.94  | 12.93  | 14.95  | 35.62  | 2.75x   |

   Path length is est_path_len_m, the length of the ESTIMATED trajectory. At
   L0 that is close to the rover's true path; at L3 it is not, and the last
   column is the inflation -- a rigid rotation preserves path length exactly,
   so anything above 1.0 is translation error, not heading error.

   The firing percentages are over scans where the mapping branch actually
   ran. is_degenerate and map_full_lambda_min are blank when map_ran is 0,
   and counting those rows would dilute both numbers.

   wrote 5 CSVs to report_v2/tables
