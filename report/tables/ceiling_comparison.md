
### 1. Per-run summary  (from meta_L0.json and est_raw_L0.txt)

| run      | session | shape | path L0 [m] | duration [s] | parked open [s] | parked close [s] | mean speed [m/s] | poses L0 |
|----------|---------|-------|-------------|--------------|-----------------|------------------|------------------|----------|
| rect-new | --      | --    | 38.20       | 102.9        | 16.0            | 15.6             | 0.53             | 258      |
| ell-new  | --      | --    | 21.65       | 72.9         | 18.0            | 16.0             | 0.55             | 183      |
| zig-new  | --      | --    | 12.94       | 56.8         | 14.8            | 11.6             | 0.42             | 143      |
| rect-old | --      | --    | --          | --           | --              | --               | --               | --       |
| ell-old  | --      | --    | --          | --           | --              | --               | --               | --       |
| zig-old  | --      | --    | --          | --           | --              | --               | --               | --       |

### 2. Median lambda_min per level  (mapped scans only; eignThre = 100)

| run      | L0 median lambda_min | L1 median lambda_min | L2 median lambda_min | L3 median lambda_min |
|----------|----------------------|----------------------|----------------------|----------------------|
| rect-new | 376.9 (over)         | 336.1 (over)         | 28.4                 | 6.5                  |
| ell-new  | 277.8 (over)         | 262.4 (over)         | 22.3                 | 4.1                  |
| zig-new  | 363.0 (over)         | 338.2 (over)         | 19.7                 | 2.5                  |
| rect-old | --                   | --                   | --                   | 7.6                  |
| ell-old  | --                   | --                   | --                   | 3.1                  |
| zig-old  | --                   | --                   | --                   | 2.7                  |

### 3. Scans on which the threshold fires  (% of MAPPED scans)

| run      | L0 fires [%] | L1 fires [%] | L2 fires [%] | L3 fires [%] |
|----------|--------------|--------------|--------------|--------------|
| rect-new | 0.0          | 0.0          | 100.0        | 100.0        |
| ell-new  | 0.0          | 0.0          | 100.0        | 100.0        |
| zig-new  | 0.0          | 0.0          | 100.0        | 100.0        |
| rect-old | --           | --           | --           | 100.0        |
| ell-old  | --           | --           | --           | 100.0        |
| zig-old  | --           | --           | --           | 100.0        |

   base for those percentages:

| run      | L0 mapped scans | L1 mapped scans | L2 mapped scans | L3 mapped scans |
|----------|-----------------|-----------------|-----------------|-----------------|
| rect-new | 239             | 239             | 239             | 239             |
| ell-new  | 179             | 159             | 159             | 179             |
| zig-new  | 139             | 119             | 119             | 139             |
| rect-old | --              | --              | --              | 239             |
| ell-old  | --              | --              | --              | 179             |
| zig-old  | --              | --              | --              | 119             |

### 4. Path length per level, and L3 against L0

| run      | L0 [m] | L1 [m] | L2 [m] | L3 [m] | L3 / L0 |
|----------|--------|--------|--------|--------|---------|
| rect-new | 38.20  | 38.22  | 34.88  | 60.22  | 1.58x   |
| ell-new  | 21.65  | 21.68  | 25.40  | 60.31  | 2.79x   |
| zig-new  | 12.94  | 12.93  | 14.95  | 35.62  | 2.75x   |
| rect-old | --     | --     | --     | 56.40  | --      |
| ell-old  | --     | --     | --     | 55.51  | --      |
| zig-old  | --     | --     | --     | 34.13  | --      |

   Path length is est_path_len_m, the length of the ESTIMATED trajectory. At
   L0 that is close to the rover's true path; at L3 it is not, and the last
   column is the inflation -- a rigid rotation preserves path length exactly,
   so anything above 1.0 is translation error, not heading error.

   The firing percentages are over scans where the mapping branch actually
   ran. is_degenerate and map_full_lambda_min are blank when map_ran is 0,
   and counting those rows would dilute both numbers.
