
### 1. Per-run summary  (from meta_L0.json and est_raw_L0.txt)

| run       | session    | shape         | path L0 [m] | duration [s] | parked open [s] | parked close [s] | mean speed [m/s] | poses L0 |
|-----------|------------|---------------|-------------|--------------|-----------------|------------------|------------------|----------|
| september | 2026-09-16 | open traverse | 44.89       | 99.3         | 4.8             | 4.8              | 0.50             | 249      |

### 2. Median lambda_min per level  (mapped scans only; eignThre = 100)

| run       | L0 median lambda_min | L1 median lambda_min | L2 median lambda_min | L3 median lambda_min |
|-----------|----------------------|----------------------|----------------------|----------------------|
| september | 438.2 (over)         | 387.9 (over)         | 31.2                 | 11.6                 |

### 3. Scans on which the threshold fires  (% of MAPPED scans)

| run       | L0 fires [%] | L1 fires [%] | L2 fires [%] | L3 fires [%] |
|-----------|--------------|--------------|--------------|--------------|
| september | 0.0          | 0.0          | 100.0        | 100.0        |

   base for those percentages:

| run       | L0 mapped scans | L1 mapped scans | L2 mapped scans | L3 mapped scans |
|-----------|-----------------|-----------------|-----------------|-----------------|
| september | 239             | 239             | 239             | 239             |

### 4. Path length per level, and L3 against L0

| run       | L0 [m] | L1 [m] | L2 [m] | L3 [m] | L3 / L0 |
|-----------|--------|--------|--------|--------|---------|
| september | 44.89  | 44.85  | 41.28  | 71.71  | 1.60x   |

   Path length is est_path_len_m, the length of the ESTIMATED trajectory. At
   L0 that is close to the rover's true path; at L3 it is not, and the last
   column is the inflation -- a rigid rotation preserves path length exactly,
   so anything above 1.0 is translation error, not heading error.

   The firing percentages are over scans where the mapping branch actually
   ran. is_degenerate and map_full_lambda_min are blank when map_ran is 0,
   and counting those rows would dilute both numbers.
