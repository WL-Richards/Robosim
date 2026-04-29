# RIO 2 HAL call cost fixtures

Versioned per-call cost measurements (mean / stddev / p99 / outlier rate)
captured by `tools/rio-bench/` running on a physical RoboRIO 2.

Filename convention: `<wpilib-version>-<rio-firmware>.csv`.

Loaded by the sim core at startup so tier 1 (native) can apply realistic
HAL-call jitter without an actual RIO. See
`docs/ARCHITECTURE.md` "Execution tiers" — tier 1 fidelity comes from
this table plus cgroups/CPU throttling.

Until real bench data is available the file `_estimated.csv` (added when
tier 2 work begins) is used and tagged `unvalidated`. Real measurements
overwrite it once bench access is arranged. See
`docs/OPEN_QUESTIONS.md` OQ-10.
