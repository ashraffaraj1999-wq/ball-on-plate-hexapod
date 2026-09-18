# Experimental Results

Logs captured from the physical rig in September 2021.

## Naming

| Pattern | Meaning |
|---|---|
| `x…` / `y…` | the two axes |
| `…step` | step-response test |
| `…circle` | circular trajectory tracking |
| `…res` | general run |
| trailing `2` | a repeat of the same test |

## Contents

| Controller | Tests | Notes |
|---|---|---|
| PD | step | `matlab.mat` holds `d_x`, `d_y` |
| Sliding mode | step, general runs | `alpha.mat` and `beta.mat` hold the commanded plate tilt angles |
| Super-twisting | step, circular tracking | |

Sample time is `Ts = 0.02 s` (50 Hz); sample index × 0.02 gives seconds.

## Loading the data

Variable names inside these files are short and undocumented. To list them:

```matlab
cd experiments/scripts
load_results
```

Then load the file required:

```matlab
load('../super_twisting/xcircle.mat');
```

## Plotting script

`scripts/pdplot.m` reconstructs the staircase reference used in the PD step test:

```matlab
first = 752;  last = 1674;
ref = [ones(1,first)*15, ones(1,last-first)*20, ones(1,length(x1)-last)*10];
t   = (1:length(x1)) * 0.02;
plot(t, x1, t, y1, t, ref)
```

The test commanded 15 cm → 20 cm → 10 cm, switching at t ≈ 15.0 s and t ≈ 33.5 s. It
expects `x1` and `y1` in the workspace.

## Coverage

Logging was not uniform across controllers. The control signal was recorded only for
sliding mode, and trajectory tracking only for super-twisting. Data for the PD, LQR, PID,
pole-placement and fuzzy runs was not retained.

The available data supports comparing PD, sliding mode and super-twisting on step
response, and characterising super-twisting on a circular trajectory.
