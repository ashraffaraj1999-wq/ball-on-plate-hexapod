# Contents

A `.m` file is generally a parameter script that must be run before opening the `.slx` of
the same name; the models read their gains from the base workspace.

---

## `src/model/`

| File | Description |
|---|---|
| `Euler_Lagrange_Modelling.m` | Symbolic derivation of the ball-and-plate equations of motion. Builds the Lagrangian from `Tb`, `Tp` and `V` and applies the Euler-Lagrange operator to all four generalized coordinates. |
| `System.slx` | Plant model used in the simulation loops. |
| `Observer.slx` | Standalone Luenberger observer model. |
| `Simulink.m` | Numeric constants for the symbolic model (`mb`, `g`, `rb`, `Ib`, `Ip`). The last line contains a typo (`Ip=13;s`). |
| `Kinematics.m` | Solves the Lyapunov equation `PB + BᵀP = −T` symbolically and checks controllability of the double integrator. The filename does not describe the contents. |
| `Others.m` | Root-locus lead-compensator design for `10/s²`. Exploratory; not used in the final controllers. |

## `src/kinematics/`

| File | Description |
|---|---|
| `inv_ken.m` | Closed-form inverse geometric model of the 6-RSS platform. Takes `(px, py, pz, α, β, γ)` and returns six motor angles in degrees. Compiled into every `rpi/` model. |
| `inv_ken2.m` | Earlier variant. |
| `inv_ken_test.slx` | Simulation harness. |

## `src/hardware/`

| File | Description |
|---|---|
| `Angle2Duty.m` | Six per-servo angle-to-pulse-width calibration tables with linear fits. |
| `Motors.m` | Connects to the Pi, configures the six PWM pins at 50 Hz, moves the platform to the home pose `(0,0,23,0,0,0)`. |
| `rst2.m` | Per-servo neutral pulse widths. |
| `Reset123.m` | Sets all six duty cycles to zero. |
| `stuff2.m` | Earlier calibration set at 10° increments, superseded by `Angle2Duty.m`. |
| `blink_LED.slx` | Toolchain check. |
| `pwm.slx` | Single-channel PWM test. |
| `Serial.slx` | Touchscreen read and decode. |
| `Serial_1.slx` | Serial read variant. |
| `stuff.slx` | Serial read with sine-driven PWM output. |
| `test.slx` | Constant pose through the inverse kinematics to all six PWM channels, open loop. |

## `src/identification/`

| File | Description |
|---|---|
| `Identification.slx` | Identification experiment: drives the plate with known tilt profiles and logs the ball response. |
| `xidennew.mat`, `yidennew.mat` | Final identification datasets, giving `K1 = 6.1486` and `K2 = 6.664`. |
| `xident.mat`, `yiden.mat` | Earlier identification runs. |
| `xpractice.mat`, `ypractice.mat` | Validation data. |
| `t.mat`, `y.mat` | Time vector and output from an early run. |

---

## `src/control/`

Nine families, each with a `sim/` folder (plant model in the loop) and, where deployed, an
`rpi/` folder (real sensor and servos).

### `pd/`

| File | Description |
|---|---|
| `sim/PD.m` | `ζ = 0.9`, `ωₙ = 2`; `Kd = 2ζωₙ/K`, `Kp = ωₙ²/K`. |
| `sim/PD_discrete.slx` | Discrete-time PD simulation. |
| `sim/PD_statespace_discrete.m` | State-feedback form, discretized, per-axis gains. |
| `rpi/PD.slx` | Deployed model: serial read, decode, deadband, observer, state feedback, saturation, inverse kinematics, six PWM outputs. |
| `rpi/PD_2016.slx` | R2016b export. |
| `rpi/PD_statespace.m` | Parameters: `tr = 3 s`, `ζ = 1.1`, observer poles at 0.2× the controller poles. |
| `rpi/PD_practice.slx`, `PD_statespace_practice.slx`, `PD_statespace_practice_d.slx` | Earlier deployed iterations. |

### `pid/`

| File | Description |
|---|---|
| `sim/PID.m` | Gains obtained by matching the closed-loop characteristic polynomial to a desired one with a third pole at −100, solved symbolically. |
| `sim/PID.slx`, `rpi/PID_practice.slx` | Simulation and deployed models. |

### `pole_placement/`

| File | Description |
|---|---|
| `sim/Discrete_pole_placement.m` | Discretizes the plant, places poles with `place`, designs an observer via the Ackermann form. |
| `sim/Pole_Placement_Integrator.m` | Integral action on the augmented plant `[A 0; −C 0]`. |
| `sim/Pole_Placement_PID.m` | Pole placement used to derive PID-equivalent gains, per axis. |
| `sim/Pole_Placement*.slx` | Continuous, discrete, integrator and test models. |
| `rpi/Pole_Placement_practice.slx` | Deployed model. |

### `lqr/`

| File | Description |
|---|---|
| `sim/LQR.m` | `Q = diag(4,1)`, `R = 2`, discretized at `Ts = 0.01`; `dlqr` gain with observer poles at −10. |
| `rpi/LQR.m` | Hardware retune. |
| `sim/LQR.slx`, `rpi/LQR_practice.slx` | Models. |

### `sliding_mode/`

| File | Description |
|---|---|
| `sim/Sliding_mode.m` | `λ = 2`, `η = 0.5`, switching gain `K = 1.1η/K`. |
| `rpi/Sliding_mode.m` | Per-axis gains, `λ = 2`, `η = 1.5`. |
| `rpi/slidingmode_practice.slx`, `_2016.slx` | Deployed model and its R2016b export. |

### `super_twisting/`

| File | Description |
|---|---|
| `rpi/Super_twisting.m` | `λx = 1.2`, `λy = 1.3`, `L = 6`, `W = 0.5√L`, `γ = 4L`. |
| `sim/Super_twisting.slx` | Simulation. |
| `rpi/Super_Twisting_test.slx`, `supertwisting2.slx` | Deployed models. |

### `adaptive/`: simulation only

| File | Description |
|---|---|
| `sim/Adaptive.m` | MRAC. Reference model `ζm = 1.1`, `ωm = 2`; adaptation gains `Γx = diag(35,10)`, `Γr = 1`; solves `PAm + AmᵀP = −Q` symbolically for the update law. |
| `sim/Adaptive.slx`, `Discrete_Adaptive_Saturation.slx` | Continuous and discrete-with-saturation models. |

### `fuzzy/`

| File | Description |
|---|---|
| `sim/fuzzytest.m`, `fuzzytest.slx` | Fuzzy controller simulation. |
| `sim/fuzzysliding.m`, `.slx`, `.fis` | Fuzzy-sliding hybrid; the FIS tunes the sliding-mode gain. |
| `rpi/fuzzy_practical.slx` | Deployed fuzzy controller. |
| `rpi/fuzzy_sliding.slx` | Deployed fuzzy-sliding controller. |
| `rpi/test2.fis`, `test3.fis`, `test4.fis` | Successive FIS iterations. |

### `mpc/`: simulation only

| File | Description |
|---|---|
| `sim/Fast_MPC.m` | Condensed MPC. Builds the prediction matrices over `N = 50`, `Nu = 5` for the 4-state 2-input plant and reduces them to the static gains `K`, `Ke`, `G1`, `G2`, so the online computation is a matrix multiplication. |
| `sim/FAst_MPC.slx` | Simulation model. |
| `sim/func1.slx` | Supporting function model. |

---

## `experiments/`

Logs from the physical rig. Naming: `x…`/`y…` for the two axes, `…step` for step response,
`…circle` for circular tracking, `…res` for a general run, trailing `2` for a repeat run.

| Folder | Contents |
|---|---|
| `pd/` | `xstep`, `ystep`, `xpd`, `ypd`, `matlab.mat` (holds `d_x`, `d_y`) |
| `sliding_mode/` | `xstep`/`ystep`, `xres`/`yres`, `xres2`/`yres2`, `ystep2`, and `alpha.mat`/`beta.mat` holding the commanded plate tilt angles |
| `super_twisting/` | `xstep`/`ystep`, `xstep2`/`ystep2`, `xcircle`/`ycircle` |
| `scripts/pdplot.m` | Original plotting script; reconstructs the staircase reference of the PD step test (15 → 20 → 10 cm, switching at samples 752 and 1674, `Ts = 0.02`) |
| `scripts/load_results.m` | Lists each `.mat` file and the variables it contains |

Variable names inside the `.mat` files are short and undocumented; `pdplot.m` expects `x1`
and `y1`. Use `load_results` or `whos -file <name>.mat` to inspect a file before loading it.

---

## `docs/figures/`

| File | Description |
|---|---|
| `fig1-kinematic-chain.png` | Kinematic chain of the mechanism: bodies `C0` to `C13`, joints `L1` to `L18` |
| `fig2-leg-kinematic-diagram.png` | Kinematic diagram of one leg showing `l₁`, `l₂` and the joint axes |
| `fig3-denavit-hartenberg-frames.png` | DH frame assignment along one leg |
| `fig4-base-vertices-and-leg-frames.png` | Base vertices `A1` to `A6` with their local frames |
| `fig5-platform-hexagon-dimensions.png` | Base and platform hexagon dimensions `b`, `d`, `b'`, `d'` |
| `fig6-inverse-kinematics-geometry.png` | Geometry of the inverse solution: `Aᵢ`, `Bᵢ`, `Cᵢ`, `θᵢ₁` |

---

## Build output

Generated C, build directories, Simulink caches and compiled `.elf` binaries are not
tracked; they are produced from the models by the build command. See `.gitignore`.
