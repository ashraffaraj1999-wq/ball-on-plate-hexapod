# Dynamic Model of the Ball-and-Plate System

Derivation by the Euler-Lagrange method. Symbolic source:
[`src/model/Euler_Lagrange_Modelling.m`](../src/model/Euler_Lagrange_Modelling.m).

---

## 1. Assumptions

The system contains many interacting parameters, so the model rests on five
simplifications:

1. The ball is homogeneous.
2. The ball rolls on the plate without slipping.
3. Friction between ball and plate is negligible.
4. The ball remains in continuous contact with the plate.
5. The rotation axes of the moving platform are independent of each other.

---

## 2. Generalized coordinates

| Symbol | Meaning |
|---|---|
| `xb`, `yb` | ball position on the plate |
| `α` | plate rotation about the `y` axis |
| `β` | plate rotation about the `x` axis |
| `mb`, `rb`, `Ib` | ball mass, radius, moment of inertia about its centre |
| `Ip` | plate moment of inertia |

`q = [xb, yb, α, β]ᵀ`.

---

## 3. Energies

**Ball.** Translational kinetic energy of the centre of mass plus rotational kinetic
energy about its own centre:

```
Tb = ½·mb·(ẋb² + ẏb²) + ½·Ib·(ωx² + ωy²)
```

Rolling without slipping gives `ẋb = rb·ωx` and `ẏb = rb·ωy`, so

```
Tb = ½·(mb + Ib/rb²)·(ẋb² + ẏb²)
```

**Plate.** Rotational kinetic energy of the plate about the coordinate axes, plus that of
the ball treated as a point mass at `(xb, yb)`:

```
Tp = ½·(Ip + Ib)·(α̇² + β̇²) + ½·mb·(xb·α̇ + yb·β̇)²
```

**Potential energy**, from the height the tilt gives the ball:

```
V = mb·g·(xb·sin α + yb·sin β)
```

**Lagrangian:**

```
L = Tb + Tp − V
```

---

## 4. Equations of motion

Applying `d/dt(∂L/∂q̇ᵢ) − ∂L/∂qᵢ = Qᵢ` to the four coordinates gives

```
(mb + Ib/rb²)·ẍb − mb·(xb·α̇ + yb·β̇)·α̇ + mb·g·sin α = 0
(mb + Ib/rb²)·ÿb − mb·(xb·α̇ + yb·β̇)·β̇ + mb·g·sin β = 0
(Ip + Ib + mb·xb²)·α̈ + 2·mb·xb·ẋb·α̇ + mb·xb·yb·β̈ + mb·g·xb·cos α = τx
(Ip + Ib + mb·yb²)·β̈ + 2·mb·yb·ẏb·β̇ + mb·xb·yb·α̈ + mb·g·yb·cos β = τy
```

which collect into

```
M(q)·q̈ + C(q,q̇)·q̇ + G(q) = τ
```

**Inertia matrix**

```
        ⎡ mb + Ib/rb²        0             0             0        ⎤
M(q) =  ⎢      0      Ib + Ip + mb·xb²     0        mb·xb·yb      ⎥
        ⎢      0             0      mb + Ib/rb²          0        ⎥
        ⎣      0        mb·xb·yb           0    Ip + Ib + mb·yb²  ⎦
```

**Gravity vector**

```
G(q) = [ mb·g·sin α ,  mb·g·xb·cos α ,  mb·g·sin β ,  mb·g·yb·cos β ]ᵀ
```

**Input**

```
τ = [ 0 , τx , 0 , τy ]ᵀ
```

The Coriolis matrix `C(q,q̇)` is given in full in the symbolic script.

---

## 5. Linear approximation

Near an equilibrium of the platform the angles are small and the angular rates negligible,
so:

```
cos α ≈ cos β ≈ 1        sin α ≈ α        sin β ≈ β        α̇ ≈ 0        β̇ ≈ 0
```

Substituting, with `Ib = ⅖·mb·rb²` for a solid sphere:

```
ẍb = − mb/(mb + Ib/rb²) · g · α  =  −(5g/7)·α
ÿb = −(5g/7)·β
α̈  = −mb·g/(Ip + Ib)·xb + τx/(Ip + Ib)
β̈  = −mb·g/(Ip + Ib)·yb + τy/(Ip + Ib)
```

The ball dynamics decouple into two identical double integrators. Per axis, with state
`[position, velocity]ᵀ`:

```
A = ⎡0  1⎤      B = ⎡0⎤      C = [1  0]      D = 0
    ⎣0  0⎦          ⎣K⎦
```

with `K = −5g/7`. The pair is controllable and observable, `ctrb` and `obsv` both have
rank 2.

### Applicability of the torque input

The equations above assume a freely moving platform acted on by two external torques, with
the weight of the ball also influencing the platform angles. That is not the case here: the
upper platform is mounted on the hexapod, external torques cannot be applied about the `x`
and `y` axes, and the motion of the ball does not affect the platform, since the only means
of changing the platform's pose is by moving the six lower motor angles.

Accordingly the controller output is the commanded tilt `(α, β)`, treated as a kinematic
input and realized through the inverse geometric model.

---

## 6. Identification

`src/identification/Identification.slx` drives the plate with known tilt profiles and logs
the ball response:

| Axis | Identified `K` | Theoretical `5g/7` | Difference |
|---|---|---|---|
| x | 6.1486 | 7.007 | −12.3 % |
| y | 6.664 | 7.007 | −4.9 % |

Both identified gains are below the theoretical value, consistent with rolling resistance
and the compliance of the touchscreen surface, which assumption 3 neglects. The two axes
differ from each other by approximately 8 %, which assumption 5 does not predict; the
hexapod geometry is not symmetric between the two tilt directions and the touchscreen
active area is not square (22.8 × 30.4 cm).

The hardware controllers therefore use separate gains per axis, `K1` for x and `K2` for y.

---

## 7. Files

| File | Purpose |
|---|---|
| [`src/model/Euler_Lagrange_Modelling.m`](../src/model/Euler_Lagrange_Modelling.m) | Symbolic derivation of the equations of motion |
| [`src/model/System.slx`](../src/model/System.slx) | Plant model used in the simulation loops |
| [`src/model/Observer.slx`](../src/model/Observer.slx) | Luenberger observer development model |
| [`src/model/Simulink.m`](../src/model/Simulink.m) | Numeric constants for the symbolic model |
| [`src/identification/`](../src/identification/) | Identification model and logged data |
