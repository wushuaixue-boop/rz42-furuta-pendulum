# From Kane's method to LQR: how the RZ42 model and controller fit together

This note explains, step by step, what this project does with the dynamics. It covers how the equations of motion are derived with **Kane's method**, how they become a **control-theory problem** in state-space form, how that connects to **parameter identification**, and how the **LQR** and swing-up controllers are built from it.

The pipeline:

```
 CAD + physics           Kane's method              control form                    controller
 ─────────────   ──►   ─────────────────   ──►   ─────────────────────   ──►   ─────────────────────
 frames, points,        F_r + F_r* = 0            x' = f(x, u)                  LQR about upright
 masses, inertias       M(q) q'' + C q' +         linearise: x' = A x + B u      + energy swing-up
                        G(q) + F(q') = tau        input u = q1_cmd (stepper)     + mode switching
                              │                              ▲
                              └──► identification: ratios κ, ρ, ω², β, φ from encoder data
```

Where each step lives in the repository:

| Step | File |
|---|---|
| Kane derivation | `identification/01_kane_model.ipynb`, sections 1–9 |
| Simulation, linearisation, position-input model | `01_kane_model.ipynb`, sections 10–13 and the `A_pos` cell |
| Identification | `identification/02_parameter_identification.ipynb` |
| Controller | `control/` (`models.py`, `lqr.py`, `swingup.py`, `hybrid.py`) |
| Results | `docs/CONTROL_REPORT.md`, `results/` |

---

## 1. The system and its coordinates

A Furuta pendulum has two rigid bodies:
- **Arm (body A):** driven by the stepper. It rotates about the vertical motor axis by the angle **q1**.
- **Pendulum (body B):** hangs from the end of the arm on a 608ZZ bearing. It rotates about the arm's own axis by **q2**. **q2 = 0 is hanging and q2 = π is upright.**

| Symbol | Meaning |
|---|---|
| N | the fixed world frame, with N.z up |
| A | the arm frame: `A.orient_axis(N, q1, N.z)` |
| B | the pendulum frame: `B.orient_axis(A, q2, A.x)` |
| O | a point on the motor axis (fixed) |
| Ao | arm centre of mass, at x_a along A.x |
| P | pendulum pivot (608 centre), at r·A.x + h·A.z, with r = 62 mm |
| Bo | pendulum centre of mass, at c_p·B.x − l_p·B.z from P |
| m_a, J_a | arm mass and inertia about the vertical axis (including the rotor) |
| m_p, I_px, I_py, I_pz, I_pxz | pendulum mass and inertia about its centre of mass, in B |
| b_a, F_a, b_p, F_p | viscous and Coulomb friction of the arm joint and the pendulum bearing |
| T | motor torque on the arm |

The configuration is fully described by **q = [q1, q2]**: two degrees of freedom. We also define generalized speeds **u1 = q1', u2 = q2'**, so the kinematic equations are simply q' = u.

---

## 2. Kane's method: how it works

Newton–Euler methods write F = ma for each body. That brings in every internal reaction force, for example the bearing forces between the arm and the pendulum, and these then have to be eliminated. Lagrange's method avoids them, but needs energies and their derivatives.

**Kane's method** projects Newton's law onto the directions in which the system can actually move. Forces that do no work in those directions, such as bearing reactions, disappear automatically.

### Step 1: velocities and partial velocities

Compute the velocity of each centre of mass and the angular velocity of each body, in terms of the generalized speeds:

$$\mathbf v_{Ao} = x_a u_1\,\mathbf A_y, \qquad \boldsymbol\omega_A = u_1\,\mathbf N_z, \qquad \boldsymbol\omega_B = u_1\,\mathbf N_z + u_2\,\mathbf A_x, \qquad \mathbf v_{Bo} = \dots$$

The **partial velocity** with respect to $u_r$ is the coefficient of $u_r$ in each velocity:

$$\mathbf v^{(r)}_{P} = \frac{\partial \mathbf v_P}{\partial u_r}, \qquad \boldsymbol\omega^{(r)}_{K} = \frac{\partial \boldsymbol\omega_K}{\partial u_r}$$

It is "the direction in which that point or body moves when only $u_r$ is non-zero". In the notebook this is `me.partial_velocity(vels, U, N)`.

Two examples:
- $\boldsymbol\omega^{(2)}_B = \mathbf A_x$: increasing u2 turns the pendulum about the hinge, and nothing else.
- $\boldsymbol\omega^{(2)}_A = 0$: the arm does not turn when only the pendulum moves.

### Step 2: generalized active forces $F_r$

Project every applied force and torque onto the partial velocities:

$$F_r = \sum_{\text{points}} \mathbf v^{(r)}_P \cdot \mathbf R_P \;+\; \sum_{\text{bodies}} \boldsymbol\omega^{(r)}_K \cdot \mathbf T_K$$

The loads are:
- gravity on both centres of mass: $-m g\,\mathbf N_z$;
- the motor torque on the arm, $T\,\mathbf A_z$;
- arm friction $-(b_a u_1 + F_a\tanh(u_1/v_s))\,\mathbf A_z$;
- bearing friction $\tau_p = b_p u_2 + F_p\tanh(u_2/v_s)$, applied as $-\tau_p\mathbf A_x$ on the pendulum and $+\tau_p\mathbf A_x$ on the arm (action and reaction).

The result is short:

$$F_1 = T - b_a u_1 - F_a\tanh\tfrac{u_1}{v_s}, \qquad F_2 = -g\,m_p l_p\sin q_2 - b_p u_2 - F_p\tanh\tfrac{u_2}{v_s}$$

Two things stand out:
- **Gravity does nothing in row 1.** The arm moves horizontally, so it does no work against gravity. This is why the arm's centre of mass cannot be identified from dynamics.
- **The bearing friction appears only in row 2.** The arm's half of the action/reaction pair has zero partial velocity there.

### Step 3: generalized inertia forces $F_r^*$

Project the "inertia forces" the same way:

$$F_r^* = \sum_{\text{points}} \mathbf v^{(r)}_P \cdot (-m\,\mathbf a_P) \;+\; \sum_{\text{bodies}} \boldsymbol\omega^{(r)}_K \cdot \big(-\boldsymbol\alpha_K\cdot\mathbf I_K - \boldsymbol\omega_K\times\mathbf I_K\cdot\boldsymbol\omega_K\big)$$

Here $\mathbf a$ is the acceleration of the centre of mass, $\boldsymbol\alpha$ the angular acceleration, and $\mathbf I$ the inertia dyadic. The cross-product term produces the centrifugal and Coriolis effects.

### Step 4: Kane's equations

$$F_r + F_r^* = 0, \qquad r = 1, 2$$

These are two equations, one per degree of freedom, with no reaction forces anywhere. They are linear in the accelerations $\dot u$, so the notebook splits them as

$$M_d\,\dot u + g_d = 0, \qquad M_d = \frac{\partial (F_r+F_r^*)}{\partial \dot u}, \quad g_d = (F_r+F_r^*)\big|_{\dot u=0}$$

The notebook checks the hand-built result against SymPy's own `KanesMethod`. The difference is exactly zero.

---

## 3. The result: standard robot-dynamics form

Rearranged, Kane's equations become the familiar form

$$M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) + F(\dot q) = \tau, \qquad \tau = \begin{bmatrix}T\\0\end{bmatrix}$$

$$M=\begin{bmatrix} J_0 + J_1\sin^2 q_2 & K\cos q_2\\ K\cos q_2 & J_2\end{bmatrix},\quad
C\dot q=\begin{bmatrix}2J_1 \sin q_2\cos q_2\,\dot q_1\dot q_2 - K\sin q_2\,\dot q_2^2\\ -J_1\sin q_2\cos q_2\,\dot q_1^2\end{bmatrix},\quad
G=\begin{bmatrix}0\\ G\sin q_2\end{bmatrix}$$

The 14 physical unknowns only ever appear in **5 inertial combinations** plus 4 friction terms:

$$J_0 = J_a + m_a x_a^2 + m_p(r+c_p)^2 + I_{pz},\quad J_1 = I_{py}-I_{pz}+m_p l_p^2,\quad J_2 = I_{px}+m_p l_p^2,\quad K = I_{pxz}+m_p l_p(r+c_p),\quad G = g\,m_p l_p$$

What each term means physically:

| Term | Meaning |
|---|---|
| $J_0 + J_1\sin^2 q_2$ | inertia the motor feels. It grows when the pendulum swings outward, because more mass is then far from the motor axis. |
| $K\cos q_2$ | **coupling**: arm acceleration pushes the pendulum, and the pendulum pushes back on the arm. |
| $J_2$ | pendulum inertia about its hinge |
| $G\sin q_2$ | gravity torque on the pendulum |
| $J_1\sin q_2\cos q_2\,\dot q_1^2$ | centrifugal effect: a spinning arm flings the pendulum outward |

---

## 4. Turning it into a control problem

### 4.1 State space

Control theory wants first-order equations $\dot x = f(x, u)$. With the state

$$x = [q_1,\ q_2,\ \dot q_1,\ \dot q_2]^T$$

the kinematic equations give the first two rows, and solving $M\ddot q = \tau - C\dot q - G - F$ gives the last two. In the notebook: `M_ss = diag(Mk, Md)`, `g_ss = [gk; gd]`, `ssd_sol = -M_ss.LUsolve(g_ss)`.

### 4.2 Equilibria and linearisation

The pendulum is at rest when $G\sin q_2 = 0$:
- **hanging**, $q_2 = 0$: stable;
- **upright**, $q_2 = \pi$: unstable.

Close to an equilibrium $x_{eq}$, expand f to first order:

$$\dot{\delta x} \approx A\,\delta x + B\,\delta u, \qquad A = \frac{\partial f}{\partial x}\Big|_{eq},\quad B = \frac{\partial f}{\partial u}\Big|_{eq}$$

All velocity-squared terms (the C matrix) vanish here, because velocities are zero at equilibrium. Two quick checks of the linear model:
- **Hanging:** the eigenvalues give the natural frequency, about 1.94 Hz with the arm held.
- **Upright:** one eigenvalue is real and positive, $+\sqrt{G/J_2} \approx +12\ \text{s}^{-1}$. This is the pendulum falling over, which a controller must stabilise.

### 4.3 Choosing the input: torque, acceleration, or position command

The textbook Furuta pendulum uses **motor torque** as the input ($u = T$). Our rig has a **closed-loop stepper in position mode**: we send a target angle and the driver makes whatever torque is needed. So the physically correct input is

$$u = q_{1,cmd}$$

and the arm follows it through the driver's own loop. We model that loop as a second-order servo:

$$\ddot q_1 = \omega_s^2\,(q_{1,cmd} - q_1) - 2\zeta_s\omega_s\,\dot q_1 \qquad (\omega_s = 2\pi\cdot 15\ \text{rad/s},\ \zeta_s = 0.8\ \text{— placeholders})$$

**Only the pendulum row is needed.** Since the arm acceleration is now imposed by the servo, row 1 (with the unknown J0 and the motor torque) drops out. Row 2, divided by $J_2$, is all that is left:

$$\ddot q_2 = -\kappa\cos q_2\,\ddot q_1 + \rho\sin q_2\cos q_2\,\dot q_1^2 - \omega^2\sin q_2 - \beta\dot q_2 - \phi\tanh(\dot q_2/v_s)$$

$$\kappa = \tfrac{K}{J_2},\quad \rho = \tfrac{J_1}{J_2},\quad \omega^2 = \tfrac{G}{J_2},\quad \beta = \tfrac{b_p}{J_2},\quad \phi = \tfrac{F_p}{J_2}$$

This is the key link between modelling, identification and control:
- these **five ratios** are exactly what `02_parameter_identification.ipynb` estimates from encoder data only, with no torque sensor;
- they are also exactly what the controller needs.

Identified values:

| κ | ρ | ω² | β | φ |
|---|---|---|---|---|
| 0.945 | 0.965 | 148.3 s⁻² | 0.47 s⁻¹ | 0.95 s⁻² |

### 4.4 The linear model the LQR uses

Linearise about upright, writing δ = q2 − π. Since cos π = −1, the coupling term flips sign: $-\kappa\cos q_2\,\ddot q_1 \to +\kappa\ddot q_1$. Substituting the servo model for $\ddot q_1$:

$$\dot x = A_{pos}\,x + B_{pos}\,q_{1,cmd},\qquad
A_{pos}=\begin{bmatrix}0&0&1&0\\0&0&0&1\\-\omega_s^2&0&-2\zeta_s\omega_s&0\\-\kappa\omega_s^2&\omega^2&-2\kappa\zeta_s\omega_s&-\beta\end{bmatrix},\quad
B_{pos}=\begin{bmatrix}0\\0\\\omega_s^2\\\kappa\omega_s^2\end{bmatrix}$$

- Rows 3 and 4 say that the command pulls the arm (row 3), and the arm's acceleration pushes the pendulum with gain κ (row 4).
- **Eigenvalues:** −75 ± 57j (the servo), −12.4, and **+11.9** (the pendulum falling).
- The controllability matrix $[B, AB, A^2B, A^3B]$ has full rank 4, so this single input can steer all four states.

The upright transfer function shows why a *position* command can balance a pendulum at all, even though it only acts through acceleration:

$$\frac{\delta(s)}{q_1(s)} = \frac{\kappa s^2}{s^2 + \beta s - \omega^2} \qquad (\text{ideal servo})$$

The s² in the numerator means a constant arm angle does nothing; only arm acceleration moves the pendulum.

---

## 5. LQR: optimal state feedback around upright

LQR picks the linear feedback $u = -Kx$ that minimises

$$J = \sum_{k=0}^{\infty} \big(x_k^T Q\, x_k + u_k^T R\, u_k\big)$$

- **Q** says how much each state error costs: pendulum angle, arm angle, and their velocities.
- **R** says how much command effort costs.

The answer comes from the discrete algebraic Riccati equation:

$$P = A_d^T P A_d - A_d^T P B_d (R + B_d^T P B_d)^{-1} B_d^T P A_d + Q, \qquad K = (R + B_d^T P B_d)^{-1} B_d^T P A_d$$

How this project applies it (`control/lqr.py`):

1. **Discretise.** The controller runs at 200 Hz, so $A_{pos}, B_{pos}$ are converted with a zero-order hold: $(A_d, B_d) = \text{expm}$ of the block matrix, times $T_s$.
2. **Choose the weights.** Q = diag(100, 200, 1, 2) for [q1, δ, q1', q2'], and R = 10. These were tuned **in simulation**, on the hidden plant, with taps on the pendulum and a deliberately slower servo (8 Hz instead of 15). The weights that survive that mismatch with the smallest arm excursion were kept.
3. **Gains:** K = [−1.195, 1.182, −0.076, 0.096].
   - The **negative** arm gain is correct and typical of the Furuta pendulum. To catch a pendulum falling one way, the arm first has to move *toward* the fall.
   - Closed-loop poles: s ≈ −8.7 ± 4.5j, −10.0, −1257, all stable.
4. **Control law with angle wrapping:**
   $$q_{1,cmd} = q_{1,eq} - K\,[\,q_1 - q_{1,eq},\ \text{wrap}(q_2-\pi),\ \dot q_1,\ \dot q_2\,]^T$$
   - `wrap()` maps the pendulum error to [−π, π), so any multiple of 2π counts as upright.
   - $q_{1,eq}$ starts at the arm angle at the moment of the catch and walks back to 0 slowly. This avoids a jump in the command.
5. **Velocities.** The controller only measures angles, so q1' and q2' are estimated by filtered finite differences (12 Hz cutoff). That cutoff was a trade-off between command chatter and lag.

**What LQR cannot do.** The linear model is only valid near upright. Starting from hanging, the pendulum must be brought up first, which is a nonlinear problem.

---

## 6. Swing-up: back to the nonlinear model, through energy

Far from upright, we use the pendulum's **energy**, which is exact at any angle. Per unit $J_2$, measured from hanging:

$$E = \tfrac12\dot q_2^2 + \omega^2(1 - \cos q_2), \qquad E_{up} = 2\omega^2$$

Differentiating along the nonlinear pendulum equation of section 4.3:

$$\dot E = -\kappa\,\dot q_2\cos q_2\;\ddot q_1 \;-\; (\text{friction terms})$$

So the arm adds energy whenever its acceleration has the opposite sign to $\dot q_2\cos q_2$. The swing-up law (`control/swingup.py`) commands

$$q_{1,cmd} = \text{sat}\Big(c\,\max\big(0,\ 1 - E/E_{up}\big)\ \overline{\dot q_2\cos q_2},\ A_{max}\Big)$$

How the pieces work:
- The arm follows a signal that oscillates at the swing frequency w, so $\ddot q_1 \approx -w^2 q_1$, and the energy rate becomes $\approx +\kappa c w^2 (1-E/E_{up})(\dot q_2\cos q_2)^2 \ge 0$.
- The arm amplitude is bounded by $A_{max}$ = 0.6 rad.
- The overbar is a 5 Hz low-pass filter, which stops a fast feedback loop through the coupling term.
- Pumping stops once E reaches $E_{up}$.

**Switching to LQR** (`control/hybrid.py`) happens when the pendulum is within 25° of upright and slower than 8 rad/s. If it falls back beyond 45°, or the arm gets close to its travel limit, the controller returns to swing-up. The gap between 25° and 45° is the hysteresis that prevents chattering between modes.

---

## 7. How the pieces depend on each other

| Needed for | Parameters | Source |
|---|---|---|
| balancing (LQR) | κ, ω², β, and the servo ω_s, ζ_s | identification (κ, ω², β); servo values are **placeholders until measured** |
| swing-up | ω² (sets $E_{up}$), sign of κ | identification |
| large-angle accuracy | ρ, φ | identification |
| motor torque margin only | J0, arm friction | not identifiable without a torque sensor; CAD estimate |
| optional: SI values, centre of mass | J2, G, l_p | added-mass experiment, pre-assembly weighing |

**Assumptions** (also listed in `docs/CONTROL_REPORT.md`):
- rigid bodies and a stiff shaft coupling;
- Coulomb friction smoothed by tanh;
- a second-order servo model of the ZDT driver, with placeholder bandwidth;
- no microstep ripple or missed steps.

All results so far come from the hidden MuJoCo plant, not from the real rig.
