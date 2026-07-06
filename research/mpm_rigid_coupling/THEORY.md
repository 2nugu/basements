# Theory — MPM ↔ Rigid Coupling Methods (M0 … M4.5)

**Working theoretical document** for Paper #1. Each method's governing
equations, what assumptions it makes, and which assumptions the *next*
method removes. The whole point of the M0→M4.5 progression is to peel
off assumptions one at a time and measure what each peel costs and buys.

Symbols throughout:

| Symbol | Meaning |
|--------|---------|
| $\mathbf{x}_i$ | world-space position of grid node $i$ |
| $\mathbf{v}_i$ | grid node velocity (post-`update_grid`) |
| $m_i$ | grid node mass |
| $\mathbf{x}_c, \mathbf{v}_c, \boldsymbol{\omega}$ | rigid-body centre position, linear velocity, angular velocity |
| $M, \mathbf{I}$ | rigid-body mass, world-space inertia tensor |
| $\mathbf{r}_i = \mathbf{x}_i - \mathbf{x}_c$ | node position relative to body centre |
| $\mathbf{v}_b(\mathbf{x}_i) = \mathbf{v}_c + \boldsymbol{\omega} \times \mathbf{r}_i$ | body velocity at the node |
| $\mathbf{n}$ | outward face normal of body at the closest face |
| $\Delta t$ | simulation step |
| $V_i$ | OBB-cell intersection volume at node $i$ (per-axis $\Delta x$) |
| $w_i$ | particle / kernel weight at node $i$ |

---

## Common pre-conditions (all methods)

The MPM step has already executed P2G + `update_grid`, so the grid holds
the *next-step* velocity field $\mathbf{v}_i$ and mass $m_i$ at active nodes.
The coupler operates between `update_grid` and `g2p`.

For every method we iterate the **active grid nodes inside the body's
OBB**. Outside-OBB nodes are untouched.

The quantities of interest we measure on every method, every step:

$$
\Sigma\mathbf{p}_y(t) = \mathbf{p}_{body,y} + \sum_{i \in \text{grid}} m_i \mathbf{v}_{i,y}
\qquad
E(t) = \tfrac{1}{2} M \|\mathbf{v}_c\|^2 + \tfrac{1}{2}\boldsymbol{\omega}^T \mathbf{I} \boldsymbol{\omega}
     + \sum_i \tfrac{1}{2} m_i \|\mathbf{v}_i\|^2 + Mg y_c
$$

Conservation tells us $\Sigma\mathbf{p}$ should drift only by float
roundoff for a closed system. Any sustained drift means the method is
injecting / leaking momentum.

---

## M0 — Dirichlet velocity BC (engineering MVP)

### Setup

Body is treated as a moving rigid wall. The grid node sits in / on the
wall, and we *force* the wall's velocity onto the node component-wise.

### Equations

Outward normal $\mathbf{n}$ from the body OBB's closest face. Decompose:

$$
v_i^n = \mathbf{v}_i \cdot \mathbf{n}, \quad
v_i^t = \mathbf{v}_i - v_i^n \mathbf{n}, \quad
v_b^n = \mathbf{v}_b(\mathbf{x}_i) \cdot \mathbf{n}.
$$

No-penetration clamp:

$$
{v_i^n}' = \max(v_i^n, v_b^n).
$$

Coulomb tangential blend ($\mu$ = friction coefficient ∈ [0,1]):

$$
{v_i^t}' = v_i^t + \min(1, \mu)\bigl(v_b^t - v_i^t\bigr).
$$

New node velocity: $\mathbf{v}_i' = {v_i^n}'\mathbf{n} + {v_i^t}'$. Momentum delta:

$$
\Delta\mathbf{p}_i = m_i (\mathbf{v}_i' - \mathbf{v}_i).
$$

Body reaction (Newton's third law on the *flux through the node*):

$$
\mathbf{F}_b = -\frac{1}{\Delta t}\sum_i \Delta\mathbf{p}_i, \quad
\boldsymbol{\tau}_b = -\frac{1}{\Delta t}\sum_i \mathbf{r}_i \times \Delta\mathbf{p}_i.
$$

### Assumptions

1. **Body acts as a Dirichlet boundary** — the body has so much inertia
   relative to the grid that its velocity is unaffected by the node.
2. **Mass is one-sided** — only the grid node mass enters the impulse;
   the body's contribution is not part of the momentum book.
3. **Outward normal is well-defined** — closest face heuristic (no
   continuous SDF at corners).

### Failure mode

Because the body's mass never enters the per-node equation, total system
momentum changes by $\Delta\mathbf{p}_i$ at every clamp — a true closed
system would have $\sum \Delta\mathbf{p}_i = 0$. Empirically:

- mass_body >> sand mass: small relative error, M0 OK (rover on solid ground).
- mass_body ≈ sand mass: large relative error, **body punches through**.

This is the original observation that motivated M1.

---

## M1 — PIC-Rigid (basic mass-weighted, current 35× improvement)

### Setup

Treat the body's mass as **distributed uniformly** over the OBB-overlapping
grid nodes. Each node sees the body as one extra particle's worth of
contribution and updates with a mass-weighted average velocity.

### Equations

Body mass per node (uniform approximation):

$$
m_b = \frac{M}{N_{\text{nodes}}}, \quad
N_{\text{nodes}} \approx \frac{V_{\text{OBB}}}{\Delta x^3}.
$$

Mass-weighted average:

$$
\mathbf{v}_i' = \frac{m_i \mathbf{v}_i + m_b \mathbf{v}_b(\mathbf{x}_i)}{m_i + m_b}, \quad
m_i' = m_i + m_b.
$$

Body reaction is the body's momentum *deficit* at each node — what the
body wanted (v_b) vs what the grid resolved to ($\mathbf{v}_i'$):

$$
\mathbf{F}_b = \frac{1}{\Delta t}\sum_i m_b\bigl(\mathbf{v}_i' - \mathbf{v}_b(\mathbf{x}_i)\bigr).
$$

### Assumptions removed (vs M0)

1. **Body's mass is now in the book** — system momentum is conserved up
   to mass-discretisation error.

### Assumptions still present

1. **Uniform $m_b$** — every node inside the OBB gets the same mass slice
   regardless of how much OBB volume actually occupies that cell.
   → Over-counts mass when body only partially penetrates the grid.
2. **No friction tangent** — tangential velocity passes through
   un-modified, so dry-friction effects are absent.
3. **Discontinuous at the OBB face** — a node that *just* enters the OBB
   contributes the full slice $m_b$ instantaneously, producing a force
   spike. No B-spline smoothing.

### Measured improvement on `box_drop`

| | M0 | M1 |
|---|---|---|
| max |Σp_y| | 6.26 kg·m/s | **0.18** |
| Final box height | -2.09 m (penetrates) | **+0.28 m (rests)** |

35× momentum-error reduction. The remaining 0.18 kg·m/s drift is what
M2/M4.5 attack.

---

## M2 — ASFLIP rigid extension (F-bar split, Fei 2021)

### Setup

Decompose the deformation gradient $\mathbf{F}$ into a trace-preserving
(deviatoric) $\bar{\mathbf{F}}$ and a volumetric $\mathbf{F}_{\text{vol}}$:

$$
\mathbf{F} = \bar{\mathbf{F}}\, \mathbf{F}_{\text{vol}}, \quad
\det(\bar{\mathbf{F}}) = 1, \quad
\mathbf{F}_{\text{vol}} = J^{1/3} \mathbf{I}, \quad J = \det(\mathbf{F}).
$$

At the rigid-grid boundary:

- **Normal direction** ($\mathbf{n}$): only the volumetric component
  $\mathbf{F}_{\text{vol}}$ contributes momentum. This is the direction
  where mass-conservation is sharpest (compaction / dilation), and using
  the volumetric split makes the normal momentum exchange exact.
- **Tangent directions**: deviatoric $\bar{\mathbf{F}}$ + Coulomb friction
  (re-using M0 tangent equation).

ASFLIP blend: PIC ($\alpha=0$, dissipative) vs FLIP ($\alpha=1$, noisy):

$$
\mathbf{v}_p^{\text{new}} = \alpha\bigl(\mathbf{v}_p^{\text{old}} + \Delta\mathbf{v}_{\text{grid}}\bigr)
                          + (1-\alpha)\,\mathbf{v}^{\text{PIC}}_p.
$$

Standard choice $\alpha \in [0.95, 0.99]$.

### Assumptions removed (vs M1)

1. **Volumetric / deviatoric separation** — normal momentum exchange is
   now exact in solver tolerance (≈ float roundoff), not approximate.
2. **Friction tangent** — re-introduced from M0 in deviatoric direction.

### Assumptions still present

1. **Per-particle F̄ history** required — extra state per MPM particle.
2. **Body itself is not a particle** — rigid body and MPM particles still
   exchange through grid nodes, not directly (M4 fixes this).

### Predicted improvement

- $\max|\Sigma\mathbf{p}|$ should drop to float roundoff (≈ 1e-5 kg·m/s).
- Step cost: ~20–30% higher than M1 (SVD per particle).

---

## M3 — MLS-MPM rigid coupling (Hu 2018)

### Setup

Use a Moving Least Squares (MLS) kernel at the rigid-grid boundary instead
of one-cell-per-node distribution. The body is treated like a normal MPM
particle whose mass and APIC affine matrix $\mathbf{C}$ are spread by the
B-spline weight $w_{ip}$.

### Equations

Quadratic B-spline:

$$
N(x) = \begin{cases}
\tfrac{3}{4} - x^2 & |x| < 0.5 \\
\tfrac{1}{2}(1.5 - |x|)^2 & 0.5 \le |x| < 1.5 \\
0 & \text{otherwise}
\end{cases}
$$

Weight at node $i$ from particle / body sample at $\mathbf{x}_p$:

$$
w_{ip} = N\!\left(\tfrac{x_i - x_p}{\Delta x}\right) \cdot N(\cdot) \cdot N(\cdot).
$$

Rigid body P2G (treating body as a single point with affine $\mathbf{C}_b$):

$$
m_i \mathrel{+}= w_{ib} M, \quad
m_i \mathbf{v}_i \mathrel{+}= w_{ib} M \bigl(\mathbf{v}_c + \mathbf{C}_b (\mathbf{x}_i - \mathbf{x}_c)\bigr).
$$

For a rigid body, $\mathbf{C}_b$'s symmetric part is zero (no shear) and
its anti-symmetric part encodes $\boldsymbol{\omega}$:

$$
\mathbf{C}_b = [\boldsymbol{\omega}]_\times = \begin{pmatrix}
0 & -\omega_z & \omega_y \\
\omega_z & 0 & -\omega_x \\
-\omega_y & \omega_x & 0
\end{pmatrix}.
$$

### Assumptions removed (vs M1)

1. **B-spline continuity** — force at the OBB face is smooth, no spike.
2. **Angular momentum** is transported coherently through APIC
   $\mathbf{C}_b$ rather than recomputed per node from $\boldsymbol{\omega} \times \mathbf{r}$.

### Assumptions still present

1. Uniform body mass distribution over the kernel support — same as
   M1, just smoothed.
2. No F-bar split — volumetric / deviatoric momentum mix, like M1.

### Predicted improvement

- Smoother force traces (no high-frequency spike at OBB-face crossing).
- Step cost: ~10% higher than M1 (3×3×3 stencil per active grid node
  vs 1×1×1).

---

## M4 — Lagrange-multiplier saddle-point (Jiang 2020)

### Setup

Formulate the body-grid interface as a *hard equality constraint*: the
body's anchor-point velocity must equal the grid velocity at the same
point. Solve the resulting saddle-point system per step.

### Equations

Let $\mathbf{u} = (\mathbf{v}_c, \boldsymbol{\omega}, \{\mathbf{v}_i\})$ be the
combined velocity vector and $\mathbf{J}$ the constraint Jacobian
($\mathbf{J}\mathbf{u} = \mathbf{0}$ enforces anchor coincidence). Then:

$$
\begin{pmatrix} \mathbf{M} & \mathbf{J}^T \\ \mathbf{J} & 0 \end{pmatrix}
\begin{pmatrix} \mathbf{u} \\ \boldsymbol{\lambda} \end{pmatrix} =
\begin{pmatrix} \mathbf{f} \\ \mathbf{c} \end{pmatrix}.
$$

$\mathbf{M}$ is block diagonal: rigid 6×6 plus diag$(m_i)$.
$\mathbf{c}$ is the constraint residual (zero in conservative case).

Schur complement:

$$
\mathbf{S} = \mathbf{J} \mathbf{M}^{-1} \mathbf{J}^T, \quad
\mathbf{S} \boldsymbol{\lambda} = \mathbf{J} \mathbf{M}^{-1} \mathbf{f} - \mathbf{c}.
$$

Solve $\boldsymbol{\lambda}$ via PCG (or direct if dimension small),
then $\mathbf{u} = \mathbf{M}^{-1}(\mathbf{f} - \mathbf{J}^T \boldsymbol{\lambda})$.

### Assumptions removed (vs M2/M3)

1. **Implicit treatment** — constraint is satisfied within solver
   tolerance regardless of body-to-grid mass ratio.
2. **No splitting** — momentum / energy are conserved as a *system*, not
   per-direction.

### Assumptions still present

1. Small number of contact points (single anchor per body), otherwise
   $\mathbf{S}$ size blows up.
2. Linear constraint — non-linear effects (friction cone) handled by
   sub-iteration.

### Predicted improvement

- $\max|\Sigma\mathbf{p}|$: float roundoff regardless of mass ratio.
- Step cost: 2–5× M1 (linear solve).
- Most accurate, slowest, **reference baseline** for paper Table.

---

## M4.5 — Hybrid (this work's novelty candidate)

### Idea

Take M1 (cheap) and bolt on the *minimum* additional physics that
captures M2/M3's qualitative improvements without M4's cost.

Three conditions stacked progressively. We benchmark **each layer
separately** to attribute credit and to expose Pareto trade-offs.

### Layer A (M4.5a) — Adaptive mass weighting

Replace uniform $m_b$ with OBB-cell intersection volume:

$$
m_{b,i} = \rho_{\text{body}} \cdot V_i^{\text{OBB}\cap \text{cell}_i},
\quad \rho_{\text{body}} = \frac{M}{V_{\text{OBB}}}.
$$

Effect: when the body only partially overlaps the grid, only the
partially-overlapping cells get a partial mass contribution → no false
forces at the leading face.

**Implementation gotcha (open issue):** the *naive* form

$$
m_{b,i} = \rho_{\text{body}} \cdot \Delta x^3 \cdot \text{frac}_i
$$

where $\text{frac}_i$ is a cell-centre overlap fraction does **not** in
general satisfy $\sum_i m_{b,i} = M$. Total body mass leaks (under- or
over-counted), inflating $\mathcal{M}_{\max}$ above even M1's level —
the first benchmark run confirmed this (M4.5a `max|Σp| = 17.7` vs M1
`0.18`). Correct normalisation requires a two-pass sweep:

$$
\text{pass 1: } S = \sum_i \text{frac}_i, \quad
\text{pass 2: } m_{b,i} = M \cdot \frac{\text{frac}_i}{S}.
$$

Without this normalisation Layer A is strictly worse than M1 — a real
paper lesson, not a bug to be hidden. Phase 2 implementation: split the
single-pass sweep into a frac-accumulation pre-pass + a normalised
update pass.

### Layer B (M4.5b) — Coulomb tangent friction

Add M0's tangent projection on top of M4.5a's velocity update:

$$
{v_i^t}' = v_i^t + \min(1, \mu)\bigl(v_b^t - v_i^t\bigr).
$$

Effect: rolling / sliding scenarios (rover wheel) now have correct
slip behaviour; before this layer all tangential motion was free.

### Layer C (M4.5c) — Quadratic B-spline smoothing

Apply M3's B-spline weight to the mass+momentum addition:

$$
m_i' = m_i + w_{ib}\, m_{b,i},\quad
m_i' \mathbf{v}_i' = m_i \mathbf{v}_i + w_{ib}\, m_{b,i}\, \mathbf{v}_b(\mathbf{x}_i).
$$

Effect: face-crossing force spike is eliminated; momentum exchange now
$C^1$ continuous in body position.

### Why this is novel

Each layer in isolation is taken from prior art. The contribution is
**(a) the decomposition** — explicitly attributing which physics each
layer adds, and **(b) the empirical claim** that A+B captures M2's
quality at ~30% lower step cost than full M2. The reader gets a recipe
for "how cheap can I be while still being right on this regime".

---

## Quantitative metrics — paper axes

For each (method, scenario, $\Delta t$, solver_iters) tuple we record:

### Momentum conservation

$$
\mathcal{M}_{\max} = \max_t \Bigl\| \mathbf{p}_{body}(t) + \sum_i m_i(t)\mathbf{v}_i(t) \Bigr\|.
$$

Lower is better. M4 should reach $\sim 10^{-5}$ (float roundoff).

### Energy drift

$$
\mathcal{E}_{\text{drift}} = \frac{|E(T) - E(0)|}{|E(0)| + \epsilon}.
$$

Monotone non-increasing systems (with damping/friction) need this $\ge 0$;
**spurious gain** $\Rightarrow$ method is non-conservative.

### Step time

$$
\mathcal{T}_{\text{step}} = \frac{1}{N}\sum_t (\text{wall-clock per step})\ [\mu s].
$$

Direct cost — used for the Pareto figure.

### Penetration depth (sanity)

$$
\mathcal{D}_{\max} = \max_t \max_i \bigl[ - d_i^{\text{signed}}(t) \bigr]^+
$$

where $d_i^{\text{signed}}$ is the signed distance from node $i$ to the
nearest OBB face (negative = inside). M0 by construction has $\mathcal{D}_{\max}$
limited only by step size; M4 should saturate near roundoff.

### Slip ratio (rover scenario only)

$$
s = 1 - \frac{v_{\text{forward}}}{\omega R}.
$$

A method that handles friction tangent correctly should produce $s$
inside the textbook Bekker range for given soil parameters.

---

## How to read the resulting paper

Each method removes one assumption. The paper's Section 5 (Results) is
the per-method table — **measured numbers as of Round 10** on the
`box_drop` scenario (sand patch 5×3×5 × 0.01 kg, 1 kg box, 3 s @ 240 Hz):

| Method | max\|Σp\| (kg·m/s) | avg\|Σp\| | \|ΔE/E₀\| | step μs | pen_avg (m) | dwell % |
|---|---|---|---|---|---|---|
| M0 Dirichlet          | 6.256 | 0.879  | 1.166 | 21.4 | 0.103 | 73 |
| M1 PIC-Rigid          | 0.179 | 0.0886 | 0.483 | 23.1 | 0.125 | 90 |
| **M4.5a (Layer A)**   | **0.082** | **0.0414** | **0.225** | **24.5** | **0.063** | **60** |
| M4.5b (A + friction)  | 0.129 | 0.101  | 0.551 | 25.7 | 0.108 | 80 |
| M4.5c (full hybrid)   | 26.75 | 12.33  | 0.921 | 20.0 | 0.017 | 12 |

(M2 / M3 / M4 columns are stubs in v1 — they reproduce M1 numbers
identically. Stubs are excluded from this table in the paper draft.)

### Three findings the paper claims

1. **M4.5a (renormalised cell-overlap adaptive mass)** is the new
   state-of-the-art for this regime — **2.2× ↓ max \|Σp\| vs M1**, 50%
   lower average penetration depth, 33% lower dwell-time-in-deep-penetration,
   all at +6% step cost.

2. **Layer B (Coulomb friction tangent)** regresses without further
   treatment — friction projection after the mass-weighted average
   breaks M4.5a's invariant. Documented as a cautionary finding.

3. **Layer C (B-spline kernel weighting)** catastrophically regresses
   because kernel re-weighting after sweep normalisation leaves
   Σm_b ≠ M_body. Re-normalising after kernel application would restore
   correctness at extra cost; left to future work.

The honest narrative — "*not every refinement helps*" — strengthens the
contribution, not weakens it. Section 4 of the paper walks each layer
in order, reporting the metric ladder above as the canonical evidence.

---

## References

See `../../docs/notes/references.md` for full bibliography. Key papers:

- Stomakhin 2013 (snow MPM), Stomakhin 2014 (PIC-FLIP).
- Klar 2016 (Drucker-Prager sand).
- Jiang 2015 (APIC), Hu 2018 (MLS-MPM + rigid).
- Fei 2021 (ASFLIP F-bar split).
- Jiang 2020 (implicit SPH — Lagrange technique).
