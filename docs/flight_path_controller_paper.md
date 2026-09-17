# Flight path controller — paper figure companion

Text that was moved out of `flight_path_controller2.drawio` to make
`flight_path_controller_paper.drawio` legible in print. The figure only carries
block names, letter tags and signal symbols; everything below goes into the
caption, the running text and a table.

The paper variant is 620 × 300 px with 11 px text. Placed at 17 cm width
(`figure*` in a two-column layout, or full width in one column) that gives
~8.6 pt text. It is **not** meant for a single 8.4 cm column — that would halve
it to ~4.3 pt. Export with `drawio -x -f pdf --crop` (done:
`flight_path_controller_paper.pdf`) and include with
`\includegraphics[width=\textwidth]{...}`.

## Caption

> **Figure N.** Flight path controller as a cascade of guidance and steering.
> The reference path (A) is a lemniscate in azimuth/elevation, fixed before
> flight. Guidance (B) finds the closest path point, walks a fixed arc ahead to
> the attractor point and returns the great-circle course to it,
> $\chi_\mathrm{set}$. The entry logic (C) overrides this course during the dive
> and hold phases and limits its steepness during the transition, giving
> $\chi_\mathrm{cmd}$. The gain-scheduled PD controller (D) closes the loop on
> the error $e$ between $\chi_\mathrm{cmd}$ and the blended feedback angle
> $\psi'$ (G), a speed-dependent mix of heading $\psi$ and course $\chi$, and
> outputs the relative steering $u_s$. The depower ladder (F) sets the relative
> depower $u_d$ per phase. Blocks B–G run once per timestep. The winch loop (H)
> closes on tether force and is independent of the steering loop. Solid grey
> lines are measurements from the plant (E); dashed orange lines carry the
> discrete phase.

## Running text (equations)

Signal conventions: bearings are measured from the zenith direction, positive
towards larger azimuth, so $\chi_\mathrm{set}$ is directly comparable to the
kite's heading $\psi$. A positive $u_s$ produces a positive heading rate on this
plant (fed unnegated).

Feedback blend (G):

$$
\chi' = \operatorname{wrap}(\chi + \chi_\mathrm{off}), \qquad
\psi' = \psi + w \cdot \operatorname{wrap}(\chi' - \psi), \qquad
w = \operatorname{clamp}\!\left(\frac{v_k - v_{k,\psi}}{v_{k,\chi} - v_{k,\psi}},\,0,\,1\right)
$$

with $w = 1$ from phase 3 on when pure-course feedback is enabled. Below
$v_{k,\psi}$ the controller follows heading, above $v_{k,\chi}$ course.

Error and gain schedule (D):

$$
e = \operatorname{wrap}(\psi' - \chi_\mathrm{cmd}), \qquad
K = K_\mathrm{ph} \, \frac{v_{a,\mathrm{ref}}}{\max(v_a, v_{a,\min})}, \qquad
K_\mathrm{ph} = \begin{cases} k_e K_p & \text{phase} < 3 \\ K_p & \text{phase} \ge 3 \end{cases}
$$

The $1/v_a$ scheduling cancels the plant's turn-rate law
$\dot\psi \approx c_1 v_a u_s$; $k_e$ (25 %) softens the entry phases. The
controller is PD (no integral term) with a filtered derivative, output clamped
to $\pm u_{s,\max}$. Engagement is bumpless: while parked (phase 0) the PD is
stepped with zero error and its output discarded, so the first command in
phase 1 is not a step.

Entry logic (C): phases 0 park → 1 dive → 2 hold → 3 transition → 4 figure-eight.
In phases 1 and 2, $\chi_\mathrm{cmd}$ is fixed to $\chi_\mathrm{dive}$ /
$\chi_\mathrm{hold}$. Above the entry gate the descent limiter blends
$\chi_\mathrm{set}$ towards $\pm\chi_\mathrm{max}$ (wrapped, so it never sweeps
the long way round at $\pm 180°$).

Depower ladder (F): $u_d$ steps from the entry value (dive/hold) to the pattern
setpoint and, when the winch triggers the final phase, to the final value; each
step is ramped over $T_r$.

## Symbol ↔ parameter table

| Symbol | Meaning | `FC_Settings` key | Default |
|---|---|---|---|
| $\alpha, \beta$ | azimuth, elevation of the kite / path | — | — |
| $\psi, \chi$ | heading, course (from `SysState`) | — | — |
| $v_k, v_a$ | kite speed, apparent wind speed | — | — |
| $u_s, u_d$ | relative steering, relative depower | — | — |
| $F_t$ | tether force (winch loop input) | — | — |
| $d_a$ | arc distance from closest point to attractor | `attractor_distance` (`FigureEightSettings`) | 7° |
| $\chi_\mathrm{off}$ | course offset before blending | `course_offset` (`CourseControllerSettings`) | $\pi$ |
| $v_{k,\psi}, v_{k,\chi}$ | blend endpoints: pure heading below, pure course above | `v_kite_heading`, `v_kite_course` | 5, 10 m/s |
| — | force $w=1$ from phase 3 | `fig8_pure_course` | false |
| $K_p$ | PD proportional gain at $v_a = v_{a,\mathrm{ref}}$ | `heading_p` | 0.1941 |
| $\tau_D$, $N$ | derivative time, derivative filter | `heading_d`, `heading_d_n` | 0.12 s, 2 |
| $v_{a,\mathrm{ref}}, v_{a,\min}$ | gain-schedule reference and floor | `v_app_ref`, `v_app_min` | 27, 10 m/s |
| $k_e$ | gain factor in phases 0–2 | `entry_gain` | 0.25 |
| $u_{s,\max}$ | steering clamp | `max_steering` | 0.32 |
| $\chi_\mathrm{dive}, \chi_\mathrm{hold}$ | fixed course in phase 1, 2 | `chi_dive`, `chi_hold` | −85°, −90° |
| $\chi_\mathrm{max}$ | steepness limit in the transition | `entry_chi_max` | 95° |
| — | descent-limiter gate and blend width (cross-track) | `entry_d_gate`, `entry_d_blend` | 12°, 4° |
| — | hold duration; gate for entering phase 4 | `hold_time`, `fig8_d_gate` | 0.8 s, 5° |
| $u_{d,\mathrm{entry}}, u_{d,\mathrm{set}}, u_{d,\mathrm{fin}}$ | depower in entry / pattern / final phase | `entry_depower`, `depower_setpoint`, `depower_final` | 0.34, 0.26, 0.328 |
| $T_r$ | depower ramp time | `depower_blend_time` | 4 s |

Defaults are the struct defaults in `src/course_controller.jl` /
`src/fc_settings.jl`; a run's actual values are in the archived
`fc_settings_*.yaml` next to its log.
