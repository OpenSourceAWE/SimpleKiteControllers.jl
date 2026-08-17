# Combining heading and course

## What is actually used as `psi_prime`

This controller is controlling $\psi'$, a linear combination of heading and course
defined as:

$$
\psi' = \psi + w_\chi \cdot {\mathrm{wrap}}_\pi(\chi - \psi)
\tag{1}
$$

where $\psi$ is the heading, $\chi$ the course, and
${\mathrm{wrap}}_\pi(\cdot)$ maps an angle onto $[-\pi, \pi)$. The wrapping is
not cosmetic: a plain convex combination $(1-w_\chi)\psi + w_\chi\chi$ would sweep
the long way round whenever the two angles straddle the $\pm\pi$ branch cut.
Written this way, $\psi'$ is a pure heading signal at $w_\chi = 0$, a pure course
signal at $w_\chi = 1$, and the shortest-arc interpolation between them in
between.

### The blending weight

Unlike the reference formulation below, the weight is **not** scheduled on the
angular velocity in the small-earth projection, but directly on the kite speed
$v_k = \lVert {\mathbf{v}}_\mathrm{kite} \rVert$:

$$
w_\chi = \mathrm{clamp}\left(
  \frac{v_k - v_{k,\psi}}{v_{k,\chi} - v_{k,\psi}},\ 0,\ 1 \right)
\tag{2}
$$

with $v_{k,\psi} = 5\,\mathrm{m/s}$ (`v_kite_heading`) and
$v_{k,\chi} = 10\,\mathrm{m/s}$ (`v_kite_course`). At or below $5\,\mathrm{m/s}$
the loop feeds back heading alone, at or above $10\,\mathrm{m/s}$ course alone,
and the two are blended linearly in the band between. The motivation is the same
as in the reference: the course estimate is derived from the kite's motion in the
small earth projection and is meaningless when the kite is nearly stationary,
while the heading is always observable.

Scheduling on translational speed rather than on angular velocity was chosen
because the quantity that actually degrades the course estimate is how far the
kite moves per sample, not how fast it turns. A kite flying straight and fast has
a perfectly good course estimate but $\omega \approx 0$, and the
$\omega$-schedule would wrongly fall back to heading there.

### Course sign convention

The course taken from `SysState` is corrected by $\pi$ before it enters (1), in
[`calc_steering`](../src/course_controller.jl):

```julia
course_shifted = wrap2pi(course + ccs.course_offset)   # course_offset defaults to π
psi_prime = heading + w_course * wrap2pi(course_shifted - heading)
```

The raw tangent-frame course has its zero pointing *away* from zenith, whereas
the bearing convention used everywhere else in the guidance is `0` = towards
zenith, positive towards larger azimuth. The $+\pi$ puts course and heading on the
same zero and the same sign, which is a precondition for (1) being meaningful at
all — blending two angles measured in different frames would produce a signal
that is neither.

### Optional bypass on the pattern

Path following is a course problem, so from phase 3 (transition and figure eight)
onwards the speed schedule can be bypassed and the course fed back at any speed:

$$
w_\chi = 1 \quad \text{if } \mathtt{fig8\_pure\_course} \wedge \text{phase} \ge 3
\tag{3}
$$

The reason is that on the pattern the schedule only dips into the blending band
during the slow part of a turn, i.e. it swaps the feedback signal mid-manoeuvre,
which is exactly when a consistent signal matters most. The entry phases (dive
and hold) always keep the schedule, since the kite genuinely is slow there. With
the shipped settings (`data/fc_settings.yaml`,
`data/fc_settings_reelout.yaml`) `fig8_pure_course` is `false`, so the speed
schedule governs in every phase.

### Use in the loop

$\psi'$ is the *regulated* variable, not the setpoint: the heading PID is driven
by the wrapped error against the commanded course $\chi_\mathrm{cmd}$,

$$
e = {\mathrm{wrap}}_\pi(\psi' - \chi_\mathrm{cmd})
\tag{4}
$$

and regulated against a zero reference (`DiscretePID` does not wrap, so the error
is formed outside it). The proportional gain is scheduled on apparent wind speed,
$K \sim 1/v_a$, because the plant turn rate is $\dot\psi = c_1 v_a u_s$.

In the logs, $w_\chi$ is recorded as `var_08` (0 = heading, 1 = course) and $e$ as
`var_06`, so `var_06` equals `heading - bearing` at low speed and
`course - bearing` at high speed.

### Relation to the reference formulation

The form given in (1) and the $\mathrm{atan2}$ fusion of (6.17) below are
two different interpolations between the same two angles: (6.17) normalises the
convex combination of the two unit vectors (an *nlerp*), whereas (1) interpolates
the angle itself (a *slerp*). They agree exactly at $w_\chi \in \{0, 0.5, 1\}$ and
differ only slightly in between — below $0.15°$ for a $30°$ gap between heading
and course, and about $4°$ at a $90°$ gap. For the V3's typical
course-minus-heading drift angle of ${\sim}13°$ the difference is under $0.02°$,
i.e. far below the noise on either input. Form (1) was preferred because it makes
the endpoints and the clamp behaviour obvious by inspection.

The two remaining differences from the reference are deliberate:

| | Reference (6.16/6.17) | This package |
|---|---|---|
| Scheduling variable | angular velocity $\omega$ in the small earth projection | kite speed $\lVert{\mathbf{v}}_\mathrm{kite}\rVert$ |
| Weight limit | $k_\chi \le 0.85$ | $w_\chi \le 1$ |
| Interpolation | nlerp via $\mathrm{atan2}$ | slerp via wrapped difference |

The $0.85$ cap exists in the reference to stop the kite getting too slow when
flying figures of eight at very high elevation. It is not carried over here: the
elevation of the pattern is set by the guidance (the lemniscate and, during
entry, the descent limiter `entry_chi_max`) rather than by leaving a residual
heading component in the feedback signal, so the effect the cap guarded against
is addressed upstream of $\psi'$.


## Just as reference, not used in this form for this package

The accuracy of the controller can be improved further, if not only the heading angle is
controlled in a feedback loop, but also the course angle: A difference between the course
and the heading angles is induced (i) by the gravity forces and (ii) by wind turbulences.
On the other hand the course of the kite can only be controlled when it is moving.
For controlling a kite, that is moving only very slowly in the small earth projection the
heading angle the must be used. Therefore a linear parameter varying (LPV) controller
is used, that uses mainly the course angle at high angular velocities ($\omega > 2.275\omega_{up}$) and
only the heading angle, for $\omega < \omega_{up}$ . Both angles are fused according to the following
algorithm:
$$
k_\chi =
\begin{cases}
\min\left(\dfrac{\omega - 0.8}{1.2},\ 0.85\right) & \text{if } \omega > \omega_{up} \\[4pt]
0.0 & \text{else}
\end{cases}
\tag{6.16}
$$

$$
\begin{aligned}
x &= \sin\psi\,(1 - k_\chi) + \sin\chi\, k_\chi \\
y &= \cos\psi\,(1 - k_\chi) + \cos\chi\, k_\chi \\
\psi' &= \mathrm{atan2}(x, y)
\end{aligned}
\tag{6.17}
$$

The value of $k_\chi$ is limited to 0.85. This avoids the kite to get too slow when flying figures
of eight at a very high elevation angle. The block diagram of this controller is shown in
Fig. 6.9.