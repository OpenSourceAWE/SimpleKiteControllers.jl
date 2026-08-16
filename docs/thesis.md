# Combining heading and course

**Just as reference, not used in this form for this package**

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
\psi' &= \operatorname{atan2}(x, y)
\end{aligned}
\tag{6.17}
$$

The value of $k_\chi$ is limited to 0.85. This avoids the kite to get too slow when flying figures
of eight at a very high elevation angle. The block diagram of this controller is shown in
Fig. 6.9.