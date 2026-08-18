# Trajectory optimization

## First test case (in Julia)

1. define inflow conditions
2. send init request with initial guess
3. receive trajectory
4. fly it

### Open questions:

**Which inflow conditions**

My current suggestion would be the struct:
```python
class InflowConditions(TypedDict):
    wind_speed: float                   # in m/s at 6 m height
    wind_direction: float               # in degrees, 0 = North, 90 = East
    profile_law: Literal[0, 1, 2, 3]    # 0=CONST, 1=EXP, 2=LOG, 3=EXPLOG
    # the following fields are optional; the defaults given in the comments are used if omitted
    alpha: NotRequired[float]           # exponent of the wind profile law, default: 0.08163
    z0: NotRequired[float]              # surface roughness [m], default: 0.0002
    turbulence: NotRequired[float]      # in [0, 1], 0 = no turbulence, 1 = full turbulence, default: 0.0
```
Is that good enough for you, or what shall I add/ change?
--> can be implemented, currently only LOG and CONST implemented

I see you defined:
```python
              wind = Dict("model_type" => "tabulated",
                          "heights" => [10.0, 100.0, 300.0],
                          "speeds" => [5.5, 8.0, 9.3]),
```
I could add that as custom wind profile law. But that would require some discussion first, because it
does not really define a profile law.
--> Which interpolation would be good? Uwe will investigate.

**Which additional parameters are needed for init**

I see you passed:
```python
             extras = Dict("sim_parameters" => Dict(
                 "input_depower" => 1.6, "reg_weight" => 1.0,
                 "detect_simple_bounds" => true)))
```
Shall these parameters be added to `InitParams` ?
We should define for V3 what is zero and what is one for rel_depower.
What is the meaning of these parameters?
--> Uwe: Check how rel_depower is converted to depower line length in KitePod.jl

**Resolved 2026-08-18** (see [steering_depower.md](steering_depower.md)):
`input_depower` is `l_dp`, an absolute power-tape length in metres, and both sides
move 5 m of tape per unit of `rel_depower` — but off different zeros, AWETrim's
`l_dp = 0.6 + 5*u_p` against V3Kite's `0.2 + 5*u_p`. The 0.4 m offset is 8 kcu-%.
It is not a rescaling that can simply be applied: pinning `input_depower` at the
flown depower leaves the solve infeasible.

I think, all parameters needed for the initialization should be part of the `InitParams` struct.

**Which result are we interested in**
- average force
- average speed
- average power
- RMS path following error in degrees
- anything else ?

--> Uwe can compare time series data.

**Other questions**
Is an initial guess needed for the trajectory?
-> YES
Optimizer requires it.