# Trajectory optimization

## First test case (in Julia)

1. define inflow conditions
2. send init request
3. receive trajectory
4. fly it

### Open questions:

**Which inflow conditions**

My current suggestion would be the struct:
```
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

**Which result are we interested in**
- average force
- average power
- RMS path following error in degrees
- anything else ?