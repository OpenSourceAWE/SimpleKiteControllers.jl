# input parameters of the init and the step functions of the REST API of the flight path planner/ optimizer

from typing import Literal, NotRequired, TypedDict


class WinchParams(TypedDict):
    mode: Literal["reelout", "reelin"]
    k_v: float      # v_set = k_v * sqrt(force)
    f_min: float    # minimum winch force
    f_max: float    # maximum winch force

class InflowConditions(TypedDict):
    wind_speed: float                   # in m/s at 6 m height
    wind_direction: float               # in degrees, 0 = North, 90 = East
    profile_law: Literal[0, 1, 2, 3]    # 0=CONST, 1=EXP, 2=LOG, 3=EXPLOG
    # the following fields are optional; the defaults given in the comments are used if omitted
    alpha: NotRequired[float]           # exponent of the wind profile law, default: 0.08163
    z0: NotRequired[float]              # surface roughness [m], default: 0.0002
    turbulence: NotRequired[float]      # in [0, 1], 0 = no turbulence, 1 = full turbulence, default: 0.0


# should be 1000 points
# should be repetitive, i.e. the last point should be equal to the first point
class Trajectory(TypedDict):
    azimuth: list[float]    # in degrees
    elevation: list[float]  # in degrees


class InitParams(TypedDict):
    name: str          # name of the simulation
    max_time: float    # maximum time for the simulation
    length: float      # initial length of the tether
    winch_params: WinchParams
    inflow_conditions: InflowConditions
    trajectory: Trajectory


class StepParams(TypedDict):
    winch_params: WinchParams
    trajectory: Trajectory
