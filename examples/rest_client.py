# input parameters of the init and the step functions of the REST API of the flight path planner/ optimizer

from typing import Literal, TypedDict


class WinchParams(TypedDict):
    mode: Literal["reelout", "reelin"]
    k_v: float      # v_set = k_v * sqrt(force)
    f_min: float    # minimum winch force
    f_max: float    # maximum winch force


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
    trajectory: Trajectory


class StepParams(TypedDict):
    winch_params: WinchParams
    trajectory: Trajectory
