# input parameters of the init and the step functions of the REST API of the flight path planner/ optimizer

struct WinchParams
    mode::String # valid values: "reelout", "reelin"
    k_v::Float64 # v_set = k_v * sqrt(force)
    v_max::Float64 # maximum winch speed
    f_min::Float64 # minimum winch force
    f_max::Float64 # maximum winch force
end

# should be 1000 points
# should be repetitive, i.e. the last point should be equal to the first point
struct Trajectory
    azimuth::Vector{Float64}   # in degrees
    elevation::Vector{Float64} # in degrees
end

struct InitParams
    name::String      # name of the simulation
    max_time::Float64 # maximum time for the simulation
    length::Float64   # initial length of the tether
    winch_params::WinchParams
    trajectory::Trajectory
end

struct StepParams
    winch_params::WinchParams
    trajectory::Trajectory
end