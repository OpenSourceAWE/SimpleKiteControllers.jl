# input parameters of the init and the step functions of the REST API of the flight path planner/ optimizer

Base.@kwdef struct InflowConditions
    wind_speed::Float64     # in m/s at 6 m height
    wind_direction::Float64 # in degrees, 0 = North, 90 = East
    profile_law::Int64      # 0=CONST, 1=EXP, 2=LOG, 3=EXPLOG, 4=CUSTOM_LOG, 5=CUSTOM_EXP, 6=CUSTOM_JET
    # the custom profiles are fitted using the heights and speeds given in the heights and speeds fields
    # CUSTOM_JET: u(z) = u_bg(z) + U_J * exp(-(z - z_c)^2 / (2*sigma^2))
    # the following fields are optional; the defaults given below are used if omitted
    alpha::Float64 = 0.08163            # exponent of the wind profile law
    z0::Float64 = 0.0002                # surface roughness                                          [m]
    turbulence::Float64 = 0.0           # in [0, 1], 0 = no turbulence, 1 = full turbulence
    heights::Vector{Float64} = [6.0]    # heights at which the wind speed is given
    speeds::Vector{Float64} = [wind_speed] # wind speeds at the given heights
end

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
    inflow_conditions::InflowConditions
    trajectory::Trajectory
end

struct StepParams
    winch_params::WinchParams
    trajectory::Trajectory
end