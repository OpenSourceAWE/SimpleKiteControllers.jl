@with_kw mutable struct ParkingControllerSettings @deftype Float64
    dt
    # turn rate controller settings
    kp_tr=0.06
    ki_tr=0.0012
    # outer controller (heading/ course) settings
    kp=15
    ki=0.5
    # NDI block settings
    va_min = 5.0   # minimum apparent wind speed
    va_max = 100.0 # maximum apparent wind speed
    k_ds = 2.0 # influence of the depower settings on the steering sensitivity
    c1 = 0.048 # v9 kite model
    c2 = 0     # a value other than zero creates more problems than it solves
end

mutable struct ParkingController
    pcs::ParkingControllerSettings
    pid_tr::DiscretePID
    pid_outer::DiscretePID
    last_heading::Float64
    chi_set::Float64
    last_ndi_gain::Float64
end

function ParkingController(pcs::ParkingControllerSettings; last_heading = 0.0)
    Ts = pcs.dt
    pid_tr    = DiscretePID(;K=pcs.kp_tr, Ti=pcs.kp_tr/ pcs.ki_tr, Ts)
    pid_outer = DiscretePID(;K=pcs.kp,    Ti=pcs.kp/ pcs.ki,       Ts)
    return ParkingController(pcs, pid_tr, pid_outer, last_heading, 0, 0)
end

"""
    linearize(pcs::ParkingControllerSettings, psi_dot, psi, elevation, v_app; ud_prime=0)

Nonlinear, dynamic inversion block (NDI) according to Eq. 6.4 and Eq. 6.12.

Parameters:
- psi_dot: desired turn rate in radians per second
- psi: heading in radians
- elevation: elevation angle in radians
- v_app: apparent wind speed in m/s
- ud_prime: depower setting in the range of 0 to 1, 0 means fully powered, 1 means fully depowered
"""
function linearize(pc::ParkingController, psi_dot, psi, elevation, v_app; ud_prime=0)
    pcs = pc.pcs
    # Eq. 6.13: calculate va_hat
    va_hat = clamp(v_app, pcs.va_min, pcs.va_max)
    # Eq. 6.12: calculate the steering from the desired turn rate
    u_s = (1.0 + pcs.k_ds * ud_prime) / (pcs.c1 * va_hat) * (psi_dot - pcs.c2 / va_hat * sin(psi) * cos(elevation))
    if abs(psi_dot) < 1e-6
        ndi_gain = pc.last_ndi_gain
    else
        ndi_gain = clamp(u_s / psi_dot, -20, 20.0)
    end
    pc.last_ndi_gain = ndi_gain
    return u_s, ndi_gain
end

"""
    navigate(fpc, limit=50.0)

Calculate the desired flight direction chi_set using great circle navigation.
Limit delta_beta to the value of the parameter limit (in degrees).
"""
function navigate(pc::ParkingController, azimuth, elevation; limit=50.0)
    phi_set  = 0.0         # azimuth
    beta_set = deg2rad(77) # zenith
    beta = elevation
    phi = azimuth
    # println("beta: $(rad2deg(beta)), phi: $(rad2deg(phi))")
    r_limit = deg2rad(limit)
    if beta_set - beta > r_limit
        beta_set = beta + r_limit
    elseif beta_set - beta < -r_limit
        beta_set = beta - r_limit
    end
    y = sin(phi_set - phi) * cos(beta_set)
    x = cos(beta) * sin(beta_set) - sin(beta) * cos(beta_set) * cos(phi_set - phi)
    pc.chi_set = atan(y, x)
end

"""
    calc_steering(pc::ParkingController, heading; elevation=0.0, v_app=10.0, ud_prime=0.0)

Calculate rel_steering and ndi_gain from the actual heading, elevation, and apparent wind speed.

Parameters:
- pc: parking controller
- heading: actual heading in radians
- elevation: elevation angle in radians
- v_app: apparent wind speed in m/s
- ud_prime: depower setting in the range of 0 to 1, 0 means fully powered, 1 means fully depowered

"""
function calc_steering(pc::ParkingController, heading, chi_set; elevation=0.0, v_app=10.0, ud_prime=0.0)
    # calculate the desired turn rate
    heading = wrap2pi(heading) # a different wrap2pi function is needed that avoids any jumps
    psi_dot_set = pc.pid_outer(wrap2pi(chi_set), heading)
    psi_dot = (wrap2pi(heading - pc.last_heading)) / pc.pcs.dt
    pc.last_heading = heading
    psi_dot_in = pc.pid_tr(psi_dot_set, psi_dot)
    # linearize the NDI block
    u_s, ndi_gain = linearize(pc, psi_dot_in, heading, elevation, v_app; ud_prime)
    u_s, ndi_gain, psi_dot, psi_dot_set
end
