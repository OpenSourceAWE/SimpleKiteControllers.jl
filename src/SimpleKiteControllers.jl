module SimpleKiteControllers

using DiscretePIDs, Parameters, KiteUtils

export ParkingController, ParkingControllerSettings
export linearize, calc_steering, navigate

include("parking_controller.jl")
include("figure_eight_controller.jl")

end

