using SimpleKiteControllers
using Test

@testset "SimpleKiteControllers.jl" begin
    @testset "test_linearize" begin
        # set the parameters of the parking controller
        pcs = ParkingControllerSettings(dt=0.05)
        pcs.kp_tr=1.05
        pcs.ki_tr=0.012
        # create the parking controller
        pc = ParkingController(pcs)
        # set the desired turn rate
        psi_dot = 0.1
        # set the heading
        psi = 0.0
        # set the elevation angle
        elevation = 0.0
        # set the apparent wind speed
        v_app = 10.0
        # set the depower setting
        ud_prime = 0.5
        # linearize the NDI block
        u_s, ndi_gain = linearize(pc, psi_dot, psi, elevation, v_app; ud_prime)
        @test u_s ≈ 0.41666666666666674
        @test ndi_gain ≈ 4.166666666666667
    end
    

# function test_calc_steering()
#     @testset "test_calc_steering" begin
#         # set the parameters of the parking controller
#         pcs = ParkingControllerSettings(dt=0.05)
#         pcs.kp_tr=1.05
#         pcs.ki_tr=0.012
#         # create the parking controller
#         pc = ParkingController(pcs)
#         # set the heading
#         heading = deg2rad(1.0)
#         elevation = deg2rad(70.0)
#         chi_set = deg2rad(34.0)
#         u_s, ndi_gain, psi_dot, psi_dot_set = calc_steering(pc, heading, chi_set; elevation)
#         @test u_s ≈ 18.135061759003584
#         @test ndi_gain ≈ 2.0833333333333335
#         @test psi_dot ≈ 0.3490658503988659
#         @test psi_dot_set ≈ 8.639379797371932
#     end
#     nothing
# end

# function test_navigate()
#     @testset "test_navigate" begin
#         # set the parameters of the parking controller
#         pcs = ParkingControllerSettings(dt=0.05)
#         pcs.kp=1.05
#         pcs.ki=0.012
#         # create the parking controller
#         pc = ParkingController(pcs)
#         # set the azimuth
#         azimuth = deg2rad(90.0)
#         # set the elevation
#         elevation = deg2rad(70.0)
#         chi_set = navigate(pc, azimuth, elevation)
#         @test chi_set ≈ -deg2rad(34.019878734151234)
#     end
#     nothing
# end

# function test_parking_controller()
#     @testset verbose=true "test_parking_controller" begin
#         test_calc_steering()
#         test_linearize()
#         test_navigate()
#     end
#     nothing
# end
end
