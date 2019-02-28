using Test
using RigidBodyDynamics
using StrandbeestRobot
using NBInclude

@testset "Initial configuration" begin
    @testset "floating base" begin
        mechanism = StrandbeestRobot.mechanism(floating=true)
        state = MechanismState(mechanism)
        cost = StrandbeestRobot.loop_joint_error(mechanism)
        StrandbeestRobot.solve_initial_configuration!(state)
        for joint in tree_joints(mechanism)
            @test RigidBodyDynamics.is_configuration_normalized(joint, configuration(state, joint); atol=1e-8)
        end
        @test cost(configuration(state)) <= 1e-9
    end

    @testset "fixed base" begin
        mechanism = StrandbeestRobot.mechanism(floating=false)
        state = MechanismState(mechanism)
        cost = StrandbeestRobot.loop_joint_error(mechanism)
        StrandbeestRobot.solve_initial_configuration!(state)
        @test cost(configuration(state)) <= 1e-9
    end
end

@testset "notebooks" begin
    notebook_dir = joinpath(@__DIR__, "..", "notebooks")
    @nbinclude(joinpath(notebook_dir, "Basic simulation.ipynb"); regex = r"^((?!\#NBSKIP).)*$"s)
    @nbinclude(joinpath(notebook_dir, "Passive walking.ipynb"); regex = r"^((?!\#NBSKIP).)*$"s)
    @nbinclude(joinpath(notebook_dir, "Powered walking.ipynb"); regex = r"^((?!\#NBSKIP).)*$"s)
end
