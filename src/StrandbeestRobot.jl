module StrandbeestRobot

using RigidBodyDynamics
using RigidBodyDynamics.PDControl
using RigidBodyDynamics.Contact
const rbd = RigidBodyDynamics
using LightXML
using StaticArrays
using Optim
using LinearAlgebra
using RigidBodySim

urdfpath() = joinpath(@__DIR__, "..", "data", "Strandbeest.urdf")

function mechanism(::Type{T} = Float64;
                   floating = true,
                   contact_model = default_contact_model(),
                   add_flat_ground = true) where {T}
    mechanism = parse_urdf(T, urdfpath())
    add_loop_joints!(mechanism, urdfpath())

    if floating
        crossbar = findbody(mechanism, "crossbar")
        basejoint = joint_to_parent(crossbar, mechanism)
        floatingjoint = Joint("floating_base", frame_before(basejoint),
            frame_after(basejoint), QuaternionFloating{Float64}())
        replace_joint!(mechanism, basejoint, floatingjoint)
    end

    add_foot_contacts!(mechanism, contact_model)
    if add_flat_ground
        add_environment_primitive!(mechanism,
            HalfSpace3D(
                Point3D(root_frame(mechanism), 0., 0, 0),
                FreeVector3D(root_frame(mechanism), 0., 0, 1)))
    end


    mechanism
end


"""
Parse the Drake-specific `loop_joint` tags from a URDF and use them
to add non-tree joints to the given mechanism.
"""
function add_loop_joints!(mechanism::Mechanism{T}, urdf::AbstractString) where T
    doc = parse_file(urdf)
    xml_root = LightXML.root(doc)
    xml_loops = get_elements_by_tagname(xml_root, "loop_joint")
    for xml_loop in xml_loops
        name = attribute(xml_loop, "name")
        @assert attribute(xml_loop, "type") == "continuous"
        axis = SVector{3}(rbd.parse_vector(T, find_element(xml_loop, "axis"), "xyz", "1 0 0"))
        joint = Joint(name, Revolute(axis))
        xml_link1 = find_element(xml_loop, "link1")
        body1 = findbody(mechanism, attribute(xml_link1, "link"))
        H1 = Transform3D(frame_before(joint), default_frame(body1),
            rbd.parse_pose(T, xml_link1)...)
        xml_link2 = find_element(xml_loop, "link2")
        body2 = findbody(mechanism, attribute(xml_link2, "link"))
        H2 = Transform3D(frame_after(joint), default_frame(body2),
            rbd.parse_pose(T, xml_link2)...)
        attach!(mechanism, body1, body2, joint,
            joint_pose = H1,
            successor_pose = inv(H2))
    end
end

"""
Return a function which maps a configuration vector
for the given mechanism to the total squared error in
the alignment of the robot's non-tree joints.
"""
function loop_joint_error(mechanism::Mechanism)
    let statecache = StateCache(mechanism), resultcache = DynamicsResultCache(mechanism)
        function(q)
            state = statecache[eltype(q)]
            result = resultcache[eltype(q)]
            set_configuration!(state, q)
            zero_velocity!(state)
            gains = rbd.CustomCollections.ConstDict{JointID}(SE3PDGains(PDGains(1, 0), PDGains(1, 0)))
            rbd.constraint_bias!(result, state; stabilization_gains=gains)
            ret = result.constraintbias ⋅ result.constraintbias
        end
    end
end

default_contact_model() = SoftContactModel(hunt_crossley_hertz(k = 500e3), ViscoelasticCoulombModel(1.0, 20e3, 100.))

function add_foot_contacts!(mechanism::Mechanism, contactmodel=default_contact_model())
    for body in bodies(mechanism)
        if occursin("bars_g_h_i", string(body))
            frame = default_frame(body)
            point = Point3D(frame, 0, 0, 0.49)
            add_contact_point!(body, ContactPoint(
                point, contactmodel))

        end
    end
end

function solve_initial_configuration!(state::MechanismState{T}) where T
    # Find an initial state of the robot which correctly aligns all
    # the loop joints.
    mechanism = state.mechanism

    # We do this before adding the contact points to work around
    # https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/483

    # First, initialize the state with a configuration within the
    # joint limits:
    lb = similar(configuration(state))
    ub = similar(configuration(state))
    for joint in tree_joints(mechanism)
        lb_joint = lb[joint]
        ub_joint = ub[joint]
        if isfloating(joint)
            # use identity transform for floating joint
            tf = one(Transform3D{T}, frame_after(joint), frame_before(joint))
            set_configuration!(state, joint, tf)
            # Optim can't handle equal bounds, so give it a bit of wiggle room
            lb_joint .= configuration(state, joint) .- 1e-3
            ub_joint .= configuration(state, joint) .+ 1e-3
        else
            lb_joint .= rbd.lower.(position_bounds(joint))
            ub_joint .= rbd.upper.(position_bounds(joint))
        end
    end
    q = configuration(state)
    for i in eachindex(q)
        if isfinite(lb[i]) && isfinite(ub[i])
            q[i] = (lb[i] + ub[i]) / 2
        else
            q[i] = clamp(0, lb[i], ub[i])
        end
    end
    setdirty!(state)

    # Use Optim's Fminbox to minimize the loop joint error within
    # the joint limits:
    cost = loop_joint_error(mechanism)
    result = Optim.optimize(cost, lb, ub,
        Vector(configuration(state)),
        Fminbox(LBFGS()), autodiff=:forward)

    # Verify that we've actually closed all the loops
    @assert Optim.minimum(result) < 1e-9

    set_configuration!(state, Optim.minimizer(result))
    normalize_configuration!(state)
    state
end

end
