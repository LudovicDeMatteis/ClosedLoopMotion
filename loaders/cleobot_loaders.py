import pinocchio as pin
import numpy as np
import sobec
from toolbox_parallel_robots.projections import configurationProjection
from toolbox_parallel_robots.mounting import closedLoopMountProximal
import os

CWD = os.path.dirname(os.path.abspath(__file__))


def generateContactModels(model, contactIds, q_ref):
    data = model.createData()
    contact_constraints_models = []
    for cId in contactIds:
        pin.framesForwardKinematics(model, data, q_ref)
        floorContactPositionLeft = data.oMf[cId].translation
        floorContactPositionLeft[0] = 0
        floorContactPositionLeft[2] = 0
        MContactPlacement = pin.SE3(
            pin.utils.rotate("x", 0.0), floorContactPositionLeft
        )  # SE3 position of the contact
        footFloorConstraint = pin.RigidConstraintModel(
            pin.ContactType.CONTACT_6D,
            model,
            model.frames[cId].parentJoint,
            model.frames[cId].placement,
            0,  # To the world
            MContactPlacement,
            pin.ReferenceFrame.LOCAL,
        )
        contact_constraints_models.append(footFloorConstraint)
    return contact_constraints_models


def cleobot_closed(base_height=0.800):
    try:
        from example_parallel_robots.loader_tools import load
    except ImportError as e:
        print(e)
        print(
            "Please install the `toolbox_parallel_robots` and `example_parallel_robots` packages to run this model"
        )
        return
    (
        model,
        robot_constraint_models,
        actuation_model,
        visual_model,
        collision_model,
    ) = load("cleobot", free_flyer=True)
    # Add missing inertias in the model ujoints
    missing_inertia = []
    for i in missing_inertia:
        model.inertias[i].inertia += np.eye(3) * 1e-3

    # Set the initial configuration
    frame_right_id = model.getFrameId("right_foot_frame")
    frame_left_id = model.getFrameId("left_foot_frame")
    model.frames[frame_right_id].name = "foot_frame_right"
    model.frames[frame_left_id].name = "foot_frame_left"

    for c in robot_constraint_models:
        c.corrector.Kp[:] = np.ones(6) * 00
        c.corrector.Kd[:] = np.ones(6) * 00
    model.armature[actuation_model.mot_ids_v] = [
        3400 * 8 * 1e-7,
        1477 * 18 * 1e-7,
        1477 * 18 * 1e-7,
        1477 * 18 * 1e-7,
        1477 * 18 * 1e-7,
        1477 * 18 * 1e-7,
    ] * 2
    data = model.createData()
    cdata = [c.createData() for c in robot_constraint_models]
    q = closedLoopMountProximal(model, data, robot_constraint_models, cdata)
    entraxe = 0.11
    foot_id = [model.getFrameId(f) for f in ["foot_frame_right", "foot_frame_left"]]
    Lcontact_frame = []
    for fid in foot_id:
        f = model.frames[fid]
        if "right" in f.name:
            placement = pin.SE3.Identity()
            placement.translation[1] = -entraxe
            placement.rotation = pin.utils.rotate("z", np.deg2rad(-7.5))
            Lcontact_frame.append([f, placement.copy()])
        else:
            placement = pin.SE3.Identity()
            placement.translation[1] = entraxe
            placement.rotation = pin.utils.rotate("z", np.deg2rad(7.5))
            Lcontact_frame.append([f, placement.copy()])
    print([f.name for f in model.frames])
    torso_name = "torso"
    idknee1 = model.getFrameId("collisionknee1")
    idknee2 = model.getFrameId("collisionknee2")
    torso_placement = pin.SE3.Identity()
    torso_placement.translation[2] = 0.53
    torso_placement.translation[0] = 0.0
    id_torso = model.getFrameId(torso_name)
    Lcontact_frame.append([model.frames[id_torso], torso_placement])

    nconstraint_model = []
    for f1, placement in Lcontact_frame[:]:
        nconstraint_model.append(
            pin.RigidConstraintModel(
                pin.ContactType.CONTACT_6D,
                model,
                f1.parentJoint,
                f1.placement,
                0,
                placement,
                pin.ReferenceFrame.LOCAL,
            )
        )

    ncdata = [c.createData() for c in nconstraint_model]
    q0 = np.load(f"{CWD}/initial_configs_cleobot/q0_closed.npy")
    # q = closedLoopMountProximal(
    #     model,
    #     data,
    #     robot_constraint_models + nconstraint_model[:2],
    #     cdata + ncdata[:2],
    #     q_prec=q,
    # )

    # q0 = closedLoopMountProximal(
    #     model,
    #     data,
    #     robot_constraint_models + nconstraint_model[:],
    #     cdata + ncdata[:],
    #     q_prec=q,
    # )

    model.referenceConfigurations["half_sitting"] = q0

    # Create the robot with Sobec wrapper
    robot = sobec.wwt.RobotWrapper(model, contactKey="foot_frame")
    robot.collision_model = collision_model
    robot.visual_model = visual_model
    robot.actuationModel = actuation_model
    robot.loop_constraints_models = robot_constraint_models
    assert len(robot.contactIds) == 2
    # Add armature (motors inertia)
    armature = np.concatenate((np.zeros(6), np.full(model.nv - 6, 1e-3)))
    robot.model.armature = armature
    return robot


if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer
    from toolbox_parallel_robots.slider_generation import createSlidersInterface

    robot = cleobot_closed()
    viz = create_viewer(robot)
    q0 = robot.x0[: robot.model.nq]
    viz.display(q0)
    # createSlidersInterface(
    #     robot.model,
    #     robot.loop_constraints_models,
    #     robot.visual_model,
    #     robot.actuationModel.mot_ids_q,
    #     viz,
    #     q0,
    # )
    np.save(f"{CWD}/initial_configs_cleobot/q0_closed.npy", q0)
