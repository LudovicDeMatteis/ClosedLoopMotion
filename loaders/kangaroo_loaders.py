import pinocchio as pin
import numpy as np
import sobec
from toolbox_parallel_robots.projections import configurationProjection
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


def kangaroo_closed(base_height=0.800):
    # Q0_SHARED = np.load(f"{CWD}/initial_configs_digit/q0_{str(base_height).replace('.', '_')}.npy")
    try:
        from example_parallel_robots.loader_tools import load
        from toolbox_parallel_robots.freeze_joints import freezeJoints
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
    ) = load("kangaroo_2legs", free_flyer=True)
    # Add missing inertias in the model ujoints
    missing_inertia = []
    for i in missing_inertia:
        model.inertias[i].inertia += np.eye(3) * 1e-3

    # Set the initial configuration
    frame_right_id = model.getFrameId("foot_right_frame")
    frame_left_id = model.getFrameId("foot_left_frame")
    model.frames[frame_right_id].name = "foot_frame_right"
    model.frames[frame_left_id].name = "foot_frame_left"
    # Create the constraint models
    contactIds = [frame_right_id, frame_left_id]
    contact_constraints_models = generateContactModels(
        model, contactIds, pin.neutral(model)
    )
    baseId = model.getFrameId("root_joint")

    MBasePlacement = pin.SE3.Identity()
    MBasePlacement.translation = np.array([0, 0, base_height])
    base_cstr_model = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_6D,
        model,
        model.frames[baseId].parentJoint,
        model.frames[baseId].placement,
        0,  # To the world
        MBasePlacement,
        pin.ReferenceFrame.LOCAL,
    )
    contact_constraints_datas = [cm.createData() for cm in contact_constraints_models]
    base_cstr_data = base_cstr_model.createData()
    q_prec = pin.neutral(model)
    q_prec[36] = 0.1
    w = np.ones(model.nv)
    w[36] = 1e3
    q_ref = configurationProjection(
        model,
        model.createData(),
        contact_constraints_models + [base_cstr_model],
        contact_constraints_datas + [base_cstr_data],
        q_prec=q_prec,
        W=np.diag(w),
    )

    robot_constraint_data = [m.createData() for m in robot_constraint_models]
    q0 = configurationProjection(
        model,
        model.createData(),
        robot_constraint_models + contact_constraints_models + [base_cstr_model],
        robot_constraint_data + contact_constraints_datas + [base_cstr_data],
        q_ref,
        W=np.diag(w),
    )
    # q0 = np.load(f"{CWD}/initial_configs_kangaroo/q0_closed.npy")
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


def kangaroo_simplified(base_height=0.800):
    try:
        from example_parallel_robots.loader_tools import load
        from toolbox_parallel_robots.freeze_joints import freezeJoints
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
    ) = load("kangaroo_2legs", free_flyer=True)
    # Add missing inertias in the model ujoints
    missing_inertia = []
    for i in missing_inertia:
        model.inertias[i].inertia += np.eye(3) * 1e-3

    # Set the initial configuration
    frame_right_id = model.getFrameId("foot_right_frame")
    frame_left_id = model.getFrameId("foot_left_frame")
    model.frames[frame_right_id].name = "foot_frame_right"
    model.frames[frame_left_id].name = "foot_frame_left"

    q0 = np.load(f"{CWD}/initial_configs_kangaroo/q0_closed.npy")
    joints_keep_names = [
        "universe",
        "root_joint",
        # Right
        "free_hip_left",
        "free_universal_x_hip_left",
        "free_universal_y_hip_left",
        "free_ankle_universal_x_left",
        "lower_knee_left",
        "motor_knee_left",
        "4rof_knee_left",
        "spherical_actuation_knee_left",  # ?
        "spherical_4rod_left",
        "spherical_shin_left",
        "upper_knee_left",
        "free_universal_y_foot_left",
        # Left
        "free_zhip",
        "free_universal_yhip",
        "lower_hip",
        "free_ankle_universal_x",
        "upper_knee",
        "lower_knee",
        "uneveral_foot1",
        "4rod_knee",
        "spherical_shin_rod1",
        "spherical_4rod",
        "motor_knee",
        "spherical_motor_knee_holder",
    ]

    reduced_models = []

    jointToLockIds = [
        i for (i, n) in enumerate(model.names) if n not in joints_keep_names
    ]
    (
        model,
        _,
        _,
        visual_model,
        collision_model,
    ) = freezeJoints(
        model,
        robot_constraint_models,
        actuation_model,
        visual_model,
        collision_model,
        jointToLockIds,
        q0,
    )
    # TODO : Add the constraint models and the actuation models

    model.referenceConfigurations["half_sitting"] = pin.neutral(model)

    # Create the robot with Sobec wrapper
    robot = sobec.wwt.RobotWrapper(model, contactKey="foot_frame")
    robot.collision_model = collision_model
    robot.visual_model = visual_model
    robot.actuationModel = actuation_model
    robot.loop_constraints_models = robot  # reduced_models
    assert len(robot.contactIds) == 2
    # Add armature (motors inertia)
    armature = np.concatenate((np.zeros(6), np.full(model.nv - 6, 1e-3)))
    robot.model.armature = armature
    return robot


if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer
    from toolbox_parallel_robots.slider_generation import createSlidersInterface

    robot = kangaroo_closed()
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
    # np.save(f"{CWD}/initial_configs_kangaroo/q0_closed.npy", q0)
