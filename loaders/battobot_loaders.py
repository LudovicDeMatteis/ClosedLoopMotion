import pinocchio as pin
import numpy as np
import sobec
import os
CWD = os.path.dirname(os.path.abspath(__file__))

def battobot_open(base_height=0.575):
    Q0_SHARED = np.load(f"{CWD}/initial_configs/q0_{str(base_height).replace('.', '_')}.npy")
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
    ) = load("battobot", free_flyer=True)
    # Add missing inertias in the model ujoints
    missing_inertia = [10, 13, 16, 28, 31, 34]
    for i in missing_inertia:
        model.inertias[i].inertia += np.eye(3) * 1e-3
    # Lock the joints
    joints_lock_names = [
        # Right
        "left_spherical_foot_1",
        "left_spherical_foot_2",
        "free_knee_left_Y",
        "free_knee_left_Z",
        "motor_ankle1_left",
        "left_spherical_ankle_1_Y",
        "left_spherical_ankle_1_Z",
        "motor_ankle2_left",
        "left_spherical_ankle_2_Y",
        "left_spherical_ankle_2_Z",
        "motor_knee_left",
        "transmission_knee_left",
        # Right
        "right_spherical_foot_1",
        "right_spherical_foot_2",
        "free_knee_right_Y",
        "free_knee_right_Z",
        "motor_ankle1_right",
        "right_spherical_ankle_1_Y",
        "right_spherical_ankle_1_Z",
        "motor_ankle2_right",
        "right_spherical_ankle_2_Y",
        "right_spherical_ankle_2_Z",
        "motor_knee_right",
        "transmission_knee_right",
    ]
    jointToLockIds = [i for (i, n) in enumerate(model.names) if n in joints_lock_names]
    (
        model,
        _,  # Constraints models and actuation models
        _,  # are changed ad hoc for Digit
        visual_model,
        collision_model,
    ) = freezeJoints(
        model,
        robot_constraint_models,
        actuation_model,
        visual_model,
        collision_model,
        jointToLockIds,
        pin.neutral(model),
    )
    robot_constraint_models = [] # Reset the robot constraint models (empty for open loop)

    model.referenceConfigurations["half_sitting"] = Q0_SHARED
    model.frames[38].name = "foot_frame_right"
    model.frames[76].name = "foot_frame_left"

    # Add joint limits
    model.upperPositionLimit[10] = 0.311
    model.lowerPositionLimit[16] = -0.311
    # Create the robot with Sobec wrapper
    robot = sobec.wwt.RobotWrapper(model, contactKey="foot_frame")
    robot.collision_model = collision_model
    robot.visual_model = visual_model
    assert len(robot.contactIds) == 2
    # Add armature (motors inertia)
    armature = np.concatenate((np.zeros(6), np.full(model.nv - 6, 1e-3)))
    robot.model.armature = armature
    return robot

def battobot_closed(export_joints_ids=False, base_height=0.575):
    Q0_SHARED = np.load(f"{CWD}/initial_configs/q0_{str(base_height).replace('.', '_')}.npy")
    try:
        from example_parallel_robots.loader_tools import load
        from toolbox_parallel_robots.projections import configurationProjection
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
    ) = load("battobot", free_flyer=True)

    missing_inertia = [10, 13, 16, 28, 31, 34]
    for i in missing_inertia:
        model.inertias[i].inertia += np.eye(3) * 1e-3

    joints_lock_names = [
        # Right
        "left_spherical_foot_1",
        "left_spherical_foot_2",
        "free_knee_left_Y",
        "free_knee_left_Z",
        "motor_ankle1_left",
        "left_spherical_ankle_1_Y",
        "left_spherical_ankle_1_Z",
        "motor_ankle2_left",
        "left_spherical_ankle_2_Y",
        "left_spherical_ankle_2_Z",
        "motor_knee_left",
        "transmission_knee_left",
        # Right
        "right_spherical_foot_1",
        "right_spherical_foot_2",
        "free_knee_right_Y",
        "free_knee_right_Z",
        "motor_ankle1_right",
        "right_spherical_ankle_1_Y",
        "right_spherical_ankle_1_Z",
        "motor_ankle2_right",
        "right_spherical_ankle_2_Y",
        "right_spherical_ankle_2_Z",
        "motor_knee_right",
        "transmission_knee_right",
    ]

    LOOP_JOINT_IDS_Q = []
    LOOP_JOINT_IDS_V = []
    for i, name in enumerate(joints_lock_names):
        jId = model.getJointId(name)
        for niq in range(model.joints[jId].nq):
            LOOP_JOINT_IDS_Q.append(model.joints[jId].idx_q + niq)
        for niv in range(model.joints[jId].nv):
            LOOP_JOINT_IDS_V.append(model.joints[jId].idx_v + niv)
    SERIAL_JOINT_IDS_Q = [i for i in range(model.nq) if i not in LOOP_JOINT_IDS_Q]
    SERIAL_JOINT_IDS_V = [i for i in range(model.nv) if i not in LOOP_JOINT_IDS_V]

    q_ref = pin.neutral(model)
    q_ref[SERIAL_JOINT_IDS_Q] = Q0_SHARED
    robot_constraint_datas = [cm.createData() for cm in robot_constraint_models]
    w = np.ones(model.nv)
    w[SERIAL_JOINT_IDS_V] = 1e5
    W = np.diag(w)
    q0 = configurationProjection(
        model,
        model.createData(),
        robot_constraint_models,
        robot_constraint_datas,
        q_ref,
        W,
    )

    model.referenceConfigurations["half_sitting"] = q0

    model.frames[14].name = "foot_frame_right"
    model.frames[58].name = "foot_frame_left"

    # Add joint limits
    model.upperPositionLimit[10] = 0.311
    model.lowerPositionLimit[37] = -0.311

    robot = sobec.wwt.RobotWrapper(model, contactKey="foot_frame", closed_loop=True)
    robot.collision_model = collision_model
    robot.visual_model = visual_model
    robot.actuationModel = actuation_model
    robot.loop_constraints_models = robot_constraint_models
    assert len(robot.contactIds) == 2

    armature = np.zeros(model.nv)
    for i in actuation_model.mot_ids_v:
        armature[i] = 1e-3
    robot.model.armature = armature
    if export_joints_ids:
        return robot, (
            SERIAL_JOINT_IDS_Q,
            SERIAL_JOINT_IDS_V,
            LOOP_JOINT_IDS_Q,
            LOOP_JOINT_IDS_V,
        )
    else:
        return robot