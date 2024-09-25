import pinocchio as pin
import numpy as np
import sobec
import os

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
    return(contact_constraints_models)


def kangaroo_closed():
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
    ) = load("kangaroo_2legs", free_flyer=True)
    data = model.createData()

    contactIds = [model.getFrameId("foot_left_frame"), model.getFrameId("foot_right_frame")]
    contact_constraints_models = generateContactModels(model, contactIds, pin.neutral(model))
    baseId = model.getFrameId("hanche")

    robot_constraint_datas = [cm.createData() for cm in robot_constraint_models]
    contact_constraints_datas = [cm.createData() for cm in contact_constraints_models]

    MBasePlacement = pin.SE3.Identity()
    MBasePlacement.translation = np.array([0, 0, 0.800])
    MBasePlacement.translation[1] = -(contact_constraints_models[0].joint1_placement.translation[1] + contact_constraints_models[1].joint1_placement.translation[1]) / 2
    base_cstr_model = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_6D,
        model,
        model.frames[baseId].parentJoint,
        model.frames[baseId].placement,
        0,  # To the world
        MBasePlacement,
        pin.ReferenceFrame.LOCAL,
        )
    base_cstr_data = base_cstr_model.createData()

    q0 = configurationProjection(
        model,
        data,
        contact_constraints_models + [base_cstr_model] + robot_constraint_models,
        contact_constraints_datas + [base_cstr_data] + robot_constraint_datas,
        q_prec=pin.neutral(model),
    )

    model.referenceConfigurations["half_sitting"] = q0

    model.frames[model.getFrameId("foot_left_frame")].name = "left_foot_frame"
    model.frames[model.getFrameId("foot_right_frame")].name = "right_foot_frame"

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
    return robot

if __name__ == "__main__":
    robot = kangaroo_closed()
    import meshcat
    from pinocchio.visualize import MeshcatVisualizer
    viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    server = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    viz.initViewer(loadModel=True, viewer=server)
    viz.clean()
    viz.loadViewerModel()
    q = robot.model.referenceConfigurations["half_sitting"]
    viz.display(q)