import pinocchio as pin 
import crocoddyl
import numpy as np
import example_robot_data as robex

TEST_6D_CONTACT = True
TEST_3D_CONTACT = True

robot = robex.load("talos_full")
model = robot.model
q0 = robot.q0
q = pin.randomConfiguration(robot.model)
v = np.random.rand(robot.model.nv)
a = np.random.rand(robot.model.nv)
u = np.random.rand(robot.model.nv)

frame1 = model.getFrameId("leg_right_6_joint")

joint1_id = model.frames[frame1].parentJoint
joint1_placement = model.frames[frame1].placement
joint2_id = 0
joint2_placement = pin.SE3.Identity()

frame2 = model.addFrame(
    pin.Frame(
        "frame2",
        joint2_id,
        joint2_placement,
        pin.FrameType.OP_FRAME,
    )
)
data = model.createData()
state = crocoddyl.StateMultibody(model)

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

## Compare for 6D contact
def get_6d_contact_model(model):
    contact = crocoddyl.ContactModel6D(
        state,
        frame1, 
        pin.SE3.Identity(),
        pin.LOCAL,
        12,
        np.array([1e-6, 1e-6])
    )
    return contact

def get_6d_acceleration(model, data, q, v, contact_model6d, contact_data6d):
    pin.computeAllTerms(model, contact_data6d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data6d.pinocchio)
    x = np.concatenate([q, v])
    contact_model6d.calc(contact_data6d, x)
    return contact_data6d.a0

def get_6d_velocity(model, data, q, v, contact_model6d, contact_data6d):
    pin.computeAllTerms(model, contact_data6d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data6d.pinocchio)
    x = np.concatenate([q, v])
    contact_model6d.calc(contact_data6d, x)
    return contact_data6d.v

def get_6d_position(model, data, q, contact_model6d, contact_data6d):
    pin.computeAllTerms(model, contact_data6d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data6d.pinocchio)
    x = np.concatenate([q, v])
    contact_model6d.calc(contact_data6d, x)
    return pin.log6(contact_model6d.reference.actInv(contact_data6d.rMf)).copy()

def get_6d_position_error(model, data, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    oMc1 = data.oMf[frame1]
    oMc2 = data.oMf[frame2]
    c1Mc2 = oMc1.actInv(oMc2)
    position_error6d = -pin.log6(c1Mc2)
    return position_error6d

def get_6d_velocity_error(model, data, q, v):
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Xc2 = c1Mc2.toActionMatrix()
    c1vc2 = c1Mc2.act(c2vc2)
    velocity_error6d = (
        c1vc1
        - c1vc2
    )
    return velocity_error6d

def get_6d_acceleration_error(model, data, q, v, a):
    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1ac1 = pin.getFrameAcceleration(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2ac2 = pin.getFrameAcceleration(model, data, frame2, pin.ReferenceFrame.LOCAL)

    oMc1 = data.oMf[frame1]
    oMc2 = data.oMf[frame2]
    c1Mc2 = oMc1.actInv(oMc2)
    c1Xc2 = c1Mc2.toActionMatrix()
    c1vc2 = c1Mc2.act(c2vc2)
    c1ac2 = c1Mc2.act(c2ac2)

    acc_error6d = (
        c1ac1 
        - c1ac2 
        + c1vc1.cross(c1vc2)
    )
    return acc_error6d

if TEST_6D_CONTACT:
    contact_model6d = get_6d_contact_model(model)
    contact_data6d = contact_model6d.createData(model.createData())
    #
    croc_pos_error_6d = get_6d_position(model, data, q, contact_model6d, contact_data6d)
    pos_error_6d = get_6d_position_error(model, data, q)
    #
    croc_vel_error_6d = get_6d_velocity(model, data, q, v, contact_model6d, contact_data6d)
    vel_error_6d = get_6d_velocity_error(model, data, q, v)
    #
    croc_acc_error_6d = get_6d_acceleration(model, data, q, v, contact_model6d, contact_data6d)
    acc_error_6d = get_6d_acceleration_error(model, data, q, v, v*0)

    print("CONTACT 6D")
    print("Position error :")
    print(croc_pos_error_6d)
    print(pos_error_6d)
    print("Velocity error :")
    print(croc_vel_error_6d)
    print(vel_error_6d)
    print("Acceleration error :")
    print(croc_acc_error_6d)
    print(acc_error_6d)

## Compare for 3D contact
def get_3d_contact_model(model):
    contact = crocoddyl.ContactModel6D(
        state,
        frame1, 
        pin.SE3.Identity(),
        pin.LOCAL,
        # 12,
        np.array([1e-6, 1e-6])
    )
    return contact

def get_3d_acceleration(model, data, q, v, contact_model3d, contact_data3d):
    pin.computeAllTerms(model, contact_data3d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data3d.pinocchio)
    x = np.concatenate([q, v])
    contact_model6d.calc(contact_data3d, x)
    return contact_data3d.a0[:3]

def get_3d_velocity(model, data, q, v, contact_model3d, contact_data3d):
    pin.computeAllTerms(model, contact_data3d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data3d.pinocchio)
    x = np.concatenate([q, v])
    contact_model3d.calc(contact_data3d, x)
    return contact_data3d.v.linear

def get_3d_position(model, data, q, contact_model3d, contact_data3d):
    pin.computeAllTerms(model, contact_data3d.pinocchio, q, v)
    pin.updateFramePlacements(model, contact_data3d.pinocchio)
    x = np.concatenate([q, v])
    contact_model3d.calc(contact_data3d, x)
    dp = contact_data3d.pinocchio.oMf[contact_model3d.id].translation - contact_model3d.reference.translation
    dp_local = contact_data3d.pinocchio.oMf[contact_model3d.id].rotation.transpose() @ dp
    return dp_local

def get_3d_position_error(model, data, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    oMc1 = data.oMf[frame1]
    oMc2 = data.oMf[frame2]
    c1Mc2 = oMc1.actInv(oMc2)
    position_error3d = -c1Mc2.translation
    return position_error3d

def get_3d_velocity_error(model, data, q, v):
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Xc2 = c1Mc2.toActionMatrix()
    c1vc2 = c1Mc2.act(c2vc2)

    oR1 = data.oMf[frame1].rotation
    oR2 = data.oMf[frame2].rotation
    f1Rf2 = oR1.T @ oR2
    # R = np.array([
    #     [0.0, -1.0, 0.0],
    #     [1.0, 0.0, 0.0],
    #     [0.0, 0.0, 1.0]
    # ])
    alpha = c2vc2.linear
    # alpha = np.array([0.1, 0.2, 0.3])
    velocity_error3d = (
        c1vc1.linear
        - c1Mc2.rotation @ c2vc2.linear
        # - f1Rf2 @ alpha
    )
    return velocity_error3d

def get_3d_acceleration_error(model, data, q, v, a):
    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1ac1 = pin.getFrameAcceleration(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2ac2 = pin.getFrameAcceleration(model, data, frame2, pin.ReferenceFrame.LOCAL)

    oMc1 = data.oMf[frame1]
    oMc2 = data.oMf[frame2]
    c1Mc2 = oMc1.actInv(oMc2)
    c1Xc2 = c1Mc2.toActionMatrix()
    c1vc2 = c1Mc2.act(c2vc2)
    c1ac2 = c1Mc2.act(c2ac2)

    oR1 = data.oMf[frame1].rotation
    oR2 = data.oMf[frame2].rotation
    f1Rf2 = oR1.T @ oR2
    ov1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.WORLD)
    ov2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.WORLD)
    # R = np.array([
    #     [0.0, -1.0, 0.0],
    #     [1.0, 0.0, 0.0],
    #     [0.0, 0.0, 1.0]
    # ])
    alpha = c2vc2.linear
    alpha_dot = c2ac2.linear
    # alpha = np.array([0.1, 0.2, 0.3])
    acceleration_error3d = (
        c1ac1.linear
        -f1Rf2 @ alpha_dot
        + oR1.T@(skew(ov1.angular) - skew(ov2.angular)) @ oR2 @ alpha
    )
    return acceleration_error3d

if TEST_3D_CONTACT:
    contact_model3d = get_3d_contact_model(model)
    contact_data3d = contact_model3d.createData(model.createData())
    #
    croc_pos_error_3d = get_3d_position(model, data, q, contact_model3d, contact_data3d)
    pos_error_3d = get_3d_position_error(model, data, q)
    #
    croc_vel_error_3d = get_3d_velocity(model, data, q, v, contact_model3d, contact_data3d)
    vel_error_3d = get_3d_velocity_error(model, data, q, v)
    #
    croc_acc_error_3d = get_3d_acceleration(model, data, q, v, contact_model3d, contact_data3d)
    acc_error_3d = get_3d_acceleration_error(model, data, q, v, v*0)

    print("CONTACT 3D")
    print("Position error :")
    print(croc_pos_error_3d)
    print(pos_error_3d)
    print("Velocity error :")
    print(croc_vel_error_3d)
    print(vel_error_3d)
    print("Acceleration error :")
    print(croc_acc_error_3d)
    print(acc_error_3d)
