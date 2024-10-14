import pinocchio as pin 
import numpy as np
import example_robot_data as robex

robot = robex.load("talos_full")
model = robot.model
data = model.createData()
q0 = robot.q0
q = pin.randomConfiguration(robot.model)
v = np.random.rand(robot.model.nv)
a = np.random.rand(robot.model.nv)
u = np.random.rand(robot.model.nv)

frame1 = model.getFrameId("leg_right_6_joint")
frame2 = model.getFrameId("leg_left_6_joint")

joint1_id = model.frames[frame1].parentJoint
joint1_placement = model.frames[frame1].placement
joint2_id = model.frames[frame2].parentJoint
joint2_placement = model.frames[frame2].placement

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

## Compare for 6D contact
def get_6d_contact_model(model):
    contact_model6d = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_6D,
        model,
        joint1_id,
        joint1_placement,
        joint2_id,
        joint2_placement,
        pin.ReferenceFrame.LOCAL,
    )
    return contact_model6d

def get_6d_acceleration(model, data, q, v, u, contact_model6d, contact_data6d):
    pin.initConstraintDynamics(model, data, [contact_model6d])
    a = pin.constraintDynamics(model, data, q, v, u, [contact_model6d], [contact_data6d])
    return a

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

def get_dpos_dq(model, data, q):
    pos = get_6d_position_error(model, data, q)
    oM1 = data.oMf[frame1]
    oM2 = data.oMf[frame2]

    pin.computeJointJacobians(model, data, q)

    j1Jj1 = pin.getJointJacobian(model, data, joint1_id, pin.ReferenceFrame.LOCAL)
    j2Jj2 = pin.getJointJacobian(model, data, joint2_id, pin.ReferenceFrame.LOCAL)

    f1Jf1 = joint1_placement.toActionMatrixInverse() @ j1Jj1
    f2Jf2 = joint2_placement.toActionMatrixInverse() @ j2Jj2

    f1Mf2_log6 = pin.Jlog6(oM1.actInv(oM2))
    dpos_dq = (f1Mf2_log6 @ (-oM2.toActionMatrixInverse() @ oM1.toActionMatrix() @ f1Jf1 + f2Jf2))

    return dpos_dq

def get_dpos_dq_fd(model, data, q):
    eps = 1e-7
    dpos_dq = np.zeros((6, model.nv))
    q_eps = np.zeros(model.nv)
    pos0 = get_6d_position_error(model, data, q).copy()
    for i in range(model.nv):
        q_eps[i] = eps
        q1 = pin.integrate(model, q, q_eps)
        pos1 = get_6d_position_error(model, data, q1)
        dpos_dq[:, i] = (pos1 - pos0) / eps
        q_eps[i] = 0
    return dpos_dq

contact_model6d = get_6d_contact_model(model)
contact_data6d = contact_model6d.createData()
a_6d = get_6d_acceleration(model, data, q, v, u, contact_model6d, contact_data6d)
#
pos_error_6d = get_6d_position_error(model, data, q)
dpos_dq = get_dpos_dq(model, data, q)
dpos_dq_fd = get_dpos_dq_fd(model, data, q)
#
vel_error_6d = get_6d_velocity_error(model, data, q, v)
#
acc_error_6d = get_6d_acceleration_error(model, data, q, v, a)

acc_error_6d_acc0 = get_6d_acceleration_error(model, data, q, v, a*0)

np.set_printoptions(precision=3, suppress=True, linewidth=200, threshold=200)

print("CONTACT 6D")
print("Position error :")
print(pos_error_6d)
print("dpos_dq linear\n", dpos_dq[:3, :])
print("dpos_dq_fd linear\n", dpos_dq_fd[:3, :])
print("dpos_dq angular\n", dpos_dq[3:, :])
print("dpos_dq_fd angular\n", dpos_dq_fd[3:, :])
print()
print("Velocity error :")
print(vel_error_6d)
print("Acceleration error :")
print(acc_error_6d)