import pinocchio as pin 
import numpy as np
import example_robot_data as robex

TEST_3D_CONTACT = True

robot = robex.load("talos_full")
model = robot.model
data = model.createData()
q0 = robot.q0
q = pin.randomConfiguration(robot.model)
v = np.random.rand(robot.model.nv)
a = np.random.rand(robot.model.nv)
u = np.random.rand(robot.model.nv)

frame1 = model.getFrameId("left_sole_link")
frame2 = model.getFrameId("right_sole_link")
model.frames[frame1].placement = pin.SE3.Random()
model.frames[frame2].placement = pin.SE3.Random()

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

## Compare for 3D contact
def get_3d_contact_model(model):
    contact_model3d = pin.RigidConstraintModel(
        pin.ContactType.CONTACT_3D,
        model,
        joint1_id,
        joint1_placement,
        joint2_id,
        joint2_placement,
        pin.ReferenceFrame.LOCAL,
    )
    return contact_model3d

def get_3d_acceleration(model, data, q, v, u, contact_model3d, contact_data3d):
    pin.initConstraintDynamics(model, data, [contact_model3d])
    a = pin.constraintDynamics(model, data, q, v, u, [contact_model3d], [contact_data3d])
    return a

def get_3d_position_error(model, data, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    oMc1 = data.oMf[frame1]
    oMc2 = data.oMf[frame2]
    oR1 = oMc1.rotation
    o01 = oMc1.translation
    o02 = oMc2.translation

    position_error3d = oR1.T @ (o01 - o02)
    return position_error3d

def get_3d_velocity_error(model, data, q, v):
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])

    velocity_error3d = (
        c1vc1.linear
        - c1Mc2.rotation @ c2vc2.linear
        - skew(c1Mc2.translation) @ c1vc1.angular
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
    c1vc2 = c1Mc2.act(c2vc2)

    oR1 = data.oMf[frame1].rotation
    oR2 = data.oMf[frame2].rotation
    f1Rf2 = oR1.T @ oR2

    pe = get_3d_position_error(model, data, q)
    ve = get_3d_velocity_error(model, data, q, v)

    acceleration_error3d = (
        c1ac1.linear
        -f1Rf2 @ c2ac2.linear
        + (skew(c1vc1.angular) - skew(c1vc2.angular)) @ f1Rf2 @ c2vc2.linear
        + skew(ve) @ c1vc1.angular
        + skew(pe) @ c1ac1.angular
    )
    return acceleration_error3d

def get_dpos_dq(model, data, q):
    # pos = get_3d_position_error(model, data, q)
    oM1 = data.oMf[frame1]
    oM2 = data.oMf[frame2]

    oR1 = oM1.rotation
    o01 = oM1.translation
    o02 = oM2.translation
    opos = o01 - o02

    pin.computeJointJacobians(model, data, q)

    f1Jf1 = pin.getFrameJacobian(model, data, frame1, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    f2Jf2 = pin.getFrameJacobian(model, data, frame2, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    dpos_dq = (
        oR1.T @ skew(opos) @ f1Jf1[3:, :]
        + oR1.T @ (f1Jf1[:3, :] - f2Jf2[:3, :])
    ) 

    return dpos_dq

def get_dpos_dq_fd(model, data, q):
    eps = 1e-7
    dpos_dq = np.zeros((3, model.nv))
    q_eps = np.zeros(model.nv)
    pos0 = get_3d_position_error(model, data, q).copy()
    for i in range(model.nv):
        q_eps[i] = eps
        q1 = pin.integrate(model, q, q_eps)
        pos1 = get_3d_position_error(model, data, q1)
        dpos_dq[:, i] = (pos1 - pos0) / eps
        q_eps[i] = 0
    return dpos_dq

def get_dvel_dq(model, data, q, v):
    pos = get_3d_position_error(model, data, q)
    dpos_dq = get_dpos_dq(model, data, q)
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Rc2 = c1Mc2.rotation
    oR1 = data.oMf[frame1].rotation
    oR2 = data.oMf[frame2].rotation

    pin.computeForwardKinematicsDerivatives(model, data, q, v, v*0)
    pin.computeJointJacobians(model, data, q)
    c1_dvc1_dq, _ = pin.getFrameVelocityDerivatives(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2_dvc2_dq, _ = pin.getFrameVelocityDerivatives(model, data, frame2, pin.ReferenceFrame.LOCAL)
    j1Jj1 = pin.getJointJacobian(model, data, joint1_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    j2Jj2 = pin.getJointJacobian(model, data, joint2_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    dvel_dq = (
        c1_dvc1_dq[:3]
        #
        - c1Rc2 @ c2_dvc2_dq[:3]
        - oR1.T @ skew(oR2 @ c2vc2.linear) @ (j1Jj1[3:, :] - j2Jj2[3:, :])
        #
        + skew(pos) @ c1_dvc1_dq[3:]
        - skew(c1vc1.angular) @ dpos_dq
    )
    return dvel_dq

def get_dvel_dq_fd(model, data, q, v):
    eps = 1e-7
    dvel_dq = np.zeros((3, model.nv))
    q_eps = np.zeros(model.nv)
    vel0 = get_3d_velocity_error(model, data, q, v).copy()
    for i in range(model.nv):
        q_eps[i] = eps
        q1 = pin.integrate(model, q, q_eps)
        vel1 = get_3d_velocity_error(model, data, q1, v)
        dvel_dq[:, i] = (vel1 - vel0) / eps
        q_eps[i] = 0
    return dvel_dq

def get_dvel_dv(model, data, q, v):
    pos = get_3d_position_error(model, data, q)
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Rc2 = c1Mc2.rotation

    pin.computeForwardKinematicsDerivatives(model, data, q, v, v*0)
    pin.computeJointJacobians(model, data, q)
    f1Jf1 = pin.getFrameJacobian(model, data, frame1, pin.ReferenceFrame.LOCAL)
    f2Jf2 = pin.getFrameJacobian(model, data, frame2, pin.ReferenceFrame.LOCAL)

    dvel_dv = (
        f1Jf1[:3, :] - c1Rc2 @ f2Jf2[:3, :] + skew(pos) @ f1Jf1[3:, :]
    )
    return dvel_dv

def get_dvel_dv_fd(model, data, q, v):
    eps = 1e-7
    dvel_dv = np.zeros((3, model.nv))
    v_eps = np.zeros(model.nv)
    vel0 = get_3d_velocity_error(model, data, q, v).copy()
    for i in range(model.nv):
        v_eps[i] = eps
        v1 = v + v_eps
        vel1 = get_3d_velocity_error(model, data, q, v1)
        dvel_dv[:, i] = (vel1 - vel0) / eps
        v_eps[i] = 0
    return dvel_dv

def get_dacc_dq(model, data, q, v, a):
    pos = get_3d_position_error(model, data, q)
    vel = get_3d_velocity_error(model, data, q, v)
    dpos_dq = get_dpos_dq(model, data, q)
    dvel_dq = get_dvel_dq(model, data, q, v)
    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1ac1 = pin.getFrameAcceleration(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2ac2 = pin.getFrameAcceleration(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Rc2 = c1Mc2.rotation
    oR1 = data.oMf[frame1].rotation
    oR2 = data.oMf[frame2].rotation
    c1vc2 = c1Mc2.act(c2vc2)

    pin.computeForwardKinematicsDerivatives(model, data, q, v, a)
    pin.computeJointJacobians(model, data, q)
    c1_dvc1_dq, c1_dac1_dq, c1_dac1_dv, c1_dac1_da = pin.getFrameAccelerationDerivatives(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2_dvc2_dq, c2_dac2_dq, c2_dac2_dv, c2_dac2_da = pin.getFrameAccelerationDerivatives(model, data, frame2, pin.ReferenceFrame.LOCAL)
    j1Jj1 = pin.getJointJacobian(model, data, joint1_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    j2Jj2 = pin.getJointJacobian(model, data, joint2_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    dacc_dq = (
        c1_dac1_dq[:3]
        - (c1Rc2 @ c2_dac2_dq[:3, :] + oR1.T @ skew(oR2 @ c2ac2.linear) @ (j1Jj1[3:, :] - j2Jj2[3:, :]))
        - skew(c1Rc2 @ c2vc2.linear) @ (c1_dvc1_dq[3:] - c1Rc2 @ c2_dvc2_dq[3:] - oR1.T @ skew(oR2 @ c2vc2.angular) @ (j1Jj1[3:, :] - j2Jj2[3:, :]))
        + (skew(c1vc1.angular) - skew(c1vc2.angular)) @ (c1Rc2 @ c2_dvc2_dq[:3] + oR1.T @ skew(oR2 @ c2vc2.linear) @ (j1Jj1[3:, :] - j2Jj2[3:, :]))
        + skew(vel) @ c1_dvc1_dq[3:] - skew(c1vc1.angular) @ dvel_dq
        + skew(pos) @ c1_dac1_dq[3:] - skew(c1ac1.angular) @ dpos_dq
    )
    return dacc_dq

def get_dacc_dq_fd(model, data, q, v, a):
    eps = 1e-7
    dacc_dq = np.zeros((3, model.nv))
    q_eps = np.zeros(model.nv)
    acc0 = get_3d_acceleration_error(model, data, q, v, a).copy()
    for i in range(model.nv):
        q_eps[i] = eps
        q1 = pin.integrate(model, q, q_eps)
        acc1 = get_3d_acceleration_error(model, data, q1, v, a)
        dacc_dq[:, i] = (acc1 - acc0) / eps
        q_eps[i] = 0
    return dacc_dq

def get_dacc_dv(model, data, q, v, a):
    pos = get_3d_position_error(model, data, q)
    vel = get_3d_velocity_error(model, data, q, v)
    pin.forwardKinematics(model, data, q, v, a)
    pin.updateFramePlacements(model, data)
    c1vc1 = pin.getFrameVelocity(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2vc2 = pin.getFrameVelocity(model, data, frame2, pin.ReferenceFrame.LOCAL)
    c1Mc2 = data.oMf[frame1].actInv(data.oMf[frame2])
    c1Rc2 = c1Mc2.rotation
    c1vc2 = c1Mc2.act(c2vc2)

    pin.computeForwardKinematicsDerivatives(model, data, q, v, a)
    pin.computeJointJacobians(model, data, q)
    c1_dvc1_dq, c1_dac1_dq, c1_dac1_dv, c1_dac1_da = pin.getFrameAccelerationDerivatives(model, data, frame1, pin.ReferenceFrame.LOCAL)
    c2_dvc2_dq, c2_dac2_dq, c2_dac2_dv, c2_dac2_da = pin.getFrameAccelerationDerivatives(model, data, frame2, pin.ReferenceFrame.LOCAL)
    f1Jf1 = pin.getFrameJacobian(model, data, frame1, pin.ReferenceFrame.LOCAL)
    f2Jf2 = pin.getFrameJacobian(model, data, frame2, pin.ReferenceFrame.LOCAL)

    dacc_dv = (
        c1_dac1_dv[:3]
        - c1Rc2 @ c2_dac2_dv[:3]
        + (
            (skew(c1vc1.angular) - skew(c1vc2.angular)) @ c1Rc2 @ f2Jf2[:3, :]
            - skew(c1Rc2 @ c2vc2.linear) @ (f1Jf1[3:, :] - c1Rc2 @ f2Jf2[3:, :])
        )
        + skew(vel) @ f1Jf1[3:, :] - skew(c1vc1.angular) @ dvel_dv
        + skew(pos) @ c1_dac1_dv[3:]
    )
    return dacc_dv

def get_dacc_dv_fd(model, data, q, v, a):
    eps = 1e-7
    dacc_dv = np.zeros((3, model.nv))
    v_eps = np.zeros(model.nv)
    acc0 = get_3d_acceleration_error(model, data, q, v, a).copy()
    for i in range(model.nv):
        v_eps[i] = eps
        v1 = v + v_eps
        acc1 = get_3d_acceleration_error(model, data, q, v1, a)
        dacc_dv[:, i] = (acc1 - acc0) / eps
        v_eps[i] = 0
    return dacc_dv

contact_model3d = get_3d_contact_model(model)
contact_data3d = contact_model3d.createData()
a_3d = get_3d_acceleration(model, data, q, v, u, contact_model3d, contact_data3d)
#
pos_error_3d = get_3d_position_error(model, data, q)
dpos_dq = get_dpos_dq(model, data, q)
dpos_dq_fd = get_dpos_dq_fd(model, data, q)
#
vel_error_3d = get_3d_velocity_error(model, data, q, v)
dvel_dq = get_dvel_dq(model, data, q, v)
dvel_dq_fd = get_dvel_dq_fd(model, data, q, v)
dvel_dv = get_dvel_dv(model, data, q, v)
dvel_dv_fd = get_dvel_dv_fd(model, data, q, v)
#
acc_error_3d = get_3d_acceleration_error(model, data, q, v, a)
dacc_dq = get_dacc_dq(model, data, q, v, a)
dacc_dq_fd = get_dacc_dq_fd(model, data, q, v, a)
dacc_dv = get_dacc_dv(model, data, q, v, a)
dacc_dv_fd = get_dacc_dv_fd(model, data, q, v, a)

np.set_printoptions(precision=3, suppress=True, linewidth=200, threshold=200)

print("CONTACT 3D")
print("%%%%%%%%%%%% Position error: %%%%%%%%%%%%")
print("Implem", pos_error_3d)
# print("dpos_dq\n", dpos_dq)
# print("dpos_dq_fd\n", dpos_dq_fd)
assert np.allclose(dpos_dq, dpos_dq_fd, atol=1e-6)
print("%%%%%%%%%%%% Velocity error: %%%%%%%%%%%%")
print("Implem: ", vel_error_3d)
# print("dvel_dq\n", dvel_dq)
# print("dvel_dq_fd\n", dvel_dq_fd)
# print("dvel_dv\n", dvel_dv)
# print("dvel_dv_fd\n", dvel_dv_fd)
assert np.allclose(dvel_dq, dvel_dq_fd, atol=1e-6)
assert np.allclose(dvel_dv, dvel_dv_fd, atol=1e-6)
print("%%%%%%%%%%%% Acceleration error: %%%%%%%%%%%%")
print("Implem: ", acc_error_3d)
# print("dacc_dq\n", dacc_dq)
# print("dacc_dq_fd\n", dacc_dq_fd)
# print("dacc_dv\n", dacc_dv)
# print("dacc_dv_fd\n", dacc_dv_fd)
assert np.allclose(dacc_dq, dacc_dq_fd, atol=1e-6)
assert np.allclose(dacc_dv, dacc_dv_fd, atol=1e-6)

print("All tests passed!")