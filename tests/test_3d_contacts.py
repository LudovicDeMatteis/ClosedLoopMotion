import pinocchio as pin 
import numpy as np
import example_robot_data as robex

TEST_6D_CONTACT = True
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

def get_3d_velocity_error_fd(model, data, q, v, dt):
    pos_error1 = get_3d_position_error(model, data, q).copy()
    pos_error2 = get_3d_position_error(model, data, pin.integrate(model, q, v * dt)).copy()
    vel_error = (pos_error2 - pos_error1) / dt
    return vel_error

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

def get_3d_acceleration_error_fd(model, data, q, v, a, dt):
    vel_error1 = get_3d_velocity_error(model, data, q, v).copy()
    vel_error2 = get_3d_velocity_error(model, data, pin.integrate(model, q, v * dt + a*dt**2), v+a*dt).copy()
    acc_error = (vel_error2 - vel_error1) / dt
    return acc_error

contact_model3d = get_3d_contact_model(model)
contact_data3d = contact_model3d.createData()
a_3d = get_3d_acceleration(model, data, q, v, u, contact_model3d, contact_data3d)
#
pos_error_3d = get_3d_position_error(model, data, q)
#
vel_error_3d = get_3d_velocity_error(model, data, q, v)
vel_error_3d_fd = get_3d_velocity_error_fd(model, data, q, v, 1e-6)
#
acc_error_3d = get_3d_acceleration_error(model, data, q, v, a)
acc_error_3d_fd = get_3d_acceleration_error_fd(model, data, q, v, a, 1e-6)

acc_error_3d_acc0 = get_3d_acceleration_error(model, data, q, v, a*0)

print("CONTACT 3D")
print("%%%%%%%%%%%% Position error: %%%%%%%%%%%%")
print("Pinocchio: ", contact_data3d.contact_placement_error.linear)
print("Implem", pos_error_3d)
print("%%%%%%%%%%%% Velocity error: %%%%%%%%%%%%")
# print("Pinocchio: ", contact_data3d.contact_velocity_error.linear)
print("Implem: ", vel_error_3d)
print("FD on position implem: ", vel_error_3d_fd)
print("%%%%%%%%%%%% Acceleration error: %%%%%%%%%%%%")
# print("Pinocchio: ", (contact_data3d.contact1_acceleration_drift - contact_data3d.contact2_acceleration_drift).linear)
# print("Implem - a=0: ", acc_error_3d_acc0)
print("Implem: ", acc_error_3d)
print("FD on velocity implem: ", acc_error_3d_fd)
pairs = [
    [contact_data3d.contact_placement_error, pos_error_3d], 
    [contact_data3d.contact_velocity_error, vel_error_3d], 
    [contact_data3d.contact1_acceleration_drift - contact_data3d.contact2_acceleration_drift, acc_error_3d_acc0],
    [acc_error_3d, acc_error_3d_fd]
]
# for i, pair in enumerate(pairs):
    # assert np.allclose(pair[0], pair[1], atol=1e-6), f"Pair {i} is not close"


