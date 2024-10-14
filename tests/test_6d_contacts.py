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

def get_6d_velocity_error_fd(model, data, q, v, dt):
    pos_error1 = get_6d_position_error(model, data, q).copy()
    pos_error2 = get_6d_position_error(model, data, pin.integrate(model, q, v * dt)).copy()
    vel_error = (pos_error2 - pos_error1) / dt
    return vel_error

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

def get_6d_acceleration_error_fd(model, data, q, v, a, dt):
    vel_error1 = get_6d_velocity_error(model, data, q, v).copy()
    vel_error2 = get_6d_velocity_error(model, data, pin.integrate(model, q, v * dt + a * dt**2), v + a * dt).copy()
    acc_error = (vel_error2 - vel_error1) / dt
    return acc_error

contact_model6d = get_6d_contact_model(model)
contact_data6d = contact_model6d.createData()
a_6d = get_6d_acceleration(model, data, q, v, u, contact_model6d, contact_data6d)
#
pos_error_6d = get_6d_position_error(model, data, q)
#
vel_error_6d = get_6d_velocity_error(model, data, q, v)
vel_error_6d_fd = get_6d_velocity_error_fd(model, data, q, v, 1e-6)
#
acc_error_6d = get_6d_acceleration_error(model, data, q, v, a)
acc_error_6d_fd = get_6d_acceleration_error_fd(model, data, q, v, a, 1e-6)

acc_error_6d_acc0 = get_6d_acceleration_error(model, data, q, v, a*0)

print("CONTACT 6D")
print("Position error :")
print(contact_data6d.contact_placement_error)
print(pos_error_6d)
print("Velocity error :")
print(contact_data6d.contact_velocity_error)
print(vel_error_6d)
print("Velocity error fd :")
print(vel_error_6d_fd)
print("Acceleration error :")
print(contact_data6d.contact1_acceleration_drift - contact_data6d.contact2_acceleration_drift)
print(acc_error_6d_acc0)
print(acc_error_6d)
print("Acceleration error fd :")
print(acc_error_6d_fd)
pairs = [
    [contact_data6d.contact_placement_error, pos_error_6d], 
    [contact_data6d.contact_velocity_error, vel_error_6d], 
    [contact_data6d.contact1_acceleration_drift - contact_data6d.contact2_acceleration_drift, acc_error_6d_acc0],
    [acc_error_6d, acc_error_6d_fd]
]
for i, pair in enumerate(pairs):
    assert np.allclose(pair[0], pair[1], atol=1e-4), f"Pair {i} is not close" 