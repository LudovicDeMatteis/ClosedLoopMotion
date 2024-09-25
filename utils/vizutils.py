'''
https://github.com/MeMory-of-MOtion/summer-school/blob/master/tutorials/pinocchio/vizutils.py
'''

import meshcat
import numpy as np
import pinocchio as pin

# Meshcat utils

def meshcat_material(r, g, b, a):
    import meshcat

    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(r * 255) * 256 ** 2 + int(g * 255) * 256 + int(b * 255)
    material.opacity = a
    return material


def meshcat_transform(x, y, z, q, u, a, t):
    return np.array(pin.XYZQUATToSE3([x, y, z, q, u, a, t]))


# Gepetto/meshcat abstraction

def addViewerBox(viz, name, sizex, sizey, sizez, rgba):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        viz.viewer[name].set_object(meshcat.geometry.Box([sizex, sizey, sizez]),
                                    meshcat_material(*rgba))
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.addBox(name, sizex, sizey, sizez, rgba)
    else:
        raise AttributeError("Viewer %s is not supported." % viz.__class__)


def addViewerSphere(viz, name, size, rgba):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        viz.viewer[name].set_object(meshcat.geometry.Sphere(size),
                                    meshcat_material(*rgba))
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.addSphere(name, size, rgba)
    else:
        raise AttributeError("Viewer %s is not supported." % viz.__class__)


def applyViewerConfiguration(viz, name, xyzquat):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        viz.viewer[name].set_transform(meshcat_transform(*xyzquat))
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.applyConfiguration(name, xyzquat)
        viz.viewer.gui.refresh()
    else:
        raise AttributeError("Viewer %s is not supported." % viz.__class__)

def visualizeConstraints(viz, model, data, constraint_models, q=None):
    if q is not None:
        pin.framesForwardKinematics(model, data, q)
        viz.display(q)
    for i, c in enumerate(constraint_models):
        if c.name != '':
            name = c.name
        else:
            name = f"c{i}"
        offset = pin.SE3.Identity()
        offset.translation = np.array([0, 0, 0.005])
        box = addViewerBox(viz, "Constraints/"+name+"_1", 0.03, 0.02, 0.01, [1, 0, 0, 0.5])
        applyViewerConfiguration(viz, "Constraints/"+name+"_1", pin.SE3ToXYZQUATtuple(data.oMi[c.joint1_id]*c.joint1_placement.act(offset)))
        box = addViewerBox(viz, "Constraints/"+name+"_2", 0.03, 0.02, 0.01, [0, 1, 0, 0.5])
        applyViewerConfiguration(viz, "Constraints/"+name+"_2", pin.SE3ToXYZQUATtuple(data.oMi[c.joint2_id]*c.joint2_placement.act(offset)))

def addFrames(viz, frames_list, axis_length=0.5, axis_width=5):
    viz.displayFrames(True, frames_list, axis_length=axis_length, axis_width=axis_width)

def visualizeJointsFrames(viz, model, axis_length=0.5, axis_width=5):
    viz_frames = []
    for idJoint, n in enumerate(model.names.tolist()[1:]):
        f = pin.Frame(n, idJoint, pin.SE3.Identity(), pin.OP_FRAME)
        fId = model.addFrame(f)
        viz_frames.append(fId)
    addFrames(viz, viz_frames, axis_length, axis_width)

def visualizeInertias(viz, model, q, alpha=0.1):
    data = model.createData()
    pin.forwardKinematics(model, data, q)
    for idJoint in range(model.njoints):
        I = model.inertias[idJoint]
        oMi = data.oMi[idJoint]
        iMf = pin.SE3.Identity()
        iMf.translation = I.lever
        oMf = oMi.act(iMf)

        size = alpha * np.log(1+I.mass)
        addViewerSphere(viz, f"Inertias/I_{model.names[idJoint]}", size, [1, 0, 0, 0.5])
        applyViewerConfiguration(viz, f"Inertias/I_{model.names[idJoint]}", pin.SE3ToXYZQUAT(oMf))
    
def traj_cam_linear(start, end, T):
    traj = [np.linspace(start[i], end[i], T) for i in range(3)]
    traj = np.array(traj).T
    return traj

def record_trajectory(viz, start_pos, qs, path="/tmp/video.mp4", DT=0.015, fixed_cam=False, end_pos=None):
    import imageio
    if end_pos is None and not fixed_cam:
        base_pos_diff = qs[-1, :3] - qs[0, :3]
        end_pos = start_pos + base_pos_diff
    if not fixed_cam:
        traj = traj_cam_linear(start_pos, end_pos, qs.shape[0])
    images = []
    for t, q in enumerate(qs):
        viz.display(q)
        if not fixed_cam:
            viz.setCameraPosition(traj[t])
        images.append(viz.viewer.get_image())
    imageio.mimsave(path, images, fps=1//DT)

def animation(path_open, path_closed, DT=0.015, save_path="animation.mp4"):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    coms_open = np.load(path_open)
    coms_closed = np.load(path_closed)
    T = coms_open.shape[0]
    com_min = min(coms_open.min(), coms_closed.min()) * 0.95
    com_max = max(coms_open.max(), coms_closed.max()) *1.05

    fig, ax = plt.subplots()
    t = np.linspace(0, T*DT, T)
    line_closed = ax.plot(t[0], coms_closed[0], label="Complete model")
    line_open = ax.plot(t[0], coms_open[0], label="Reduced model")
    ax.set(xlim=(0, T*DT), ylim=(com_min, com_max), xlabel="Time [s]", ylabel="CoM height [m]")
    ax.legend()

    def update(frame):
        line_closed[0].set_data(t[:frame], coms_closed[:frame])
        line_open[0].set_data(t[:frame], coms_open[:frame])
        return line_closed, line_open

    ani = animation.FuncAnimation(fig, func=update, frames=T, interval=DT*1000)
    FFwriter = animation.FFMpegWriter(fps=1/DT)
    ani.save(save_path, writer = FFwriter)
    # plt.show()
