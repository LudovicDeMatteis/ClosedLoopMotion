import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt
from toolbox_parallel_robots.constraints import constraintsResidual

# LOCAL IMPORTS
import loaders

SMALL_SIZE = 12
MEDIUM_SIZE = 16
BIGGER_SIZE = 20

plt.rc("font", size=SMALL_SIZE)  # controls default text sizes
plt.rc("axes", titlesize=MEDIUM_SIZE)  # fontsize of the axes title
plt.rc("axes", labelsize=BIGGER_SIZE)  # fontsize of the x and y labels
plt.rc("xtick", labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
plt.rc("ytick", labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
plt.rc("legend", fontsize=BIGGER_SIZE)  # legend fontsize
plt.rc("figure", titlesize=BIGGER_SIZE)  # fontsize of the figure title

# Load the robot
robot = loaders.battobot_closed()
q0 = robot.model.referenceConfigurations["half_sitting"]

files = {}
vcom_list = [
    0,
    0.1,
    0.2,
    0.3,
    0.4,
    0.5,
    0.6,
    0.7,
    0.8,
    0.9,
    1.0,
    1.1,
    1.2,
]
for vel in vcom_list:
    file_complete = np.load(
        f"/tmp/walk_vel/walk_{int(10 * vel)}_battobot_closed.npy",
        allow_pickle=True,
    )[()]
    file_simplified = np.load(
        f"/tmp/walk_vel/walk_{int(10 * vel)}_battobot_open_closed.npy",
        allow_pickle=True,
    )[()]
    files[vel] = {"simplified": file_simplified, "complete": file_complete}

mean_controls = {"simplified": [], "complete": []}
max_controls = {"simplified": [], "complete": []}
min_controls = {"simplified": [], "complete": []}
com_elevation = {"simplified": [], "complete": []}
data = robot.model.createData()
cm = robot.loop_constraints_models
cd = [c.createData() for c in cm]
Tmin = 10
Tmax = 230
q0 = robot.model.referenceConfigurations["half_sitting"]
pin.centerOfMass(robot.model, data, q0)
com0 = data.com[0].copy()[2]
vcom_list_simplified = []
for vel in vcom_list:
    qs_complete = file_complete["xs"][Tmin:Tmax, : robot.model.nq]
    qs_simplified = file_simplified["xs"][Tmin:Tmax, : robot.model.nq]
    max_cstr = np.max(
        [max(constraintsResidual(robot.model, data, cm, cd, q)) for q in qs_simplified]
    )

    file_complete = files[vel]["complete"]
    file_simplified = files[vel]["simplified"]
    us_complete = np.abs(file_complete["us"][Tmin:Tmax, 5])
    mean_controls["complete"].append(np.mean(us_complete, axis=0))
    max_controls["complete"].append(np.max(us_complete, axis=0))
    min_controls["complete"].append(np.min(us_complete, axis=0))
    us_simplified = np.abs(file_simplified["us"][Tmin:Tmax, 5])
    print(vel, max_cstr)
    if max_cstr < 1:
        mean_controls["simplified"].append(np.mean(us_simplified, axis=0))
        max_controls["simplified"].append(np.max(us_simplified, axis=0))
        min_controls["simplified"].append(np.min(us_simplified, axis=0))
        vcom_list_simplified.append(vel)

    com_complete = []
    com_simplified = []
    for q in qs_complete:
        pin.centerOfMass(robot.model, data, q)
        com_complete.append(data.com[0].copy()[2] / com0)
    for q in qs_simplified:
        pin.centerOfMass(robot.model, data, q)
        com_simplified.append(data.com[0].copy()[2] / com0)
    com_complete = np.mean(np.array(com_complete))
    com_simplified = np.mean(np.array(com_simplified))
    com_elevation["complete"].append(com_complete)
    if max_cstr < 1:
        com_elevation["simplified"].append(com_simplified)


# Plots to compare the two models CoM variations on the same trajectory
plot_list = [max_controls["simplified"][0]] + [
    (x + y) / 2
    for x, y in zip(max_controls["simplified"][:-1], max_controls["simplified"][1:])
]
figsize = (15, 8)
fig, (ax, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
ax.plot(
    vcom_list_simplified,
    plot_list,
    # max_controls["simplified"],
    linestyle="-.",
    color="tab:blue",
)
ax.plot(
    vcom_list,
    max_controls["complete"],
    linestyle="--",
    color="tab:red",
)
ax.set_ylabel("Max Control Values")
ax2.plot(
    vcom_list_simplified,
    com_elevation["simplified"],
    linestyle="-.",
    color="tab:blue",
)
ax2.plot(
    vcom_list,
    com_elevation["complete"],
    linestyle="--",
    color="tab:red",
)
ax2.axhline(1, ls="--", color="black", alpha=0.5)
ax2.set_ylabel("Mean CoM elevation")
ax2.set_xlabel("CoM Velocity Command [m/s]")
fig.legend(
    ["Approximate Serial Model", "Closed-kinematics Model"],
    loc="upper center",
    bbox_to_anchor=(0.5, 0.98),
    ncol=3,
)
plt.show()
