import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt

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
com_weight_list = [
    0,
    100,
    200,
    300,
    400,
    500,
    750,
    1000,
    1250,
    1500,
    2000,
    3000,
    4000,
    5000,
    10000,
]

for w in com_weight_list:
    file_complete = np.load(
        f"/tmp/walk_com/walk_{w}_battobot_closed.npy",
        allow_pickle=True,
    )[()]
    file_simplified = np.load(
        f"/tmp/walk_com/walk_{w}_battobot_open_closed.npy",
        allow_pickle=True,
    )[()]
    files[w] = {"simplified": file_simplified, "complete": file_complete}

mean_controls = {"simplified": [], "complete": []}
max_controls = {"simplified": [], "complete": []}
min_controls = {"simplified": [], "complete": []}
com_elevation = {"simplified": [], "complete": []}
data = robot.model.createData()
Tmin = 50
Tmax = 200
q0 = robot.model.referenceConfigurations["half_sitting"]
pin.centerOfMass(robot.model, data, q0)
com0 = data.com[0].copy()[2]
for w in com_weight_list:
    file_complete = files[w]["complete"]
    file_simplified = files[w]["simplified"]
    us_complete = np.abs(file_complete["us"][Tmin:Tmax, 5])
    mean_controls["complete"].append(np.mean(us_complete, axis=0))
    max_controls["complete"].append(np.max(us_complete, axis=0))
    min_controls["complete"].append(np.min(us_complete, axis=0))
    us_simplified = np.abs(file_simplified["us"][Tmin:Tmax, 5])
    mean_controls["simplified"].append(np.mean(us_simplified, axis=0))
    max_controls["simplified"].append(np.max(us_simplified, axis=0))
    min_controls["simplified"].append(np.min(us_simplified, axis=0))

    qs_complete = file_complete["xs"][Tmin:Tmax, : robot.model.nq]
    qs_simplified = file_simplified["xs"][Tmin:Tmax, : robot.model.nq]
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
    com_elevation["simplified"].append(com_simplified)


# Plots to compare the two models CoM variations on the same trajectory
figsize = (15, 8)
fig, (ax, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
ax.plot(
    com_weight_list,
    mean_controls["simplified"],
    linestyle="-.",
    color="tab:blue",
)
ax.plot(
    com_weight_list,
    mean_controls["complete"],
    linestyle="--",
    color="tab:red",
)
ax.set_ylabel("Mean Control Values")
ax2.plot(
    com_weight_list,
    com_elevation["simplified"],
    linestyle="-.",
    color="tab:blue",
)
ax2.plot(
    com_weight_list,
    com_elevation["complete"],
    linestyle="--",
    color="tab:red",
)
ax2.axhline(1, ls="--", color="black", alpha=0.5)
ax2.set_ylabel("Mean CoM elevation")
ax2.set_xlabel("CoM elevation cost weight")
fig.legend(
    ["Approximate Serial Model", "Closed-kinematics Model"],
    loc="upper center",
    bbox_to_anchor=(0.5, 0.98),
    ncol=3,
)
plt.show()
