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
list_heights = [1.0, 0.95, 0.9, 0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 1.05, 1.1, 1.151]
list_heights = np.sort(list_heights)

for squat_height in list_heights:
    file_complete = np.load(
        f"/tmp/squat/squat_{int(squat_height * 100)}_battobot_closed.npy",
        allow_pickle=True,
    )[()]
    file_simplified = np.load(
        f"/tmp/squat/squat_{int(squat_height * 100)}_battobot_open_closed.npy",
        allow_pickle=True,
    )[()]
    files[squat_height] = {"simplified": file_simplified, "complete": file_complete}

mean_controls = {"simplified": [], "complete": []}
max_controls = {"simplified": [], "complete": []}
min_controls = {"simplified": [], "complete": []}
for squat_height in list_heights:
    file_complete = files[squat_height]["complete"]
    file_simplified = files[squat_height]["simplified"]
    us_complete = np.abs(file_complete["us"][:, 5])
    mean_controls["complete"].append(np.mean(us_complete, axis=0))
    max_controls["complete"].append(np.max(us_complete, axis=0))
    min_controls["complete"].append(np.min(us_complete, axis=0))
    if squat_height > 0.7:
        us_simplified = np.abs(file_simplified["us"][:, 5])
        mean_controls["simplified"].append(np.mean(us_simplified, axis=0))
        max_controls["simplified"].append(np.max(us_simplified, axis=0))
        min_controls["simplified"].append(np.min(us_simplified, axis=0))
    else:
        us_simplified = np.zeros_like(us_complete)

# Plots to compare the two models CoM variations on the same trajectory
filter = [True if squat_height > 0.7 else False for squat_height in list_heights]
figsize = (15, 8)
fig, (ax, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
ax.plot(
    list_heights[filter],
    mean_controls["simplified"],
    label="Approximate Serial Model",
    linestyle="-.",
    color="tab:blue",
)
ax.plot(
    list_heights,
    mean_controls["complete"],
    label="Closed-kinematics Model",
    linestyle="--",
    color="tab:red",
)
ax.axvline(x=1.0, color="k", ls="--", alpha=0.5, label="Initial CoM height")
ax.set_ylabel("Mean Control Values")
ax2.plot(
    list_heights[filter],
    max_controls["simplified"],
    label="Simplified",
    linestyle="-.",
    color="tab:blue",
)
ax2.plot(
    list_heights,
    max_controls["complete"],
    label="Complete",
    linestyle="--",
    color="tab:red",
)
ax2.axvline(x=1.0, color="k", ls="--", alpha=0.5, label="Initial CoM height")
ax2.set_ylabel("Max Control Values")
ax2.set_xlabel("Target CoM Height - Relative to Initial Height")
fig.legend(
    ["Approximate Serial Model", "Closed-kinematics Model"],
    loc="upper center",
    bbox_to_anchor=(0.5, 0.98),
    ncol=3,
)
plt.show()
