import numpy as np
import pinocchio as pin
import matplotlib.pyplot as plt

# LOCAL IMPORTS
import loaders

SMALL_SIZE = 8
MEDIUM_SIZE = 12
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

# Plots to compare the two models CoM variations on the same trajectory
figsize = (15, 8)
fig, axs = plt.subplots(3, 1, sharex=True, figsize=figsize)
file_complete = np.load("/tmp/com_0/battobot_closed.npy", allow_pickle=True)[()]
file_simplified = np.load("/tmp/com_0/battobot_open_closed.npy", allow_pickle=True)[()]
xs_complete = file_complete["xs"]
xs_simplified = file_simplified["xs"]
com_complete = []
com_simplified = []
data = robot.model.createData()
for x in xs_complete:
    pin.centerOfMass(robot.model, data, x[: robot.model.nq])
    com_complete.append(data.com[0].copy())
for x in xs_simplified:
    pin.centerOfMass(robot.model, data, x[: robot.model.nq])
    com_simplified.append(data.com[0].copy())
com_complete = np.array(com_complete)
com_simplified = np.array(com_simplified)
axs[0].plot(com_complete[:, 0], label="Complete")
axs[0].plot(com_simplified[:, 0], label="Simplified", linestyle="-.")
axs[0].axhline(com_complete[0, 0], linestyle="--", color="black")
axs[0].set_title("X Axis")
axs[1].plot(com_complete[:, 1], label="Complete")
axs[1].plot(com_simplified[:, 1], label="Simplified", linestyle="-.")
axs[1].axhline(com_complete[0, 1], linestyle="--", color="black")
axs[1].set_title("Y Axis")
axs[1].set_ylabel("Center of Mass Position [m]")
axs[2].plot(com_complete[:, 2], label="Complete")
axs[2].plot(com_simplified[:, 2], label="Simplified", linestyle="-.")
axs[2].axhline(com_complete[0, 2], linestyle="--", color="black")
axs[2].set_title("Z Axis")
axs[2].set_xlabel("Time step")
fig.legend(
    ["Complete", "Simplified", "Initial CoM position"], loc="upper center", ncol=3
)
# plt.tight_layout()
plt.show()

# Plots on the COM penalization weight
Tmin = 50
Tmax = 200
fig, axs = plt.subplots(2, 3, sharex=True, sharey="row", figsize=figsize)
weights = [0, 1250, 2500]
for i, com_weight in enumerate(weights):
    file_complete = np.load(
        f"/tmp/com_{com_weight}//battobot_closed.npy", allow_pickle=True
    )[()]
    file_simplified = np.load(
        f"/tmp/com_{com_weight}/battobot_open_closed.npy", allow_pickle=True
    )[()]

    # Extract the controls ans states
    x0 = file_complete["xs"][0]
    us_complete = file_complete["us"][Tmin:Tmax]
    xs_complete = file_complete["xs"][Tmin:Tmax]
    us_simplified = file_simplified["us"][Tmin:Tmax]
    xs_simplified = file_simplified["xs"][Tmin:Tmax]

    # Compute the CoM position at each time step
    com_complete = []
    com_simplified = []
    data = robot.model.createData()
    pin.centerOfMass(robot.model, data, x0[: robot.model.nq])
    com0 = data.com[0][2]
    for x in xs_complete:
        pin.centerOfMass(robot.model, data, x[: robot.model.nq])
        com_complete.append(data.com[0][2].copy())
    for x in xs_simplified:
        pin.centerOfMass(robot.model, data, x[: robot.model.nq])
        com_simplified.append(data.com[0][2].copy())

    # Plot the two above in subfigures
    axs[0, i].plot(com_complete, label="Complete")
    axs[0, i].plot(com_simplified, label="Simplified", linestyle="-.")
    # Add hline for initial CoM position
    axs[0, i].axhline(com0, color="black", linestyle="--")
    axs[1, i].plot(us_complete[:, 5], label="Complete")
    axs[1, i].plot(us_simplified[:, 5], label="Simplified", linestyle="-.")

axs[0, 0].set_title("CoM Penalization Weight = 0")
axs[0, 1].set_title("CoM Penalization Weight = 1250")
axs[0, 2].set_title("CoM Penalization Weight = 2500")
axs[1, 1].set_xlabel("Time step")
axs[0, 0].set_ylabel("Center of Mass Z position")
axs[1, 0].set_ylabel("Left Knee Motor Control")
# Add grid
for ax in axs.flatten():
    ax.grid()
# Add a common legend above the graphs
fig.legend(
    ["Complete", "Simplified", "Initial CoM position"],
    loc="upper center",
    bbox_to_anchor=(0.5, 0.98),
    ncol=3,
)
# plt.tight_layout()
plt.show()
