import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import sobec.walk_without_think.plotter
import specific_params
from loaders.loaders_virgile import load_complete_open

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.

import sys
print(sys.argv)

walkParams = specific_params.WalkBattobotParams('open')
if len(sys.argv) > 1:
    WS = bool(sys.argv[1])
    walkParams.saveFile = sys.argv[2]
    walkParams.guessFile = sys.argv[3] if sys.argv[3] != "" else None
    # base_height = round(float(sys.argv[4])*1e-3, 5)
    base_height = 0.575
    comWeight = int(sys.argv[4])
    walkParams.comWeight = comWeight
    walkParams.vcomRef[0] = round(float(sys.argv[5])*1e-2, 5)
    walkParams.slope = round(float(sys.argv[6])*1e-4, 5)
    autosave = sys.argv[7]
else:
    WS = False
    walkParams.saveFile = "/tmp/stairs_virgile_open_10000.npy"
    if WS:
        walkParams.guessFile = "/tmp/stairs_virgile_open_warmstarted_ws.npy"
        walkParams.saveFile = "/tmp/stairs_virgile_open_warmstarted_warmstarted.npy"
    base_height = 0.575
    walkParams.vcomRef[0] = 0.4
    walkParams.comWeight = 1000 #1000
    walkParams.slope = 00 * 1e-4
    autosave = False

print("WS", WS)
print("slope", walkParams.slope)
print("velocity", walkParams.vcomRef)


# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

robot = load_complete_open(base_height)
assert len(walkParams.stateImportance) == robot.model.nv * 2

# #####################################################################################
# ### CONTACT PATTERN #################################################################
# #####################################################################################
try:
    # If possible, the initial state and contact pattern are taken from a file.
    ocpConfig = sobec.wwt.loadProblemConfig()
    contactPattern = ocpConfig["contactPattern"]
    robot.x0 = ocpConfig["x0"]
    stateTerminalTarget = ocpConfig["stateTerminalTarget"]
except (KeyError, FileNotFoundError):
    # When the config file is not found ...
    # Initial config, also used for warm start, both taken from robot wrapper.
    # Contact are specified with the order chosen in <contactIds>.
    cycle = ( [[1, 0]] * walkParams.Tsingle
              + [[1, 1]] * walkParams.Tdouble
              + [[0, 1]] * walkParams.Tsingle
              + [[1, 1]] * walkParams.Tdouble
            )
    contactPattern = (
        []
        + [[1, 1]] * walkParams.Tstart
        + (cycle * 4)
        + [[1, 1]] * walkParams.Tend
        + [[1, 1]]
    )

# #####################################################################################
# ### VIZ #############################################################################
# #####################################################################################
try:
    import meshcat
    from pinocchio.visualize import MeshcatVisualizer
    viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    viz.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    viz.clean()
    viz.loadViewerModel(rootNodeName="universe")
except (ImportError, AttributeError):
    print("No viewer")


q0 = robot.x0[: robot.model.nq]
print(
    "Start from q0=",
    "half_sitting"
    if norm(q0 - robot.model.referenceConfigurations["half_sitting"]) < 1e-9
    else q0,
)

# #####################################################################################
# ### DDP #############################################################################
# #####################################################################################
print(robot.model.lowerPositionLimit)
print(robot.model.upperPositionLimit)
ddp = sobec.wwt.buildSolver(robot, contactPattern, walkParams, solver='FDDP')
problem = ddp.problem
x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])
# import mim_solvers
# ddp.setCallbacks([mim_solvers.CallbackVerbose(), mim_solvers.CallbackLogger()])
# ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger(), sobec.CallbackNumDiff()])

with open("/tmp/virgile-repr.ascii", "w") as f:
    f.write(sobec.reprProblem(ddp.problem))
    print("OCP described in /tmp/virgile-repr.ascii")

croc.enable_profiler()
ddp.solve(x0s, u0s, 200)

# assert sobec.logs.checkGitRefs(ddp.getCallbacks()[1], "refs/virgile-logs.npy")

# ### PLOT ######################################################################
# ### PLOT ######################################################################
# ### PLOT ######################################################################

sol = sobec.wwt.Solution(robot, ddp)

plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, sol.xs, sol.us, sol.fs0)

target = problem.terminalModel.differential.costs.costs[
    "stateReg"
].cost.residual.reference
forceRef = [
    sobec.wwt.plotter.getReferenceForcesFromProblemModels(problem, cid)
    for cid in robot.contactIds
]
forceRef = [np.concatenate(fs) for fs in zip(*forceRef)]

plotter.plotBasis(target)
plotter.plotTimeCop()
plotter.plotCopAndFeet(walkParams.footSize, 0.6)
plt.savefig("virgile-open-cop.png")
plotter.plotForces(forceRef)
plotter.plotCom(robot.com0)
plotter.plotFeet()
plotter.plotFootCollision(walkParams.footMinimalDistance)
plotter.plotJointTorques()
plotter.plotComAndCopInXY()
print("Run ```plt.ion(); plt.show()``` to display the plots.")
plt.ion()
plt.show()

costPlotter = sobec.wwt.plotter.CostPlotter(robot.model, ddp)
costPlotter.setData()
costPlotter.plotCosts()
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

pin.SE3.__repr__ = pin.SE3.__str__
np.set_printoptions(precision=2, linewidth=300, suppress=True, threshold=10000)
if not autosave:
    while input("Press q to quit the visualisation") != "q":
        viz.play(np.array(ddp.xs)[:, : robot.model.nq], walkParams.DT)

    if walkParams.saveFile is not None and input("Save trajectory? (y/n)") == "y":
        sobec.wwt.save_traj(xs=np.array(sol.xs), us=np.array(sol.us), fs=sol.fs0, acs=sol.acs, n_iter=ddp.iter, filename=walkParams.saveFile)
else:
    sobec.wwt.save_traj(xs=np.array(sol.xs), us=np.array(sol.us), fs=sol.fs0, acs=sol.acs, n_iter=ddp.iter, filename=walkParams.saveFile)

# for x in ddp.xs:
#     viz.display(x[:robot.model.nq])
    # ims.append( viz.viewer.get_image())
# import imageio # pip install imageio[ffmpeg]
# imageio.mimsave("/tmp/battobot.mp4", imgs, 1//walkParams.DT)
