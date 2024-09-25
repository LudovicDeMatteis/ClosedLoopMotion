import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import sobec.walk_without_think.plotter
import loaders
import params

# #####################################################################################
# ## TUNING ###########################################################################
# #####################################################################################

# In the code, cost terms with 0 weight are commented for reducing execution cost
# An example of working weight value is then given as comment at the end of the line.
# When setting them to >0, take care to uncomment the corresponding line.
# All these lines are marked with the tag ##0##.

import sys
print(sys.argv)

walkParams = params.JumpDisneyParams()
base_height = 0.600

# #####################################################################################
# ### LOAD ROBOT ######################################################################
# #####################################################################################

robot = loaders.disney_closed()
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
    contactPattern = walkParams.contactPattern

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
ddp = sobec.wwt.buildJumpSolver(robot, contactPattern, walkParams, solver='FDDP')
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

# plotter.plotBasis(target)
# plotter.plotTimeCop()
# plotter.plotCopAndFeet(walkParams.footSize, 0.6)
plotter.plotForces(forceRef)
# plotter.plotCom(robot.com0)
# plotter.plotFeet()
# plotter.plotFootCollision(walkParams.footMinimalDistance)
# plotter.plotJointTorques()
# plotter.plotComAndCopInXY()
print("Run ```plt.ion(); plt.show()``` to display the plots.")
plt.ion()
plt.show()

costPlotter = sobec.wwt.plotter.CostPlotter(robot.model, ddp)
costPlotter.setData()
costPlotter.plotCosts()
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################
# ## DEBUG ######################################################################

while input("Press q to quit the visualisation") != "q":
    viz.play(np.array(ddp.xs)[:, : robot.model.nq], walkParams.DT)

# for x in ddp.xs:
#     viz.display(x[:robot.model.nq])
    # ims.append( viz.viewer.get_image())
# import imageio # pip install imageio[ffmpeg]
# imageio.mimsave("/tmp/battobot.mp4", imgs, 1//walkParams.DT)
