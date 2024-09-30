import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params

def jump_battobot_open(jump_duration, guessFile, saveFile):
    walkParams = params.JumpBattobotParams('open')
    walkParams.TFlyUp = ((jump_duration / 2) / walkParams.DT)
    walkParams.TFlyDown = walkParams.TFlyUp
    walkParams.TFly = walkParams.TFlyUp + walkParams.TFlyDown
    contactPattern = (
        []
        + [[1, 1]] * (walkParams.TStand + walkParams.TPush)
        + [[0, 0]] * (walkParams.TFlyUp + walkParams.TFlyDown)
        + [[1, 1]] * (walkParams.TLand + walkParams.Tend)
    )
    Ttotal = len(contactPattern)
    TMid = walkParams.Tstart + walkParams.TFlyUp
    
    base_height = 0.575

    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################

    robot = loaders.battobot_open(base_height=base_height)
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
    ddp = sobec.wwt.buildJumpSolver(robot, contactPattern, walkParams, solver='FDDP')
    problem = ddp.problem
    x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
    ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])

    with open("/tmp/virgile-repr.ascii", "w") as f:
        f.write(sobec.reprProblem(ddp.problem))
        print("OCP described in /tmp/virgile-repr.ascii")

    croc.enable_profiler()
    ddp.solve(x0s, u0s, 200)

    sol = sobec.wwt.Solution(robot, ddp)

    return robot, ddp, sol, walkParams

if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer
    robot, ddp, sol, params = jump_battobot_open(0.4)

    plot_solution(robot, ddp, sol, params)

    # Visualize the solution
    viz = create_viewer(robot)
    while input("Press q to quit the visualisation") != "q":
        viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

    if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
        sobec.wwt.save_traj(xs=np.array(sol.xs), us=np.array(sol.us), fs=sol.fs0, acs=sol.acs, n_iter=ddp.iter, filename=params.saveFile)

