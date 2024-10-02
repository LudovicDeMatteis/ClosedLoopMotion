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

def stairs_battobot_open(com_vel, n_steps, ss_duration, ds_duration, stairs_height, external_run=True, guessFile=None, saveFile=None):
    walkParams = params.StairsBattobotParams('open')
    walkParams.vcomRef[0] = com_vel
    walkParams.Tsingle = int(ss_duration / walkParams.DT)
    walkParams.Tdouble = params.roundToOdd(int(ds_duration / walkParams.DT))
    walkParams.cycle = ( [[1, 0]] * walkParams.Tsingle
                        + [[1, 1]] * walkParams.Tdouble
                        + [[0, 1]] * walkParams.Tsingle
                        + [[1, 1]] * walkParams.Tdouble
                        )
    walkParams.contactPattern = contactPattern = (
        []
        + [[1, 1]] * walkParams.Tstart
        + (walkParams.cycle * int(n_steps))
        + [[1, 1]] * walkParams.Tend
        + [[1, 1]]
    )
    walkParams.Tcycle = len(walkParams.cycle)
    walkParams.Ttotal = len(contactPattern)
    stairs_slope = stairs_height
    walkParams.slope = stairs_slope
    walkParams.guessFile = guessFile
    walkParams.saveFile = saveFile

    base_height = 0.575

    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################

    robot = loaders.battobot_open(base_height=base_height)
    assert len(walkParams.stateImportance) == robot.model.nv * 2
    assert len(walkParams.stateTerminalImportance) == robot.model.nv * 2

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

    ddp = sobec.wwt.buildStairsSolver(robot, contactPattern, walkParams, solver='FDDP')
    problem = ddp.problem
    x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
    ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])

    with open("/tmp/virgile-repr.ascii", "w") as f:
        f.write(sobec.reprProblem(ddp.problem))
        print("OCP described in /tmp/virgile-repr.ascii")

    croc.enable_profiler()
    # ddp.alphas = [1.0, 0.1, 0.01]
    # ddp.solve(x0s, u0s, 200, init_reg=0.001)
    ddp.solve(init_xs=x0s, init_us=u0s, maxiter=200, init_reg=1e-3, is_feasible=False)
    # assert sobec.logs.checkGitRefs(ddp.getCallbacks()[1], "refs/virgile-logs.npy")

    sol = sobec.wwt.Solution(robot, ddp)
    return (robot, ddp, sol, walkParams)

if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer
    robot, ddp, sol, params = stairs_battobot_open(0.7, 4, 0.4, 0.01, 0)

    plot_solution(robot, ddp, sol, params)

    # Visualize the solution
    viz = create_viewer(robot)
    while input("Press q to quit the visualisation") != "q":
        viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

    if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
        sobec.wwt.save_traj(xs=np.array(sol.xs), us=np.array(sol.us), fs=sol.fs0, acs=sol.acs, n_iter=ddp.iter, filename=params.saveFile)
