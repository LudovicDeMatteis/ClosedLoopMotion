import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params


def jump_battobot_closed(jump_duration, guessFile=None, saveFile=None, benchmark=False):
    walkParams = params.JumpBattobotParams("closed")
    walkParams.TFlyUp = int((jump_duration / 2) / walkParams.DT)
    walkParams.TFlyDown = walkParams.TFlyUp
    walkParams.TFly = walkParams.TFlyUp + walkParams.TFlyDown
    walkParams.contactPattern = contactPattern = (
        []
        + [[1, 1]] * int(walkParams.TStand + walkParams.TPush)
        + [[0, 0]] * int(walkParams.TFlyUp + walkParams.TFlyDown)
        + [[1, 1]] * int(walkParams.TLand + walkParams.Tend)
    )
    base_height = 0.575

    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################

    robot = loaders.battobot_closed(base_height=base_height)
    assert len(walkParams.stateImportance) == robot.model.nv * 2

    # #####################################################################################
    # ### CONTACT PATTERN #################################################################
    # #####################################################################################
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
    ddp = sobec.wwt.buildJumpSolver(robot, contactPattern, walkParams, solver="FDDP")
    x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
    ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])

    croc.stop_watch_reset_all()
    croc.enable_profiler()
    ddp.solve(x0s, u0s, 200)
    croc.disable_profiler()

    sol = sobec.wwt.Solution(robot, ddp)

    if benchmark:
        return robot, ddp, sol, walkParams, croc.stop_watch_report(3)
    return robot, ddp, sol, walkParams


if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer

    robot, ddp, sol, params, report_bench = jump_battobot_closed(0.4, benchmark=True)
    plot_solution(robot, ddp, sol, params)

    # Visualize the solution
    viz = create_viewer(robot)
    while input("Press q to quit the visualisation") != "q":
        viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

    if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
        sobec.wwt.save_traj(
            xs=np.array(sol.xs),
            us=np.array(sol.us),
            fs=sol.fs0,
            acs=sol.acs,
            n_iter=ddp.iter,
            filename=params.saveFile,
        )
