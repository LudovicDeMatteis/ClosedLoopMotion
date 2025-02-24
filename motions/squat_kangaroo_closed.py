import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params


def squat_kangaroo_closed(jump_duration, guessFile=None, saveFile=None, benchmark=False):
    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################
    robot = loaders.kangaroo_closed()
    walkParams = params.SquatKangarooParams(robot.model)
    walkParams.TFlyUp = int((jump_duration / 2) / walkParams.DT)
    walkParams.TFlyDown = walkParams.TFlyUp
    walkParams.TFly = walkParams.TFlyUp + walkParams.TFlyDown
    walkParams.contactPattern = contactPattern = (
        []
        + [[1, 1]] * int(walkParams.TStand + walkParams.TPush)
        + [[1, 1]] * int(walkParams.TLand + walkParams.Tend)
    )
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

    max_iter = 1000
    if benchmark:
        from motions.utils import ReportBench

        croc.stop_watch_reset_all()
        croc.enable_profiler()
        ddp.solve(x0s, u0s, max_iter)
        croc.disable_profiler()
        report_bench = ReportBench()
        sol = sobec.wwt.Solution(robot, ddp)

        return robot, ddp, sol, walkParams, report_bench

    else:
        ddp.solve(x0s, u0s, max_iter)
        sol = sobec.wwt.Solution(robot, ddp)
        return robot, ddp, sol, walkParams


if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer, plot_bench, print_bench

    benchmark = False
    if benchmark:
        robot, ddp, sol, params, report = squat_kangaroo_closed(0.4, benchmark=True)
        print_bench(report)
        plot_bench(report)
    else:
        robot, ddp, sol, params = squat_kangaroo_closed(0.4)
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
