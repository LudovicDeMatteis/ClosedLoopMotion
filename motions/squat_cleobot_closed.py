import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params as paramsMotions


def squat_cleobot_closed(
    squat_duration,
    squat_height,
    guessFile=None,
    saveFile=None,
    benchmark=False,
):
    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################
    robot = loaders.cleobot_closed(base_height=0.575)
    params = paramsMotions.SquatCleobotParams(robot.model, squat_height)
    assert len(params.stateImportance) == robot.model.nv * 2

    # #####################################################################################
    # ### CONTACT PATTERN #################################################################
    # #####################################################################################
    contactPattern = params.contactPattern

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
    ddp = sobec.wwt.buildJumpSolver(robot, contactPattern, params, solver="FDDP")
    x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, params)
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

        return robot, ddp, sol, params, report_bench

    else:
        ddp.solve(x0s, u0s, max_iter)
        sol = sobec.wwt.Solution(robot, ddp)
        return robot, ddp, sol, params


if __name__ == "__main__":
    from motions.utils import plot_solution, create_viewer, plot_bench, print_bench

    benchmark = False
    if benchmark:
        robot, ddp, sol, params, report = squat_cleobot_closed(1, 0.75, benchmark=True)
        print_bench(report)
        plot_bench(report)
    else:
        squat_height = 0.75
        print(f"Running squat with height {squat_height}")
        robot, ddp, sol, params = squat_cleobot_closed(1, squat_height)
        # plot_solution(robot, ddp, sol, params)

        # Visualize the solution
        viz = create_viewer(robot, open=False)
        while input("Press q to quit the visualisation") != "q":
            viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

        params.saveFile = f"/tmp/squat_{int(squat_height * 100)}_battobot_closed.npy"

        if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
            sobec.wwt.save_traj(
                xs=np.array(sol.xs),
                us=np.array(sol.us),
                fs=sol.fs0,
                acs=sol.acs,
                n_iter=ddp.iter,
                filename=params.saveFile,
            )
