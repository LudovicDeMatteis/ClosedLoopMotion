import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params as paramsMotions


def squat_battobot_open(
    squat_duration,
    squat_height,
    guessFile=None,
    saveFile=None,
    benchmark=False,
):
    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################
    robot = loaders.battobot_open(base_height=0.575)
    params = paramsMotions.SquatBattobotParams("open", squat_height)
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
        robot, ddp, sol, params, report = squat_battobot_open(1, 0.75, benchmark=True)
        print_bench(report)
        plot_bench(report)
    else:
        for squat_height in [
            1.0,
            0.95,
            0.9,
            0.85,
            0.8,
            0.75,
            0.7,
            0.65,
            0.6,
            1.05,
            1.1,
            1.15,
        ]:
            print(f"Running squat with height {squat_height}")
            robot, ddp, sol, params = squat_battobot_open(1, squat_height)
            # plot_solution(robot, ddp, sol, params)

            # Visualize the solution
            viz = create_viewer(robot)
            while input("Press q to quit the visualisation") != "q":
                viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

            params.saveFile = f"/tmp/squat_{int(squat_height * 100)}_battobot_open.npy"

            if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
                sobec.wwt.save_traj(
                    xs=np.array(sol.xs),
                    us=np.array(sol.us),
                    fs=sol.fs0,
                    acs=sol.acs,
                    n_iter=ddp.iter,
                    filename=params.saveFile,
                )
