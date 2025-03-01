import pinocchio as pin
import crocoddyl as croc
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401

# Local imports
import sobec
import loaders
import params as motionsParams


def walk_cleobot_closed(
    com_vel,
    n_steps,
    ss_duration,
    ds_duration,
    com_weight,
    external_run=True,
    guessFile=None,
    saveFile=None,
    benchmark=False,
):
    # #####################################################################################
    # ### LOAD ROBOT ######################################################################
    # #####################################################################################

    robot = loaders.cleobot_closed(base_height=0.575)

    params = motionsParams.WalkCleobotParams(robot.model)
    print(params.vcomRef)
    params.vcomRef[0] = com_vel
    params.Tsingle = int(ss_duration / params.DT)
    params.Tdouble = motionsParams.roundToOdd(int(ds_duration / params.DT))
    params.cycle = (
        [[1, 0]] * params.Tsingle
        + [[1, 1]] * params.Tdouble
        + [[0, 1]] * params.Tsingle
        + [[1, 1]] * params.Tdouble
    )
    params.contactPattern = contactPattern = (
        []
        + [[1, 1]] * params.Tstart
        + (params.cycle * int(n_steps))
        + [[1, 1]] * params.Tend
        + [[1, 1]]
    )
    params.comWeight = com_weight
    params.guessFile = guessFile
    params.saveFile = saveFile
    print(len(params.contactPattern))
    input()

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
        contactPattern = params.contactPattern

    # #####################################################################################
    # ### VIZ #############################################################################
    # ####################################################################################

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
    ddp = sobec.wwt.buildSolver(robot, contactPattern, params, solver="FDDP")
    problem = ddp.problem
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
    from motions.utils import plot_solution, create_viewer, print_bench

    vel = 0.9
    robot, ddp, sol, params, report = walk_cleobot_closed(
        vel,
        2,
        0.3,
        0.01,
        0,
        saveFile=f"/tmp/walk_vel/walk_{int(10 * vel)}_battobot_closed.npy",
        benchmark=True,
    )
    print_bench(report)

    # plot_solution(robot, ddp, sol, params)

    # Visualize the solution
    viz = create_viewer(robot, open=False)
    while input("Press q to quit the visualisation") != "q":
        viz.play(np.array(ddp.xs)[:, : robot.model.nq], params.DT)

    print(params.saveFile)
    if params.saveFile is not None and input("Save trajectory? (y/n)") == "y":
        sobec.wwt.save_traj(
            xs=np.array(sol.xs),
            us=np.array(sol.us),
            fs=sol.fs0,
            acs=sol.acs,
            n_iter=ddp.iter,
            filename=params.saveFile,
        )
