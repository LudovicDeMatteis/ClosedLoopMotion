import numpy as np
from numpy.testing import assert_almost_equal
import pinocchio as pin
import crocoddyl
import meshcat
from pinocchio.visualize import MeshcatVisualizer
import time as time

from loaders.loaders_virgile import load_complete_open, load_complete_closed
import sobec
from sobec.walk_without_think.actuation_matrix import ActuationModelMatrix
import specific_params

from tqdm import tqdm

def getOpenWarmstart(ws_file, autosave=False, base_height=0.575, slope=0.0, velocity=0.2, comWeight=0):
    ws = np.load(f"{ws_file}", allow_pickle=True)[()]
    xs_closed = np.array(ws["xs"])
    us_closed = np.array(ws["us"])
    fs_closed = np.array(ws["fs"])
    acs_closed = np.array(ws["acs"])

    walkParams = specific_params.WalkBattobotParams()
    walkParams.vcomRef[0] = velocity
    walkParams.comWeight = comWeight
    walkParams.slope = slope

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

    robot_open = load_complete_open(base_height=base_height)
    robot_closed, (SER_Q, SER_V, LOOP_Q, LOOP_V) = load_complete_closed(export_joints_ids=True, base_height=base_height)

    viz = MeshcatVisualizer(robot_open.model, robot_open.collision_model, robot_open.visual_model)
    viz.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    viz.clean()
    viz.loadViewerModel(rootNodeName="universe")

    act_matrix_open = np.eye(robot_open.model.nv, robot_open.model.nv - 6, -6)
    act_matrix_closed = np.zeros((robot_closed.model.nv, len(robot_closed.actuationModel.mot_ids_v)))
    for iu, iv in enumerate(robot_closed.actuationModel.mot_ids_v):
        act_matrix_closed[iv, iu] = 1

    T = np.shape(xs_closed)[0]
    assert(T == len(contactPattern))
    nq_open, nv_open = robot_open.model.nq, robot_open.model.nv
    nq_closed, nv_closed = robot_closed.model.nq, robot_closed.model.nv
    nu_open = act_matrix_open.shape[1]
    nu_closed = act_matrix_closed.shape[1]
    assert nv_closed == act_matrix_closed.shape[0]
    qs_closed = xs_closed[:, :-nv_closed]
    vs_closed = xs_closed[:, -nv_closed:]
    q0_open = robot_open.x0[:nq_open]
    
    state = crocoddyl.StateMultibody(robot_open.model)
    nx = nq_open + nv_open
    ndx = 2 * nv_open
    SER_X = SER_Q + [
        i + nq_closed for i in SER_V
    ]
    LOOP_X = [i for i in range(nq_closed + nv_closed) if i not in SER_X]

    xs_open = xs_closed[:, SER_X]
    us_open = np.empty((xs_open.shape[0] - 1, nu_open))

    ## Time t = 0
    assert (xs_open[0, :nq_open] - q0_open < 1e-4).all()
    viz.display(xs_open[0, :nq_open])

    actuation = ActuationModelMatrix(state, nu_open, act_matrix_open)

    for t, pattern in tqdm(enumerate(contactPattern[:-1])):
        contact_stack = crocoddyl.ContactModelMultiple(state, actuation.nu)
        for k, cid in enumerate(robot_open.contactIds):
            if not pattern[k]:
                continue
            contact = crocoddyl.ContactModel6D(
                state, cid, pin.SE3.Identity(), pin.WORLD, actuation.nu, np.array([0., 100.])
            )
            contact_stack.addContact(robot_open.model.frames[cid].name + "_contact", contact)
        
        ## Running costs
        cost = crocoddyl.CostModelSum(state, actuation.nu)
        # Control
        uref = np.array([us_closed[t][i] for i in [0, 1, 2, 5, 4, 3, 6, 7, 8, 11, 10, 9]])
        ureg_res = crocoddyl.ResidualModelControl(state, uref)
        ureg_cost = crocoddyl.CostModelResidual(state, ureg_res)
        cost.addCost("ureg", ureg_cost, 1e-3)
        # for k, cid in enumerate(robot_open.contactIds):
        #     if not pattern[k]:
        #         continue
        #     # Force
        #     freg_res = crocoddyl.ResidualModelContactForce(state, cid, pin.Force(fs_open[t][6*k:6*(k+1)]), 6, actuation.nu)
        #     freg_cost = crocoddyl.CostModelResidual(state, freg_res)
        #     cost.addCost(f"freg_{k}", freg_cost, 1e-4)
        # Loop contact forces # ! Not implemented in Crocoddyl yet
        # for i, cm in enumerate(robot_closed.loop_constraints_models):
        #     freg_res = crocoddyl.ResidualModelContactForce(state, cm.joint1_id, pin.Force.Zero(), 6, actuation.nu)
        #     freg_cost = crocoddyl.CostModelResidual(state, freg_res)
        #     cost.addCost(f"freg_loop_{i}", freg_cost, 1e0)

        dam = crocoddyl.DifferentialActionModelContactFwdDynamics(
            state,
            actuation,
            contact_stack,
            cost,
        )
        stm = crocoddyl.IntegratedActionModelEuler(dam, walkParams.DT)

        term_cost = crocoddyl.CostModelSum(state, actuation.nu)

        xref_term_res = crocoddyl.ResidualModelState(state, xs_open[t+1], actuation.nu)
        xref_term_cost = crocoddyl.CostModelResidual(state, xref_term_res)
        term_cost.addCost("xreg", xref_term_cost, 1e4)

        dam = crocoddyl.DifferentialActionModelContactFwdDynamics(
            state,
            actuation,
            contact_stack.copy(),
            term_cost,
        )
        terminal_model = crocoddyl.IntegratedActionModelEuler(dam, 0.0)

        problem = crocoddyl.ShootingProblem(xs_open[t], [stm], terminal_model)

        tol = 1e-12
        solver = crocoddyl.SolverFDDP(problem)
        solver.th_stop = tol
        solver.verbose = False
        solver.setCallbacks([crocoddyl.CallbackVerbose()])

        solver.solve(maxiter=100, init_xs=[xs_open[t], xs_open[t+1]], init_us=[uref])

        xs_open[t+1] = solver.xs[1]
        assert_almost_equal(xs_open[t], solver.xs[0])
        us_open[t] = solver.us[0]

        viz.display(xs_open[t+1, :nq_open])
        print(f"Done for t = {t}")
        # time.sleep(0.1)
    if not autosave:
        while input("Press q to quit the visualisation") != "q":
            viz.play(xs_open[:, :nq_open], walkParams.DT)
    return(xs_open, us_open)

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        ws_file = sys.argv[1]
        save_file = sys.argv[2]
        # b = round(float(sys.argv[3])*1e-3, 5)
        b = 0.575
        w = int(sys.argv[3])
        v = round(float(sys.argv[4])*1e-2, 5)
        s = round(float(sys.argv[5])*1e-4, 5)
        autosave = sys.argv[6]
    else:
        ws_file = "results/walk_slope_00_comWeight_0_vel_40/closed.npy"
        save_file = "/tmp/stairs_virgile_open_ws_warmstarted.npy"
        b = 0.575
        w = 0
        s = 0.0
        v = 0.4
        autosave = False

    xs_open, us_open = getOpenWarmstart(ws_file, autosave, base_height=b, slope=s, velocity=v, comWeight=w)
    if not autosave:
        if input("Do you want to save the warmstart? (y/n)") == "y":
            sobec.wwt.save_traj(np.array(xs_open), np.array(us_open), filename=save_file)
    else:
        sobec.wwt.save_traj(np.array(xs_open), np.array(us_open), filename=save_file)
