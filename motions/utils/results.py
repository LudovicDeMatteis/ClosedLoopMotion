import sobec
import crocoddyl
import sobec.walk_without_think.plotter
import numpy as np
import matplotlib.pyplot as plt

def plot_solution(robot, ddp, sol, walkParams):
    plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
    plotter.setData(walkParams.contactPattern, sol.xs, sol.us, sol.fs0)

    target = ddp.problem.terminalModel.differential.costs.costs[
        "stateReg"
    ].cost.residual.reference
    forceRef = [
        sobec.wwt.plotter.getReferenceForcesFromProblemModels(ddp.problem, cid)
        for cid in robot.contactIds
    ]
    forceRef = [np.concatenate(fs) for fs in zip(*forceRef)]

    plotter.plotBasis(target)
    plotter.plotTimeCop()
    plotter.plotCopAndFeet(walkParams.footSize, 0.6)
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
    plt.ion()
    plt.show()

def create_viewer(robot, adress="127.0.0.1", port=6000, open=False):
    try:
        import meshcat
        from pinocchio.visualize import MeshcatVisualizer
        viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        viz.viewer = meshcat.Visualizer(zmq_url="tcp://{}:{}".format(adress, port))
        viz.clean()
        viz.loadViewerModel(rootNodeName="universe")

        return viz
    except (ImportError, AttributeError):
        print("No viewer")

        return None
