import numpy as np
from params.params_base import ParamsBase, roundToOdd


class SquatBattobotParams(ParamsBase):
    """
    Parameters for the walk of the Battobot robot.
    """

    mainJointsIds = [
        "hipz_right",
        "hipy_right",
        "knee_right",
        "hipz_left",
        "hipy_left",
        "knee_left",
    ]
    # Define time steps
    DT = 0.015
    TSquat = 100

    contactPattern = [] + [[1, 1]] * TSquat
    Ttotal = len(contactPattern)

    comRefTrajWeight = 1e5
    footSize = 0.05

    # * Impact Time costs
    impactAltitudeWeight = 1e5  # /
    impactRotationWeight = 1000  # /
    impactVelocityWeight = 1000  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 0.2  # /
    refTorqueWeight = 0.01  # /
    stateTerminalWeight = 1000
    refForceWeight = 0  # /
    copWeight = 0

    # Solver parameters
    kktDamping = 0
    baumgartGains = np.array([0, 10])
    transitionDuration = 4
    solver_th_stop = 1e-4
    solver_maxiter = 200
    solver_reg_min = 1e-6

    # Save parameters
    saveFile = "/tmp/squat/battobot_05.npy"
    guessFile = None
    preview = True
    save = False

    def __init__(self, model_type="open", squat_height=0.75):
        href_relative_mean = (1 + squat_height) / 2
        href_relative_amplitude = (1 - squat_height) / 2
        self.comRefTraj = [
            [
                1,
                1,
                href_relative_mean
                + href_relative_amplitude * np.cos(2 * np.pi * t / self.Ttotal),
            ]
            for t in range(self.Ttotal)
        ]
        self.comRefTrajImportance = np.array([[1, 1, 1] for _ in range(self.Ttotal)])
        self.comRefTrajImportance[self.Ttotal // 2] = [100, 100, 100]
        if model_type == "open":
            basisQWeights = [0, 0, 0, 50, 50, 0]
            legQWeights = [
                1,
                1,
                1,  # hip z, x, y
                1,  # knee (passive)
                1,
                1,  # ankle x, y
            ]
            basisVWeights = [0, 0, 0, 3, 3, 1]
            legVWeights = [
                1,
                1,
                1,  # hip z, x, y
                1,  # knee (passive)
                1,
                1,  # ankle x, y
            ]
            self.stateImportance = np.array(
                basisQWeights + legQWeights * 2 + basisVWeights + legVWeights * 2
            )
            nv = len(basisVWeights) + 2 * len(legVWeights)
            self.stateTerminalImportance = np.array(
                [0, 0, 10, 0, 0, 50] + [1] * (nv - 6) + [1] * nv
            )
            self.controlImportance = np.array([1] * 12)
        if model_type == "closed":
            eps = 0
            basisQWeights = [0, 0, 0, 50, 50, 0]
            legQWeights = [
                1,
                1,
                1,  # hip z, x, y
                1,  # knee (passive)
                1,
                1,  # ankle x, y
                eps,  # knee (actuated)
                eps,
                eps,
                eps,  # spherical ankle
                eps,
                eps,
                eps,  # spherical ankle
                eps,
                eps,  # Ujoint knee
                eps,  # calf motor
                eps,
                eps,  # ujoint ankles-shins
                eps,  # calf motor
                eps,
                eps,  # ujoint ankles-shins
                eps,
                eps,
                eps,  # spherical hip
            ]
            basisVWeights = [0, 0, 0, 3, 3, 1]
            legVWeights = [
                1,
                1,
                1,  # hip z, x, y
                1,  # knee (passive)
                1,
                1,  # ankle x, y
                eps,  # knee (actuated)
                eps,
                eps,
                eps,  # spherical ankle
                eps,
                eps,
                eps,  # spherical ankle
                eps,
                eps,  # Ujoint knee
                eps,  # calf motor
                eps,
                eps,  # ujoint ankles-shins
                eps,  # calf motor
                eps,
                eps,  # ujoint ankles-shins
                eps,
                eps,
                eps,  # spherical hip
            ]
            self.stateImportance = np.array(
                basisQWeights + legQWeights * 2 + basisVWeights + legVWeights * 2
            )
            velocityTarget = np.zeros(2 * len(legVWeights))
            velocityTarget[np.nonzero(legVWeights * 2)] = 1
            self.stateTerminalImportance = np.array(
                [0, 0, 10, 0, 0, 50]
                + [1] * (2 * len(legVWeights))
                + [0, 0, 0, 0, 0, 0]
                + velocityTarget.tolist()
            )
            self.controlImportance = np.array([1] * 12)

    def getReferenceForces(self, grav, com0):
        return []
