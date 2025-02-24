import numpy as np
from params.params_base import ParamsBase, roundToOdd


class WalkKangarooParams(ParamsBase):
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
    Tstart = int(0.2 / DT)
    Tsingle = int(0.4 / DT)
    Tdouble = roundToOdd(0.01 / DT)
    Tend = int(0.2 / DT)
    Tmpc = int(1.4 / DT)
    Tsimu = int(10 / DT)
    transitionDuration = (Tdouble - 1) // 2

    cycle = (
        [[1, 0]] * Tsingle
        + [[1, 1]] * Tdouble
        + [[0, 1]] * Tsingle
        + [[1, 1]] * Tdouble
    )
    contactPattern = [] + [[1, 1]] * Tstart + (cycle * 4) + [[1, 1]] * Tend + [[1, 1]]

    ## Define costs
    # * Task specific cost
    vcomWeight = 1e5
    vcomRef = np.r_[0.6, 0, 0]
    vcomImportance = np.array([1, 0, 0])

    comWeight = 0
    comRef = np.r_[0, 0, 1]  # values are element-wise multiplied by the initial com
    comImportance = np.array([0, 0, 1])

    # * Impact Time costs
    impactAltitudeWeight = 1e4  # /
    impactRotationWeight = 1e4  # /
    impactVelocityWeight = 1e4  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 1e-6  # /
    refTorqueWeight = 1e-8  # /
    stateTerminalWeight = 1e4
    refForceWeight = 0  # /

    # * Realism costs
    centerOfFrictionWeight = 0
    coneAxisWeight = 0.000
    conePenaltyWeight = 0
    copWeight = 0
    feetCollisionWeight = 0  # 1000
    groundColWeight = 0
    footSize = 0.05
    verticalFootVelWeight = 0  # 20
    jointLimitWeight = 0
    refJointAcceleration = 0.0

    flyHighWeight = 0
    flyHighSlope = 6 / 2e-2
    slope = 0.0000
    minimalNormalForce = 1.0
    withNormalForceBoundOnly = False
    footMinimalDistance = 0.2

    # Solver parameters
    kktDamping = 0
    baumgartGains = np.array([0, 100])
    transitionDuration = 4
    solver_th_stop = 1e-3
    solver_maxiter = 200
    solver_reg_min = 1e-6

    # Save parameters
    saveFile = None
    guessFile = None
    preview = True
    save = False

    def __init__(self, rmodel):
        nv = rmodel.nv
        basisQWeights = [0, 0, 0, 50, 50, 0]
        legQWeights = np.ones(nv - 6).tolist()
        basisVWeights = [0, 0, 0, 3, 3, 1]
        legVWeights = np.ones(nv - 6).tolist()

        self.stateImportance = np.array(
            basisQWeights + legQWeights + basisVWeights + legVWeights
        )
        velocityTarget = np.zeros(len(legVWeights))
        velocityTarget[np.nonzero(legVWeights)] = 1
        self.stateTerminalImportance = np.array(
            [0, 0, 0, 0, 0, 0]
            + [0] * (len(legVWeights))
            + [0, 0, 0, 0, 0, 0]
            + velocityTarget.tolist()
        )
        self.controlImportance = np.array([1] * 12)
