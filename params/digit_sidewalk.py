import numpy as np
from params.params_base import ParamsBase, roundToOdd

class SideWalkDigitParams(ParamsBase):
    '''
    Parameters for the walk of the Battobot robot.
    '''
    mainJointsIds = [
        'hipz_right',
        'hipy_right',
        'knee_right',
        'hipz_left',
        'hipy_left',
        'knee_left',
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

    cycle = ( [[1, 0]] * Tsingle
              + [[1, 1]] * Tdouble
              + [[0, 1]] * Tsingle
              + [[1, 1]] * Tdouble
            )
    contactPattern = (
        []
        + [[1, 1]] * Tstart
        + (cycle * 4)
        + [[1, 1]] * Tend
        + [[1, 1]]
    )

    ## Define costs
    # * Task specific cost
    vcomWeight = 1e6
    vcomRef = np.r_[ 0, 0.3, 0 ]
    vcomImportance = np.array([1, 1, 0])

    comWeight = 1000 
    comRef = np.r_[0, 0, 1] # values are element-wise multiplied by the initial com
    comImportance = np.array([0, 0, 1])
    
    # * Impact Time costs
    impactAltitudeWeight = 1e4  # /
    impactRotationWeight = 1e4  # /
    impactVelocityWeight = 1e4  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 0.1       # /
    refTorqueWeight = 0.05     # /
    stateTerminalWeight = 1e4
    refForceWeight = 10       # /

    # * Realism costs
    centerOfFrictionWeight = 0
    coneAxisWeight =  0.000
    conePenaltyWeight = 0
    copWeight = 10
    feetCollisionWeight = 100 # 1000
    groundColWeight = 0
    footSize = 0.05
    verticalFootVelWeight = 0 # 20
    jointLimitWeight = 0
    refJointAcceleration = 0.0

    flyHighWeight =  100
    flyHighSlope = 3/2e-2
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

    def __init__(self):
        eps = 0
        basisQWeights = [0,0,0,50,50,0]
        legQWeights = [
            1, 10, 10, # hip x, z, y
            1, # knee (motor)
            1, # knee (passive)
            1, 1, # ankle x, y
            eps, eps, # ujoint ankles-shins
            eps, eps, # ujoint ankles-shins
            eps, # calf motor
            eps, eps, eps, # spherical ankle
            eps, # calf motor
            eps, eps, eps, # spherical ankle
            eps, eps, # Ujoint knee
            eps, eps, eps, # spherical hip
        ]
        basisVWeights = [0,0,0,3,3,1]
        legVWeights = [
            1, 5, 5, # hip x, y, z
            1, # knee (motor)
            1, # knee (passive)
            1, 1, # ankle x, y
            eps, eps, # ujoint ankles-shins
            eps, eps, # ujoint ankles-shins
            eps, # calf motor
            eps, eps, eps, # spherical ankle
            eps, # calf motor
            eps, eps, eps, # spherical ankle
            eps, eps, # Ujoint knee
            eps, eps, eps, # spherical hip
        ]
        self.stateImportance = np.array(
            basisQWeights + legQWeights * 2 + basisVWeights + legVWeights * 2
        )
        velocityTarget = np.zeros(2*len(legVWeights))
        velocityTarget[np.nonzero(legVWeights*2)] = 1
        self.stateTerminalImportance = np.array([0, 0, 0, 0, 0, 0] + [0] * (2*len(legVWeights)) + [0, 0, 0, 0, 0, 0] + velocityTarget.tolist())
        self.controlImportance = np.array([1] * 12)
