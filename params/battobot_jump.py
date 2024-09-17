import numpy as np

def roundToOdd(x):
    '''
    Round a number to the nearest odd number.
    For double support phase, taking an odd number of steps is better.
    '''
    return 2 * int(np.round(x / 2 - 0.5001)) + 1

class JumpBattobotParams:
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
    Tstart = int(2 / DT)
    Tfly = int(2 / DT)
    Tend = int(2 / DT)
    Tmpc = int(1.4 / DT)
    Tsimu = int(10 / DT)
    # transitionDuration = (Tdouble - 1) // 2

    contactPattern = (
        []
        + [[1, 1]] * (Tstart)
        # + [[1, 0]] * (Tstart //2)
        # + [[0, 0]] * Tfly
        # + [[1, 1]] * Tend
        # + [[1, 1]]
    )

    ## Define costs
    # * Task specific cost
    vcomWeight = 0
    vcomRef = np.r_[ 0, 0, 0 ]
    vcomImportance = np.array([0, 0, 1])

    comWeight = 0 
    comRef = np.r_[ 0, 0, 0]
    comImportance = np.array([0, 0, 1])

    comRefTrajWeight = 1e9
    print("Tstart", Tstart)
    comRefTraj = []
    for t in range(Tstart):
        comRefTraj.append([1, 1, ((1 - 3/4)/2)*np.cos(2*t*np.pi / Tstart) + (1 + 3/4)/2])
    # import matplotlib.pyplot as plt
    # plt.figure()
    # plt.plot(comTraj)
    # plt.show()
    # comHeightTimes = [Tstart + Tfly//2]
    comRefTrajImportance = [np.array([0, 0, 1]) for _ in range(Tstart)]
    # comHeightImportance = np.array([0, 0, 1])
    
    # * Impact Time costs
    impactAltitudeWeight = 1e4  # /
    impactRotationWeight = 1e4  # /
    impactVelocityWeight = 1e4  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 0.4       # /
    refTorqueWeight = 0.05     # /
    stateTerminalWeight = 1e4
    refForceWeight = 0       # /

    # * Realism costs
    centerOfFrictionWeight = 0
    coneAxisWeight =  0.000
    conePenaltyWeight = 0
    copWeight = 10
    feetCollisionWeight = 0 # 1000
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
    solver_th_stop = 1e-4
    solver_maxiter = 200
    solver_reg_min = 1e-6

    # Save parameters
    saveFile = "/tmp/stairs_virgile_closed.npy"
    guessFile = None
    preview = True
    save = False

    def __init__(self, model_type="open"):
        if model_type == "open":
            basisQWeights = [0,0,0,50,50,0]
            legQWeights = [
                1, 1, 1, # hip z, x, y
                1, # knee (passive)
                1, 1, # ankle x, y
            ]
            basisVWeights = [0,0,0,3,3,1]
            legVWeights = [
                1, 1, 1, # hip z, x, y
                1, # knee (passive)
                1, 1, # ankle x, y
            ]
            self.stateImportance = np.array(
                basisQWeights + legQWeights * 2 + basisVWeights + legVWeights * 2
            )
            nv = len(basisVWeights) + 2* len(legVWeights)
            self.stateTerminalImportance = np.array([0, 0, 0, 0, 0, 0] + [0] * (nv - 6) + [1] * nv)
            self.controlImportance = np.array([1] * 12)
        if model_type == 'closed':
            eps = 0
            basisQWeights = [0,0,0,50,50,0]
            legQWeights = [
                20, 3, 1, # hip z, x, y
                4, # knee (passive)
                1, 1, # ankle x, y
                eps, # knee (actuated)
                eps, eps, eps, # spherical ankle
                eps, eps, eps, # spherical ankle
                eps, eps, # Ujoint knee
                eps, # calf motor
                eps, eps, # ujoint ankles-shins
                eps, # calf motor
                eps, eps, # ujoint ankles-shins
                eps, eps, eps, # spherical hip
            ]
            basisVWeights = [0,0,0,3,3,1]
            legVWeights = [
                10, 3, 1, # hip z, x, y
                2, # knee (passive)
                1, 1, # ankle x, y
                eps, # knee (actuated)
                eps, eps, eps, # spherical ankle
                eps, eps, eps, # spherical ankle
                eps, eps, # Ujoint knee
                eps, # calf motor
                eps, eps, # ujoint ankles-shins
                eps, # calf motor
                eps, eps, # ujoint ankles-shins
                eps, eps, eps, # spherical hip
            ]
            self.stateImportance = np.array(
                basisQWeights + legQWeights * 2 + basisVWeights + legVWeights * 2
            )
            velocityTarget = np.zeros(2*len(legVWeights))
            velocityTarget[np.nonzero(legVWeights*2)] = 1
            self.stateTerminalImportance = np.array([3, 3, 0, 0, 0, 30] + [0] * (2*len(legVWeights)) + [0, 0, 0, 0, 0, 0] + velocityTarget.tolist())
            self.controlImportance = np.array([1] * 12)
