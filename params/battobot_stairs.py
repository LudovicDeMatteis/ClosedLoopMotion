import numpy as np

def roundToOdd(x):
    '''
    Round a number to the nearest odd number.
    For double support phase, taking an odd number of steps is better.
    '''
    return 2 * int(np.round(x / 2 - 0.5001)) + 1

class StairsBattobotParams:
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
    Tstart = int(0.1 / DT)
    Tsingle = int(0.5 / DT)  
    Tdouble = roundToOdd(0.1 / DT)
    Tend = int(0.5 / DT)
    Tmpc = int(1.4 / DT)
    Tsimu = int(10 / DT)
    transitionDuration = (Tdouble - 1) // 2

    ncycle = 4
    cycle = ( [[1, 0]] * Tsingle
              + [[1, 1]] * Tdouble
              + [[0, 1]] * Tsingle
              + [[1, 1]] * Tdouble
            )
    contactPattern = (
        []
        + [[1, 1]] * Tstart
        + (cycle * ncycle)
        + [[1, 1]] * Tend
        + [[1, 1]]
    )
    Tcycle = len(cycle)
    Ttotal = len(contactPattern)

    ## Define costs
    # * Task specific cost
    vcomWeight = 1e3
    vcomRef = np.r_[ 0.3, 0, 0 ]
    vcomImportance = np.array([1, 0, 0])

    comWeight = 0 
    comRef = np.r_[ 0, 0, 0]
    comImportance = np.array([0, 0, 1])

    comHeightWeight = 0
    comHeightTargets = [np.r_[0.0, 0.0, 0.8]]
    comHeightTimes = []
    
    # * Impact Time costs
    impactAltitudeWeight = 1e4  # /
    impactRotationWeight = 1e4  # /
    impactVelocityWeight = 1e4  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 0.05       # /
    refTorqueWeight = 0.002     # /
    stateTerminalWeight = 1e4
    refForceWeight = 10       # /

    # * Realism costs
    centerOfFrictionWeight = 0
    coneAxisWeight =  0.000
    conePenaltyWeight = 0
    copWeight = 1
    feetCollisionWeight = 0 # 1000
    groundColWeight = 0
    footSize = 0.05
    verticalFootVelWeight = 0 # 20
    jointLimitWeight = 0
    refJointAcceleration = 0.0

    flyHighWeight = 50
    flyHighSlope = 3/2e-2
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
    saveFile = "/tmp/stairs_virgile_closed.npy"
    guessFile = None
    preview = True
    save = False

    slope = 0.1
    footTrajectoryWeight = 1e6
    def get_foot_trajectories(self):
        self.groundHeight = [self.slope * t * self.DT for t in range(self.Ttotal)]
        # print(self.groundHeight)
        # input()
        self.groundAltitude = [0] * self.Ttotal
        self.footTrajectories = np.array([[[0., 0., 0.], [0., 0., 0.]] for t in range(self.Ttotal)])
        self.footTrajImportance = np.array([[[0, 0, 0], [0, 0, 0]] for t in range(self.Ttotal)])
        for i in range(self.ncycle):
            nextFloor = self.groundHeight[self.Tstart + self.Tsingle + i*self.Tcycle]
            self.footTrajectories[self.Tstart + 3*self.Tsingle//5 + i*self.Tcycle] = np.array([[0, 0, 0], [0, 0, 1.2 * nextFloor]])
            self.footTrajImportance[self.Tstart + 3*self.Tsingle//5 + i*self.Tcycle] = np.array([[0, 0, 0], [0, 0, 1]])
            nextFloor = self.groundHeight[self.Tstart + 2*self.Tsingle + self.Tdouble + i*self.Tcycle]
            self.footTrajectories[self.Tstart + 3*self.Tsingle//5 + self.Tdouble + self.Tsingle + i*self.Tcycle] = np.array([[0, 0, 1.2 * nextFloor], [0, 0, 0]])
            self.footTrajImportance[self.Tstart + 3*self.Tsingle//5 + self.Tdouble + self.Tsingle + i*self.Tcycle] = np.array([[0, 0, 1], [0, 0, 0]])

    def __init__(self, model_type="open"):
        self.get_foot_trajectories()
        if model_type == "open":
            basisQWeights = [0,0,0,50,50,0]
            legQWeights = [
                20, 10, 1, # hip z, x, y
                4, # knee (passive)
                1, 1, # ankle x, y
            ]
            basisVWeights = [0,0,0,3,3,1]
            legVWeights = [
                10, 5, 1, # hip z, x, y
                2, # knee (passive)
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
                1, # knee (passive)
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
                1, # knee (passive)
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
            self.stateTerminalImportance = np.array([0, 0, 0, 0, 0, 0] + [0] * (2*len(legVWeights)) + [0, 0, 0, 0, 0, 0] + velocityTarget.tolist())
            self.controlImportance = np.array([1] * 12)
