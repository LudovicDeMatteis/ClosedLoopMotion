import numpy as np
from params.params_base import ParamsBase, roundToOdd

class JumpDisneyParams(ParamsBase):
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
    TStand = 20
    TPush = 20
    Tstart = TStand + TPush

    TFlyUp = 20
    TFlyDown = TFlyUp
    TFly = TFlyUp + TFlyDown

    TLand = 20
    Tend = 20

    contactPattern = (
        []
        + [[1, 1]] * (TStand + TPush)
        + [[0, 0]] * (TFlyUp + TFlyDown)
        + [[1, 1]] * (TLand + Tend)
    )
    Ttotal = len(contactPattern)
    TMid = Tstart + TFlyUp

    ## Define costs
    # * Task specific cost
    vcomWeight = 0
    vcomRef = np.r_[ 0.2, 0, 0 ]
    vcomImportance = np.array([1, 0, 0])

    # comWeight = 0
    # comRef = np.r_[ 0, 0, 0]
    # comImportance = np.array([0, 0, 1])
    
    v0 = 9.81*TFlyUp*DT # Velocity to arrive at Tmid with the 0 velocity
    href = v0*DT*TFlyUp/2 # Height to reach at Tmid

    comRefTrajWeight = 1e5
    comRefTraj = [np.array([1, 1, 1]) for _ in range(Ttotal)]
    comRefTrajImportance = [np.array([0, 0, 0]) for _ in range(Ttotal)]
    # comRefTraj[3*Tstart // 4] = np.array([1, 1, 3/4])
    # comRefTrajImportance[3*Tstart // 4] = np.array([0, 0, 1])
    # comRefTraj[Tstart-1] = np.array([1, 1, 1.1])
    # comRefTrajImportance[Tstart-1] = np.array([0, 0, 1])
    comRefTraj[TMid] = np.array([1, 1, href])
    comRefTrajImportance[TMid] = np.array([0, 0, 1])

    footSize = 0.05

    # * Impact Time costs
    impactAltitudeWeight = 1e5  # /
    impactRotationWeight = 1000  # /
    impactVelocityWeight = 1000  # /
    refMainJointsAtImpactWeight = 0

    # * Regularisation costs
    refStateWeight = 0.7       # /
    refTorqueWeight = 0.05     # /
    stateTerminalWeight = 1000
    refForceWeight = 500       # /
    copWeight = 5

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
        self.stateImportance = np.ones(52*2)
        self.stateTerminalImportance = np.array([0, 0, 0, 0, 0, 0] + [0] * 46 + [1]*52 )
        self.controlImportance = np.array([1] * 12)

    def getReferenceForces(self, grav, com0):
        alpha = 1+self.v0/(9.81*self.TPush*self.DT)
        fpush = alpha/2*grav
        alpha = 1+self.v0/(9.81*self.TLand*self.DT)
        fland = alpha/2*grav
        referenceForces = [np.array([1/2, 1/2])*grav for _ in range(self.TStand)]
        referenceForces += [np.array([fpush, fpush]) for _ in range(self.TPush)]
        referenceForces += [np.array([0, 0]) for _ in range(self.TFly)]
        referenceForces += [np.array([fland, fland]) for _ in range(self.TLand)]
        referenceForces += [np.array([1/2, 1/2])*grav for _ in range(self.Tend)]

        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(referenceForces)
        # plt.savefig("ref_forces.png")
        # plt.show()

        self.referenceForces = referenceForces
        return referenceForces