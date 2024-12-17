import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat
import numpy as np
from example_parallel_robots.loader_tools import load
from toolbox_parallel_robots.freeze_joints import freezeJoints
from toolbox_parallel_robots.projections import configurationProjection
from dataclasses import dataclass
import sys

sys.path.append("../utils")


class LoaderBattobot:
    def __init__(self):
        # Main model attributes
        self.model = None
        self.actuation_model = None
        self.constraint_models = None
        self.visual_model = None
        self.collision_model = None
        self.model_type = ""  # Should be "6D", "3D" or "simplified"
        self.q0 = None
        self.q0_simplified = None
        self.loop_joints_ids_q = []
        self.loop_joints_ids_v = []
        self.serial_joints_ids_q = []
        self.serial_joints_ids_v = []
        # Task attributes
        self.contact_frames = ["foot_frame_right", "foot_frame_left"]
        self.base_frame = "torso"
        self.joints_lock_names = [
            # Right
            "left_spherical_foot_1",
            "left_spherical_foot_2",
            "free_knee_left_Y",
            "free_knee_left_Z",
            "motor_ankle1_left",
            "left_spherical_ankle_1_Y",
            "left_spherical_ankle_1_Z",
            "motor_ankle2_left",
            "left_spherical_ankle_2_Y",
            "left_spherical_ankle_2_Z",
            "motor_knee_left",
            "transmission_knee_left",
            # Right
            "right_spherical_foot_1",
            "right_spherical_foot_2",
            "free_knee_right_Y",
            "free_knee_right_Z",
            "motor_ankle1_right",
            "right_spherical_ankle_1_Y",
            "right_spherical_ankle_1_Z",
            "motor_ankle2_right",
            "right_spherical_ankle_2_Y",
            "right_spherical_ankle_2_Z",
            "motor_knee_right",
            "transmission_knee_right",
        ]

    def load_urdf(self):
        # Load the model from example_parallel_robots
        (
            self.model,
            self.robot_constraint_models,
            self.actuation_model,
            self.visual_model,
            self.collision_model,
        ) = load("battobot", free_flyer=True)
        self.model_type = "6D"
        # Define the frame that will be in contact with the floor
        self.model.frames[15].name = "foot_frame_right"
        self.model.frames[62].name = "foot_frame_left"
        # Define initial configuration
        # TODO This should be read from the urdf
        self.q0_simplified = pin.neutral(self.model)
        # Define loop joints and serial joints
        for i, name in enumerate(self.joints_lock_names):
            jId = self.model.getJointId(name)
            for niq in range(self.model.joints[jId].nq):
                self.loop_joints_ids_q.append(self.model.joints[jId].idx_q + niq)
            for niv in range(self.model.joints[jId].nv):
                self.loop_joints_ids_v.append(self.model.joints[jId].idx_v + niv)
        self.serial_joints_ids_q = [
            i for i in range(self.model.nq) if i not in self.loop_joints_ids_q
        ]
        self.serial_joints_ids_v = [
            i for i in range(self.model.nv) if i not in self.loop_joints_ids_v
        ]

    def get_simplified_model(self, use_tmp=False, qref=None):
        jointToLockIds = [
            i for (i, n) in enumerate(self.model.names) if n in self.joints_lock_names
        ]
        if not use_tmp:
            (
                self.model,
                _,
                _,
                self.visual_model,
                self.collision_model,
            ) = freezeJoints(
                self.model,
                self.robot_constraint_models,
                self.actuation_model,
                self.visual_model,
                self.collision_model,
                jointToLockIds,
                qref if qref is not None else pin.neutral(self.model),
            )
            self.robot_constraint_models = []
            self.actuation_model = None  # WARN The actuation model is set to be None when the model is fully actuated
            self.model_type = "simplified"
        else:
            (
                self.model_tmp,
                _,
                _,
                self.visual_model_tmp,
                self.collision_model_tmp,
            ) = freezeJoints(
                self.model.copy(),
                [],
                self.actuation_model,
                self.visual_model.copy(),
                self.collision_model.copy(),
                jointToLockIds,
                qref if qref is not None else pin.neutral(self.model),
            )
            self.robot_constraint_models_tmp = []
            self.actuation_model_tmp = None  # WARN The actuation model is set to be None when the model is fully actuated

    def compute_simplified_q0(self, qref=None, use_tmp=False):
        assert (
            self.model_type == "simplified" or use_tmp
        ), "The model should be simplified to get the initail configuration"
        # Run an optimisation process to find the initial configuration of the robot
        base_height = 0.605  # Criterion could be the knee angle

        def generate_contact_models(model, contactIds, qref):
            data = model.createData()
            contact_constraints_models = []
            pin.framesForwardKinematics(model, data, qref)
            for cId in contactIds:
                floorContactPositionLeft = data.oMf[cId].translation
                floorContactPositionLeft[0] = 0
                floorContactPositionLeft[2] = 0
                MContactPlacement = pin.SE3(
                    pin.utils.rotate("x", 0.0), floorContactPositionLeft
                )
                foot_floor_constraint = pin.RigidConstraintModel(
                    pin.ContactType.CONTACT_6D,
                    model,
                    model.frames[cId].parentJoint,
                    model.frames[cId].placement,
                    0,
                    MContactPlacement,
                    pin.ReferenceFrame.LOCAL,
                )
                contact_constraints_models.append(foot_floor_constraint)
            return contact_constraints_models

        model = self.model_tmp if use_tmp else self.model

        contact_constraints_models = generate_contact_models(
            model,
            [model.getFrameId(fname) for fname in self.contact_frames],
            qref if qref is not None else pin.neutral(model),
        )
        MBasePlacement = pin.SE3.Identity()
        MBasePlacement.translation = np.array([0, 0, base_height])
        base_id = model.getFrameId(self.base_frame)
        base_cstr_model = pin.RigidConstraintModel(
            pin.ContactType.CONTACT_6D,
            model,
            model.frames[base_id].parentJoint,
            model.frames[base_id].placement,
            0,
            MBasePlacement,
            pin.ReferenceFrame.LOCAL,
        )
        constraints = contact_constraints_models + [base_cstr_model]
        constraints_data = [c.createData() for c in constraints]
        data = model.createData()
        self.q0_simplified = configurationProjection(
            model, data, constraints, constraints_data, qref
        )
        if not use_tmp:
            self.q0 = self.q0_simplified

    def compute_complete_q0(self, recompute_simplified=False):
        assert (
            self.model_type == "6D" or self.model_type == "3D"
        ), "The model should be 6D to get the initial configuration"
        if recompute_simplified:
            self.compute_simplified_q0()
        qref = pin.neutral(self.model)
        qref[self.serial_joints_ids_q] = self.q0_simplified

        robot_constraint_data = [c.createData() for c in self.robot_constraint_models]
        data = self.model.createData()
        w = np.ones(self.model.nv)
        w[self.serial_joints_ids_v] = 1
        W = np.diag(w)
        self.q0 = configurationProjection(
            self.model,
            data,
            self.robot_constraint_models,
            robot_constraint_data,
            qref,
            W,
        )

    def convert_6D_to_3D(self, qref=None):
        assert self.model_type == "6D", "The model should be 6D to convert it to 3D"
        # For this function, I should do certain things:
        # * Add the lower bars inertia to the parent bar
        # * Add the visual (and collision) of the lower bars to the parent bar
        # * Remove the ball joint at the lower bars
        # * Add frames at the end of the bars
        # * Define constraints on these frames
        data = self.model.createData()
        pin.forwardKinematics(self.model, data, qref)

        if qref is None:
            qref = self.q0
        # Define the relevant joints names
        spherical_joints_names = [
            "left_spherical_foot_1",
            "left_spherical_foot_2",
            "right_spherical_foot_1",
            "right_spherical_foot_2",
        ]
        spherical_joints_ids = [
            self.model.getJointId(name) for name in spherical_joints_names
        ]
        spherical_joints_transforms = [data.oMi[jid] for jid in spherical_joints_ids]
        ujoints_names = [
            "left_spherical_ankle_1_Y",
            "left_spherical_ankle_2_Y",
            "right_spherical_ankle_1_Y",
            "right_spherical_ankle_2_Y",
        ]
        ujoints_ids = [self.model.getJointId(name) for name in ujoints_names]
        ujoints_transforms = [data.oMi[jid] for jid in ujoints_ids]
        # Get the transformation from upper bars to lower bars
        transforms = [
            ujoints_transforms[i].actInv(spherical_joints_transforms[i])
            for i in range(len(spherical_joints_transforms))
        ]
        # Add the lower bars inertia to the parent bar


def visualize_constraints(viz, loader, idx=None, q=None):
    from utils.vizutils import visualizeConstraints

    if idx is not None and isinstance(idx, list):
        constraint_models = [loader.robot_constraint_models[i] for i in idx]
    elif idx is not None and isinstance(idx, int):
        constraint_models = [loader.robot_constraint_models[idx]]
    else:
        constraint_models = loader.robot_constraint_models
    visualizeConstraints(
        viz, loader.model, loader.model.createData(), constraint_models, q
    )


def create_viewer(robot, adress="127.0.0.1", port=6000, open=False):
    try:
        import meshcat
        from pinocchio.visualize import MeshcatVisualizer

        viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
        if open:
            viz.initViewer(open=True)
        else:
            viz.viewer = meshcat.Visualizer(zmq_url="tcp://{}:{}".format(adress, port))
        viz.clean()
        viz.loadViewerModel(rootNodeName="universe")

        return viz
    except (ImportError, AttributeError):
        print("No viewer")

        return None


def create_sliders(robot, viz, qref):
    from toolbox_parallel_robots.foo import createSlidersInterface

    createSlidersInterface(
        robot.model,
        robot.constraint_models,
        robot.visual_model,
        robot.loop_joints_ids_v,
        viz,
        qref,
    )


if __name__ == "__main__":
    loader = LoaderBattobot()
    loader.load_urdf()
    loader.get_simplified_model(use_tmp=True)
    loader.compute_simplified_q0(use_tmp=True)
    loader.compute_complete_q0(recompute_simplified=False)

    # Add missing inertias in the model ujoints
    viz = create_viewer(loader)
    q0 = loader.q0  # pin.neutral(loader.model)
    # q0 = pin.neutral(loader.model)
    viz.display(q0)

    visualize_constraints(viz, loader, q=q0)
    create_sliders(loader, viz, q0)
