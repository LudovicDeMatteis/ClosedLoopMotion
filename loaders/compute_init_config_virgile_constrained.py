import pinocchio as pin
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
from loaders_virgile import load_complete_open
from toolbox_parallel_robots.projections import configurationProjection
import os
CWD = os.path.dirname(os.path.abspath(__file__))

base_height = 0.605

robot = load_complete_open()
model = robot.model
collision_model = robot.collision_model
visual_model = robot.visual_model

data = model.createData()
q_ref = robot.x0[:model.nq]

import meshcat
from pinocchio.visualize import MeshcatVisualizer
viz = MeshcatVisualizer(model, collision_model, visual_model)
server = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
viz.initViewer(loadModel=True, viewer=server)

def generateContactModels(model, contactIds, q_ref):
      data = model.createData()
      contact_constraints_models = []
      for cId in contactIds:
            pin.framesForwardKinematics(model, data, q_ref)
            floorContactPositionLeft = data.oMf[cId].translation
            floorContactPositionLeft[0] = 0
            floorContactPositionLeft[2] = 0
            MContactPlacement = pin.SE3(
            pin.utils.rotate("x", 0.0), floorContactPositionLeft
            )  # SE3 position of the contact
            footFloorConstraint = pin.RigidConstraintModel(
            pin.ContactType.CONTACT_6D,
            model,
            model.frames[cId].parentJoint,
            model.frames[cId].placement,
            0,  # To the world
            MContactPlacement,
            pin.ReferenceFrame.LOCAL,
            )
            contact_constraints_models.append(footFloorConstraint)
      return(contact_constraints_models)

contactIds = robot.contactIds
contact_constraints_models = generateContactModels(model, contactIds, q_ref)
baseId = model.getFrameId("torso")
q_ref = pin.neutral(model)

MBasePlacement = pin.SE3.Identity()
MBasePlacement.translation = np.array([0, 0, base_height])
base_cstr_model = pin.RigidConstraintModel(
      pin.ContactType.CONTACT_6D,
      model,
      model.frames[baseId].parentJoint,
      model.frames[baseId].placement,
      0,  # To the world
      MBasePlacement,
      pin.ReferenceFrame.LOCAL,
      )
contact_constraints_datas = [cm.createData() for cm in contact_constraints_models]
base_cstr_data = base_cstr_model.createData()

q0 = configurationProjection(
      model,
      data,
      contact_constraints_models + [base_cstr_model],
      contact_constraints_datas + [base_cstr_data],
      q_prec=q_ref,
)
viz.display(q0)
print(q0)

pin.centerOfMass(model, data, q0)
print(f"CoM: {data.com[0]}")

np.save(f"{CWD}/../results/initial_configs/q0_{str(base_height).replace('.', '_')}.npy", q0)
print(f"Saved to q0_{str(base_height).replace('.', '_')}.npy")