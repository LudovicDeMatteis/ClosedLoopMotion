import pinocchio as pin
import numpy as np
import matplotlib.pylab as plt  # noqa: F401
from numpy.linalg import norm, pinv, inv, svd, eig  # noqa: F401
import loaders 
from toolbox_parallel_robots.projections import configurationProjection
import os
CWD = os.path.dirname(os.path.abspath(__file__))

base_height = 0.600

robot = loaders.digit_open()
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

q_ref = np.array([ 0.      , -0.      ,  0.65    ,  0.      , -0.      , -0.      ,  1.      ,  0.000023,  0.023788, -0.489286, -0.166051,  0.000113, -1.782625,  0.014427, -0.000117,  0.023706,  0.480951,  0.16038 ,  0.014849,  1.765223, -0.014264])

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
# q_ref = pin.neutral(model)

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

viz.display(q_ref)
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

np.save(f"{CWD}/initial_configs_digit/q0_{str(base_height).replace('.', '_')}.npy", q0)
print(f"Saved to q0_{str(base_height).replace('.', '_')}.npy")