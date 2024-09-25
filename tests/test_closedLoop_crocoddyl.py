import unittest
import crocoddyl
import pinocchio as pin
import numpy as np

class TestContactLoop6D(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestContactLoop6D, self).__init__(*args, **kwargs)
        model = pin.buildSampleModelHumanoidRandom()
        self.robot = pin.RobotWrapper(model)
        self.contactIds = [
            self.robot.model.getFrameId("rleg6_joint"),
            self.robot.model.getFrameId("lleg6_joint"),
        ]
        # print([f.name for f in self.robot.model.frames])
        # print(self.robot.model.nframes, self.contactIds)
        self.state = crocoddyl.StateMultibody(self.robot.model)
        self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)
        self.contact_model = pin.RigidConstraintModel(
            pin.ContactType.CONTACT_6D,
            self.robot.model,
            self.robot.model.frames[self.contactIds[0]].parent,
            self.robot.model.frames[self.contactIds[0]].placement,
            self.robot.model.frames[self.contactIds[1]].parent,
            self.robot.model.frames[self.contactIds[1]].placement,
            pin.ReferenceFrame.LOCAL # Should be local or WORLD ?
        )
        Kp = 100
        Kd = 0
        self.contact_model.corrector.Kp[:] = Kp * np.ones(6)
        self.contact_model.corrector.Kd[:] = Kd * np.ones(6)
        self.contact_models = [self.contact_model]
        # self.contact_models = []
        self.costs = crocoddyl.CostModelSum(self.state, self.actuation.nu)
        self.contacts = crocoddyl.ContactModelMultiple(self.state, self.actuation.nu)
        contact = crocoddyl.ContactModel6DLoop(
            self.state,
            self.contact_model.joint1_id,
            self.contact_model.joint1_placement,
            self.contact_model.joint2_id,
            self.contact_model.joint2_placement,
            self.contact_model.reference_frame,
            self.actuation.nu,
            np.array([Kp, Kd])
        )
        self.contacts.addContact("contact", contact)
        self.dam = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation, self.contacts, self.costs)
        
    def pinocchio_dynamics(self, q, v, u):
        tau = np.concatenate([np.zeros(6), u])
        data = self.robot.model.createData()
        contact_datas = [cm.createData() for cm in self.contact_models]
        pin.initConstraintDynamics(self.robot.model, data, self.contact_models)
        a = pin.constraintDynamics(self.robot.model, data, q, v, tau, self.contact_models, contact_datas)
        return(a)

    def crocoddyl_dynamics(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        a = dam_data.xout
        return(a)
    
    def finite_differences(self, q, v, u, f):
        eps = 1e-7
        f0 = f(q, v, u).copy()
        Fq = np.zeros((f0.size, v.size))
        Fv = np.zeros((f0.size, v.size))
        Fu = np.zeros((f0.size, u.size))
        q_eps = np.zeros(v.size)
        for i in range(v.size):
            q_eps[i] = eps
            q1 = pin.integrate(self.robot.model, q, q_eps)
            Fq[:, i] = (f(q1, v, u) - f0) / eps
            q_eps[i] = 0
        v1 = v.copy()
        for i in range(v.size):
            v1[i] += eps
            Fv[:, i] = (f(q, v1, u) - f0) / eps
            v1[i] -= eps
        u1 = u.copy()
        for i in range(u.size):
            u1[i] += eps
            Fu[:, i] = (f(q, v, u1) - f0) / eps
            u1[i] -= eps
        return Fq, Fv, Fu

    def crocoddyl_derivatives(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        self.dam.calcDiff(dam_data, x, u)
        Fq = dam_data.Fx[:, :v.size]
        Fv = dam_data.Fx[:, v.size:]
        Fu = dam_data.Fu
        return Fq, Fv, Fu
    
    def pinocchio_derivatives(self, q, v, u):
        tau = np.concatenate([np.zeros(6), u])
        data = self.robot.model.createData()
        contact_datas = [cm.createData() for cm in self.contact_models]
        pin.initConstraintDynamics(self.robot.model, data, self.contact_models)
        pin.constraintDynamics(self.robot.model, data, q, v, tau, self.contact_models, contact_datas)
        pin.computeConstraintDynamicsDerivatives(self.robot.model, data, self.contact_models, contact_datas)
        Fq = data.ddq_dq
        Fv = data.ddq_dv
        Ftau = data.ddq_dtau
        Fu = Ftau[:, 6:]
        return Fq, Fv, Fu

    def pinocchio_aba(self, q, v, u):
        tau = np.concatenate([np.zeros(6), u])
        data = self.robot.model.createData()
        a = pin.aba(self.robot.model, data, q, v, tau)
        return a

    def pinocchio_aba_derivatives(self, q, v, u):
        tau = np.concatenate([np.zeros(6), u])
        data = self.robot.model.createData()
        a = pin.aba(self.robot.model, data, q, v, tau)#, convention=pin.Convention.LOCAL)
        pin.computeABADerivatives(self.robot.model, data, q, v, tau)
        Fq = data.ddq_dq
        Fv = data.ddq_dv
        Fu = data.Minv[:, 6:]
        return Fq, Fv, Fu

    def velocity_error(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        # contact_data = dam_data.contacts["contact"]
        d = dam_data.multibody.contacts.contacts["contact"]
        vel = (d.f1vf1 - d.f1vf2).vector
        return vel
    
    def velocity_error_derivatives(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        self.dam.calcDiff(dam_data, x, u)
        m = self.dam.contacts.contacts["contact"].contact
        d = dam_data.multibody.contacts.contacts["contact"]
        Fq = (
            m.joint1_placement.toActionMatrixInverse() @ d.v1_partial_dq 
            - (d.f1Mf2.act(d.f2vf2)).action @ (d.f1Jf1 - d.f1Xf2 @ d.f2Jf2) 
            - d.f1Xf2 @ m.joint2_placement.toActionMatrixInverse() @ d.v2_partial_dq
        )
        Fv = (d.f1Jf1 - d.f1Xf2 @ d.f2Jf2)
        return Fq, Fv
    
    def position_error(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        # contact_data = dam_data.contacts["contact"]
        d = dam_data.multibody.contacts.contacts["contact"]
        pos = 100 * (-(pin.log6(d.f1Mf2)).vector)
        return pos
    
    def position_error_derivatives(self, q, v, u):
        dam_data = self.dam.createData()
        x = np.concatenate([q, v])
        self.dam.calc(dam_data, x, u)
        self.dam.calcDiff(dam_data, x, u)
        m = self.dam.contacts.contacts["contact"].contact
        d = dam_data.multibody.contacts.contacts["contact"]
        oMf1 = d.pinocchio.oMi[m.joint1_id].act(m.joint1_placement)
        oMf2 = d.pinocchio.oMi[m.joint2_id].act(m.joint2_placement)
        Fq = 100*(-pin.Jlog6(d.f1Mf2) @ (-oMf2.toActionMatrixInverse()@oMf1.toActionMatrix()@d.f1Jf1 + d.f2Jf2))
        Fv = 0
        return Fq, Fv

    ## Testing Zone
    def test_dynamics(self):
        q = pin.randomConfiguration(self.robot.model)
        v = np.random.random(self.robot.model.nv)
        u = np.random.random(self.actuation.nu)
        a_pin = self.pinocchio_dynamics(q, v, u)
        a_croc = self.crocoddyl_dynamics(q, v, u)
        self.assertTrue(np.allclose(a_pin, a_croc))

    # def test_pinocchio_derivatives_toaba(self):
    #     q = pin.randomConfiguration(self.robot.model)
    #     v = np.random.random(self.robot.model.nv)
    #     u = np.random.random(self.actuation.nu)
    #     # Fq, Fv, Fu = self.finite_differences(q, v, u, self.pinocchio_dynamics)
    #     Fq_pin, Fv_pin, Fu_pin = self.pinocchio_derivatives(q, v, u)

    #     data_ref = self.robot.model.createData()
    #     tau = np.concatenate([np.zeros(6), u])
    #     pin.computeABADerivatives(self.robot.model, data_ref, q, v, tau)
    #     Fq = data_ref.ddq_dq.copy()
    #     Fv = data_ref.ddq_dv.copy()
    #     Fu = data_ref.Minv[:, 6:]

    #     self.assertTrue(np.allclose(Fq, Fq_pin))
    #     self.assertTrue(np.allclose(Fv, Fv_pin))
    #     self.assertTrue(np.allclose(Fu, Fu_pin))

    # def test_pinocchio_aba_derivatives_fd(self):
    #     q = pin.randomConfiguration(self.robot.model)
    #     v = np.random.random(self.robot.model.nv)
    #     u = np.random.random(self.actuation.nu)
    #     Fq_pin, Fv_pin, Fu_pin = self.pinocchio_aba_derivatives(q, v, u)
    #     Fq, Fv, Fu = self.finite_differences(q, v, u, self.pinocchio_aba)

    #     self.assertTrue(np.allclose(Fq, Fq_pin, atol=1e-4))
    #     self.assertTrue(np.allclose(Fv, Fv_pin, atol=1e-4))
    #     self.assertTrue(np.allclose(Fu, Fu_pin, atol=1e-4))

    # def test_pinocchio_derivatives_fd(self):
    #     q = pin.randomConfiguration(self.robot.model)
    #     v = np.random.random(self.robot.model.nv)
    #     u = np.random.random(self.actuation.nu)
    #     Fq, Fv, Fu = self.finite_differences(q, v, u, self.pinocchio_dynamics)
    #     Fq_pin, Fv_pin, Fu_pin = self.pinocchio_derivatives(q, v, u)

    #     self.assertTrue(np.allclose(Fq, Fq_pin, atol=1e-4))
    #     self.assertTrue(np.allclose(Fv, Fv_pin, atol=1e-4))
    #     self.assertTrue(np.allclose(Fu, Fu_pin, atol=1e-4))

    def test_velocity_error_derivatives(self):
        q = pin.randomConfiguration(self.robot.model)
        v = np.random.random(self.robot.model.nv)
        u = np.random.random(self.actuation.nu)
        Fq, Fv = self.velocity_error_derivatives(q, v, u)
        
        Fq_fd, Fv_fd, Fu_fd = self.finite_differences(q, v, u, self.velocity_error)

        self.assertTrue(np.allclose(Fq, Fq_fd, atol=1e-4))
        self.assertTrue(np.allclose(Fv, Fv_fd, atol=1e-4))

    def test_position_error_derivatives(self):
        q = pin.randomConfiguration(self.robot.model)
        v = np.random.random(self.robot.model.nv)
        u = np.random.random(self.actuation.nu)
        Fq, Fv = self.position_error_derivatives(q, v, u)
        
        Fq_fd, Fv_fd, Fu_fd = self.finite_differences(q, v, u, self.position_error)

        self.assertTrue(np.allclose(Fq, Fq_fd, atol=1e-4))
        self.assertTrue(np.allclose(Fv, Fv_fd, atol=1e-4))

    def test_crocoddyl_derivatives(self):
        q = pin.randomConfiguration(self.robot.model)
        v = np.random.random(self.robot.model.nv)
        u = np.random.random(self.actuation.nu)
        x = np.concatenate([q, v])
        Fq, Fv, Fu = self.finite_differences(q, v, u, self.crocoddyl_dynamics)

        dam_data = self.dam.createData()
        self.dam.calc(dam_data, x, u)
        self.dam.calcDiff(dam_data, x, u)

        self.assertTrue(np.allclose(Fq, dam_data.Fx[:, :v.size], atol=1e-3))
        self.assertTrue(np.allclose(Fv, dam_data.Fx[:, v.size:], atol=1e-3))
        self.assertTrue(np.allclose(Fu, dam_data.Fu, atol=1e-4))

if __name__ == '__main__':
    unittest.main()