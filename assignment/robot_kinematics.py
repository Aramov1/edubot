import numpy as np
import sympy as sp
from sympy.utilities.lambdify import lambdify
from scipy.optimize import minimize

class RobotKinematics():
    def __init__(self):
        # Define robot configuration parameters as symbolic variables
        self.joint_keys = ['t1', 't2', 't3', 't4', 't5']
        self.t1, self.t2, self.t3, self.t4, self.t5 = sp.symbols('theta_1 theta_2 theta_3 theta_4 theta_5')
        self.joint_vars = [self.t1, self.t2, self.t3, self.t4, self.t5]

        # Define joint limits (in radians)
        self.joint_bounds = {
            't1': (-2.0, 2.0),
            't2': (-np.pi/2, np.pi/2),
            't3': (-np.pi/2, np.pi/2),
            't4': (-np.pi/2, np.pi/2),
            't5': (-np.pi/2, np.pi/2),
        }

        # Build Symbolic forward kinematics model
        self._symbolic_FK_model = self._build_symbolic_model()
        
        # Extract EE position and orienteation
        self._symbolic_xyz_pos = self._symbolic_FK_model[:3, 3]

        r20, r21, r22 = self._symbolic_FK_model[2, 0], self._symbolic_FK_model[2, 1], self._symbolic_FK_model[2, 2]
        self._symbolic_pitch_roll = [sp.asin(-r20), sp.atan2(r21, r22)] # Standard Y-X convention for a 5-DOF arm: Pitch = arcsin(-R[2,0]), Roll = atan2(R[2,1], R[2,2])

        # Combine  EE pose into a 5-element output vector: [x, y, z, pitch, roll]
        self._symbolic_EE_pose = sp.Matrix([*self._symbolic_xyz_pos, *self._symbolic_pitch_roll])

        # Build Numerical forward kinematics model
        self._numeric_FK_model = lambdify(args = self.joint_vars, expr = self._symbolic_EE_pose, modules='numpy')

        # Build Symbolic Inverse Kinematics model: Jacobian J = d(EE_pose)/d(joint_vars)
        # J is a 5x5 matrix where J[i,j] = d(EE_pose[i])/d(joint_vars[j])
        # IK update rule: delta_q = J^+ * delta_EE  (J^+ = pseudo-inverse of J)
        self._symbolic_IK_model = self._symbolic_EE_pose.jacobian(sp.Matrix(self.joint_vars))

        # Numerical Jacobian for use in IK solvers
        self._numeric_IK_model = lambdify(args = self.joint_vars, expr = self._symbolic_IK_model, modules='numpy')


    def _sym_rot_x(self, theta):
        return sp.Matrix([[1,  0,              0,             0],
                          [0,  sp.cos(theta), -sp.sin(theta), 0],
                          [0,  sp.sin(theta),  sp.cos(theta), 0],
                          [0,  0,              0,             1]])
    
    def _sym_rot_y(self, theta):
        return sp.Matrix([[sp.cos(theta),  0, sp.sin(theta), 0],
                          [0,             1, 0,             0],
                          [-sp.sin(theta), 0, sp.cos(theta), 0],
                          [0,              0, 0,             1]])

    def _sym_rot_z(self, theta):
        return sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, 0],
                          [sp.sin(theta),  sp.cos(theta), 0, 0],
                          [0,              0,             1, 0],
                          [0,              0,             0, 1]])

    def _sym_trans(self, x, y, z):
        return sp.Matrix([[1, 0, 0, x],
                          [0, 1, 0, y],
                          [0, 0, 1, z],
                          [0, 0, 0, 1]])

    def _build_symbolic_model(self):
        """Constructs the symbolic forward kinematics chain."""
        
        # TO: World     --- FROM: Base
        T_world_base = self._sym_trans(0, 0, 0) * self._sym_rot_z(sp.pi)
        
        # TO: Base      --- FROM: Shoulder (Rotation t1)
        T_base_shoulder = self._sym_trans(0, -0.0452, 0.0165) * self._sym_rot_z(self.t1)
        
        # TO: Shoulder  --- FROM: Upperarm (Rotation t2)
        T_shoulder_upperarm = self._sym_trans(0, -0.0306, 0.1025) * self._sym_rot_y(-sp.pi/2) * self._sym_rot_z(self.t2) 
        
        # TO: Upperarm  --- FROM: Lowerarm (Rotation t3)
        T_upperarm_lowerarm = self._sym_trans(0.11257, -0.028, 0) * self._sym_rot_z(self.t3) 
        
        # TO: Lowerarm  --- FROM: Wrist (Rotation t4)
        T_lowerarm_wrist = self._sym_trans(0.0052, -0.1349, 0) * self._sym_rot_y(sp.pi/2) * self._sym_rot_z(self.t4)
        
        # TO: Wrist     --- FROM: Gripper
        T_wrist_gripper = self._sym_trans(-0.0601, 0, 0) * self._sym_rot_y(-sp.pi/2) * self._sym_rot_z(self.t5) 

        # TO: Gripper     --- FROM: Gripper Center
        T_gripper_grippercenter = self._sym_trans(0, 0, 0.075)
        
        # Full Chain
        return T_world_base * T_base_shoulder * T_shoulder_upperarm * T_upperarm_lowerarm * T_lowerarm_wrist * T_wrist_gripper * T_gripper_grippercenter

    def forward_kinematics(self, q1, q2, q3, q4, q5):
        """
        If joint paramenters are within limits, returns the EE position as a 3D vector.
        Otherwise, raises ValueError.
        """ 
        
        for i, q in enumerate([q1, q2, q3, q4, q5]):
            low, high = self.joint_bounds[self.joint_keys[i]]
            
            # Use np.any to handle both single floats and arrays from meshgrid
            if np.any(q < low) or np.any(q > high):
                raise ValueError(
                    f"Joint {self.joint_keys[i]} is out of bounds! "
                    f"Input: {q}, Allowed Range: [{low}, {high}]"
                )
            
        # Return the computed forward kinematics matrix if joint values within limits
        return self._numeric_FK_model(q1, q2, q3, q4, q5)

    
    def trial_inverse_kinematics(self, target_pose, n_restarts=50, error_threshold=1e-3, dedup_tol=0.05):
        """
        Solves IK for [x, y, z, pitch, roll] using constrained optimization (SLSQP).
        Returns a list of all distinct joint configurations that reach the target pose.
        Each configuration is a 5-element list [q1, q2, q3, q4, q5].
        Returns an empty list if no solution is found.
        """
        def objective_function(q):
            current_pose = np.array(self._numeric_FK_model(q)).flatten()
            weights = np.array([10.0, 10.0, 10.0, 1.0, 1.0])
            return np.sum(((current_pose - np.array(target_pose))) ** 2 *)

        bounds = list(self.joint_bounds.values())

        # No fixed seed: each call explores a different random region of joint space
        rng = np.random.default_rng()
        guesses = [np.zeros(len(self.joint_vars))]
        for _ in range(n_restarts):
            guesses.append([rng.uniform(b[0], b[1]) for b in bounds])

        solutions = []

        for guess in guesses:
            res = minimize(
                objective_function,
                guess,
                method='SLSQP',
                bounds=bounds,
                tol=1e-8,
                options={'maxiter': 500}   # enough iterations to actually converge
            )

            # Accept any candidate below the error threshold regardless of res.success,
            # because SLSQP can hit maxiter while still being very close to the solution
            if res.fun > error_threshold:
                continue

            candidate = res.x

            # Deduplicate: skip if this configuration is close to one already collected
            is_duplicate = any(np.allclose(candidate, sol, atol=dedup_tol) for sol in solutions)
            if not is_duplicate:
                solutions.append(candidate)

        return [sol.tolist() for sol in solutions]


    def compute_workspace(self):
        """Computes the xyz coordinates for a range of joint angles."""
        resolution = 30  # Number of points per joint
        
        t1_space = np.linspace(*self.joint_bounds['t1'], resolution)
        t2_space = np.linspace(*self.joint_bounds['t2'], resolution)
        t3_space = np.linspace(*self.joint_bounds['t3'], resolution)
        t4_space = np.linspace(*self.joint_bounds['t4'], resolution)
        t5_space = np.linspace(*self.joint_bounds['t5'], resolution)

        # Create a grid of all joint combinations
        t1, t2, t3, t4, t5 = np.meshgrid(t1_space, t2_space, t3_space, t4_space, t5_space, indexing='ij')
        
        # Flatten for calculation and extract x, y, z (first 3 of the 5-element FK output)
        result = self.forward_kinematics(t1.flatten(), t2.flatten(), t3.flatten(), t4.flatten(), t5.flatten())
        x, y, z = result[0], result[1], result[2]
        
        return x, y, z