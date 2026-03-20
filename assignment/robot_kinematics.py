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
            't5': (-np.pi, np.pi),
        }

        # Build Symbolic Forward Kinematics Transform
        self._symbolic_FK_transform = self._build_symbolic_transform()
        
        # Extract EE Position
        self._symbolic_xyz_pos = self._symbolic_FK_transform[:3, 3]
        #self._symbolic_pitch = self.t2 + self.t3 + self.t4 - sp.pi/2  # Approximation: sum of joint angles for pitch
        #self._symbolic_roll = self.t5                                 # Approximation: last joint angle 

        # Extract EE Orientation - ZXY Euler angles — yaw(Z) → pitch(X) → roll(Y)
        r01 = self._symbolic_FK_transform[0, 1]
        r11 = self._symbolic_FK_transform[1, 1]        
        r20 = self._symbolic_FK_transform[2, 0]
        r21 = self._symbolic_FK_transform[2, 1]
        r22 = self._symbolic_FK_transform[2, 2]

        # ZXY: R = Rz(yaw) · Rx(pitch) · Ry(roll)
        self._symbolic_pitch = sp.atan2(r21, sp.sqrt(r20**2 + r22**2))  # about X
        self._symbolic_roll  = sp.atan2(-r20, r22)                      # about Y
        self._symbolic_yaw   = sp.atan2(-r01, r11)                       # about Z 
        

        # Combine  EE pose into a 5-element output vector: [x, y, z, pitch, roll, yaw]
        self._symbolic_EE_pose = sp.Matrix([*self._symbolic_xyz_pos, self._symbolic_pitch,self._symbolic_roll, self._symbolic_yaw])  # Using the first solution branch for pitch and roll

        # Build Symbolic Jacobian Matrix: J = d(EE_pose)/d(joint_vars)]
        self._symbolic_jacobian = self._symbolic_EE_pose.jacobian(sp.Matrix(self.joint_vars))

        # Build Numerical Forward kinematics model
        self._numeric_EE_pose = lambdify(args = self.joint_vars, expr = self._symbolic_EE_pose, modules='numpy')

        # Build Numerical Jacobian
        self._numeric_jacobian = lambdify(args = self.joint_vars, expr = self._symbolic_jacobian, modules='numpy')
        
        """

        # Build Symbolic Inverse Kinematics model: Jacobian J = d(EE_pose)/d(joint_vars)
        # J is a 5x5 matrix where J[i,j] = d(EE_pose[i])/d(joint_vars[j])
        # IK update rule: delta_q = J^+ * delta_EE  (J^+ = pseudo-inverse of J)
        self._symbolic_IK_model = self._symbolic_EE_pose.jacobian(sp.Matrix(self.joint_vars))

        # Numerical Jacobian for use in IK solvers
        self._numeric_IK_model = lambdify(args = self.joint_vars, expr = self._symbolic_IK_model, modules='numpy')

        """

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

    def _build_symbolic_transform(self):
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
        T_lowerarm_wrist = self._sym_trans(0.0052, -0.1349, 0) * self._sym_rot_z(sp.pi/2) * self._sym_rot_z(self.t4)
        
        # TO: Wrist     --- FROM: Gripper
        T_wrist_gripper = self._sym_trans(-0.0601, 0, 0) * self._sym_rot_y(-sp.pi/2) * self._sym_rot_z(self.t5) 

        # TO: Gripper     --- FROM: Gripper Center
        T_gripper_grippercenter = self._sym_trans(0, 0, 0.075)
        
        # Full Chain
        return T_world_base * T_base_shoulder * T_shoulder_upperarm * T_upperarm_lowerarm * T_lowerarm_wrist * T_wrist_gripper * T_gripper_grippercenter

    def _check_joint_limits(self, q1, q2, q3, q4, q5):
        """Checks if the given joint angles are within the defined limits."""
        for i, q in enumerate([q1, q2, q3, q4, q5]):
            low, high = self.joint_bounds[self.joint_keys[i]]
            if np.any(q < low) or np.any(q > high):
                raise ValueError(
                    f"Joint {self.joint_keys[i]} is out of bounds! "
                    f"Input: {q}, Allowed Range: [{low}, {high}]"
                )
            

    def forward_kinematics(self, q1, q2, q3, q4, q5):
        """
        If joint paramenters are within limits, returns the EE position as a 3D vector.
        Otherwise, raises ValueError.
        """ 

        # Ensure input joint angles are within allowed limits before computing FK
        self._check_joint_limits(q1, q2, q3, q4, q5)
            
        return self._numeric_EE_pose(q1, q2, q3, q4, q5)

    def jacobian(self, q1, q2, q3, q4, q5):
        """Returns the 5x5 Jacobian matrix at the given joint configuration."""
        self._check_joint_limits(q1, q2, q3, q4, q5)
        return self._numeric_jacobian(q1, q2, q3, q4, q5)
    

    def jacobian_inverse(self, current_joint_angles, 
                            singularity_threshold=0.05, lambda_max=0.1):
        """
        Computes joint angle velocities from a task-space desired velocity using the
        Damped Least Squares (DLS) pseudo-inverse of the Jacobian (and prevent singularities).

        For that , the DLS replaces 1/σ_i with σ_i/(σ_i² + λ²), which keeps
        the solution bounded at the cost of a small positional error 

        Based on Nakamura–Hanafusa variable damping rule.
        """
        # Compute Jacobian at current configuration (already a numpy array due to lambdify)
        J = self._numeric_jacobian(current_joint_angles)

        # Compute SVD decompositionof the Jacobian
        U, singular_values, Vt = np.linalg.svd(J)

        # Singularity analysis - High confition values/low eigen values indicate proximity to a singularity.
        sigma_min = singular_values[-1]
        is_singular = sigma_min < singularity_threshold

        # Prepare adaptive DLS damping (Nakamura–Hanafusa 1986) ---
        if is_singular:
            # λ² scales from 0 (at threshold) to lambda_max² (at σ=0)
            lambda_sq = lambda_max**2 * (1.0 - (sigma_min / singularity_threshold)**2)
        else:
            lambda_sq = 0.0

        # Damped pseudo-inverse via SVD: σ_i / (σ_i² + λ²)
        damped_inv_sigmas = singular_values / (singular_values**2 + lambda_sq)

        # Compute target joint velocities
        inverse_jacobian = Vt.T @ np.diag(damped_inv_sigmas) @ U.T

        return inverse_jacobian, is_singular

    
    def inverse_kinematics(self, target_pose, initial_guess = [0, 0, 0, 0, 0], n_restarts=50, error_threshold=1e-4, dedup_tol=0.05):
        """
        Solves IK for [x, y, z, pitch, roll] using constrained optimization (SLSQP).
        Returns a list of all distinct joint configurations that reach the target pose.
        Each configuration is a 5-element list [q1, q2, q3, q4, q5].
        Returns an empty list if no solution is found.
        """
        def objective_function(q):
            current_pose = np.array(self._numeric_EE_pose(*q)).flatten()[:len(target_pose)]  # Get current EE pose for the given joint angles
            weights = np.array([10.0, 10.0, 10.0, 1.0, 1.0])
            return np.sum(((current_pose - np.array(target_pose))**2))

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
        resolution = 17  # Number of points per joint
        
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