import numpy as np
import sympy as sp
from sympy.utilities.lambdify import lambdify
from scipy.optimize import minimize

# Limited Joint Conf
JOINT_CONFIG_LIMITS = {
    'Shoulder_Rotation': (-2.0, 2.0),
    'Shoulder_Pitch':    (-np.pi/2, np.pi/2),
    'Elbow':             (-np.pi/2, np.pi/2),
    'Wrist_Pitch':       (-np.pi/2, np.pi/2),
    'Wrist_Roll':        (-np.pi, np.pi),
}

# # Unlimited Joint Conf
# JOINT_CONFIG_LIMITS = {
#     'Shoulder_Rotation': (-np.pi, 2np.pi),
#     'Shoulder_Pitch':    (-np.pi, 2np.pi),
#     'Elbow':             (-np.pi, 2np.pi),
#     'Wrist_Pitch':       (-np.pi, 2np.pi),
#     'Wrist_Roll':        (-np.pi, 2np.pi),
# }

class RobotKinematics():
    def __init__(self):
        # Joint Configuration Symbols
        self.joint_vars = sp.symbols('q1:6')
        self.joint_names = list(JOINT_CONFIG_LIMITS.keys())
        self.joint_bounds = list(JOINT_CONFIG_LIMITS.values())
        
        # Build Symbolic Transform 
        self._symbolic_transform = self._build_symbolic_transform()

        # Build Symbolic FK
        self._symbolic_fk = self._build_symbolic_fk()

        # Build Symbolic Jacobian
        self._symbolic_jac = self._symbolic_fk.jacobian(self.joint_vars)

        # Build Numerical FK
        self._numeric_fk = lambdify(args = self.joint_vars, expr = self._symbolic_fk, modules='numpy')

        # Build Numerical Jacobian
        self._numeric_jac = lambdify(args = self.joint_vars, expr = self._symbolic_jac, modules='numpy')

    def _sym_rot_x(self, t):
        return sp.Matrix([[1,  0,              0,             0],
                          [0,  sp.cos(t), -sp.sin(t), 0],
                          [0,  sp.sin(t),  sp.cos(t), 0],
                          [0,  0,              0,             1]])
    
    def _sym_rot_y(self, t):
        return sp.Matrix([[sp.cos(t),  0, sp.sin(t), 0],
                          [0,              1, 0,             0],
                          [-sp.sin(t), 0, sp.cos(t), 0],
                          [0,              0, 0,             1]])

    def _sym_rot_z(self, t):
        return sp.Matrix([[sp.cos(t), -sp.sin(t), 0, 0],
                          [sp.sin(t),  sp.cos(t), 0, 0],
                          [0,              0,             1, 0],
                          [0,              0,             0, 1]])

    def _sym_trans(self, x, y, z):
        return sp.Matrix([[1, 0, 0, x],
                          [0, 1, 0, y],
                          [0, 0, 1, z],
                          [0, 0, 0, 1]])

    def _build_symbolic_transform(self):
        
        # Extract Joint Symbols
        q1, q2, q3, q4, q5 = self.joint_vars

        # TO: World     --- FROM: Base
        T_world_base = self._sym_trans(0, 0, 0) * self._sym_rot_z(sp.pi)
        
        # TO: Base      --- FROM: Shoulder (Rotation q1)
        T_base_shoulder = self._sym_trans(0, -0.0452, 0.0165) * self._sym_rot_z(q1)
        
        # TO: Shoulder  --- FROM: Upperarm (Rotation q2)
        T_shoulder_upperarm = self._sym_trans(0, -0.0306, 0.1025) * self._sym_rot_y(-sp.pi/2) * self._sym_rot_z(q2) 
        
        # TO: Upperarm  --- FROM: Lowerarm (Rotation q3)
        T_upperarm_lowerarm = self._sym_trans(0.11257, -0.028, 0) * self._sym_rot_z(q3) 
        
        # TO: Lowerarm  --- FROM: Wrist (Rotation q4)
        T_lowerarm_wrist = self._sym_trans(0.0052, -0.1349, 0) * self._sym_rot_z(sp.pi/2) * self._sym_rot_z(q4)
        
        # TO: Wrist     --- FROM: Gripper (Rotation q5)
        T_wrist_gripper = self._sym_trans(-0.0601, 0, 0) * self._sym_rot_y(-sp.pi/2) * self._sym_rot_z(q5) 

        # TO: Gripper     --- FROM: Gripper Center
        T_gripper_grippercenter = self._sym_trans(0, 0, 0.075)
        
        # Full  kinematic Chain
        return T_world_base * T_base_shoulder * T_shoulder_upperarm * T_upperarm_lowerarm * T_lowerarm_wrist * T_wrist_gripper * T_gripper_grippercenter

    def _build_symbolic_fk(self):
        
        # Extract EE Pose
        pos = self._symbolic_transform[:3, 3]
        rot = self._symbolic_transform[:3, :3]
        
        # Compute Euler Angles (Extrinsic XYZ / Intrinsic zyx)
        rot_x = sp.atan2(rot[2, 1], rot[2, 2])                               # rot_x, about X   
        rot_y = sp.atan2(-rot[2, 0], sp.sqrt(rot[2, 1]**2 + rot[2, 2]**2))   # rot_y, about Y
        rot_z = sp.atan2(rot[1, 0], rot[0, 0])                               # rot_z, about Z

        return sp.Matrix([*pos, rot_x, rot_y, rot_z])  

    def _check_joint_limits(self, joints):
        # Check number of joint configurations
        if len(joints) != len(self.joint_names):
            raise ValueError(f"Joint Configuration Refused. Expected 5 values, got {len(joints)}")
        
        for i, q in enumerate(joints):
            low, high = self.joint_bounds[i]
            if not np.all((q >= low) & (q <= high)):
                raise ValueError(f"Joint Configuration Refused. {self.joint_names[i]} outside [{low:.3f}, {high:.3f}]")
        
    def forward_kinematics(self, joints):
        """Compute 6-DOF pose, if joint configuration within bounds.

        Scalar inputs  → returns shape (6,)   [x, y, z, rx, ry, rz]
        Vectorized inputs → returns shape (6, N) [row per DOF, column per sample]
        """
        self._check_joint_limits(joints)
        # squeeze: scalar (6,1)→(6,) | vectorized (6,1,N)→(6,N)
        return np.squeeze(np.array(self._numeric_fk(*joints)))

    def inverse_kinematics(self, target_pose, n_restarts=25, error_threshold=1e-5, dedup_tol=0.01):
        """
        Explores the joint space to find multiple valid solutions.
        Returns a list of all unique solutions found.
        """

        def objective_function(q):
            # Only compare the DOF provided in the target (e.g., first 3 for fix position)
            target = np.array(target_pose)
            current_pose = self.forward_kinematics(q)[:len(target)]
            
            n_pos = 3  # first components are Cartesian position (x, y, z)
            error_pos = current_pose[:n_pos] - target[:n_pos]
            error_rot = current_pose[n_pos:] - target[n_pos:]
            
            # Wrap angular error to (−π, π) to prevent discontinuities at ±π do not
            error_rot = (error_rot + np.pi) % (2 * np.pi) - np.pi
            
            return np.sum(error_pos**2) + np.sum(error_rot**2)
        
        # Generate starting guesses
        rng = np.random.default_rng()
        guesses = [np.zeros(len(self.joint_vars))]
        for _ in range(n_restarts):
            guesses.append([rng.uniform(b[0], b[1]) for b in self.joint_bounds])

        # Run the optimizer for every guess
        solutions = []

        for guess in guesses:
            res = minimize(
                objective_function,
                guess,
                method='SLSQP',
                bounds=self.joint_bounds,
                tol=1e-8,
                options={'maxiter': 500}  
            )

            # Evaluate if new solution was found
            if res.fun < error_threshold:
                candidate = np.array(res.x)

                # Deduplicate: skip if this configuration is close to one already collected
                is_duplicate = any(np.allclose(candidate, sol, atol=dedup_tol) for sol in solutions)
                if not is_duplicate:
                    solutions.append(candidate)

        return [sol.tolist() for sol in solutions]


    def jacobian(self, q1, q2, q3, q4, q5):
        """Returns the 5x5 Jacobian matrix at the given joint configuration."""
        self._check_joint_limits([q1, q2, q3, q4, q5])
        return self._numeric_jac(q1, q2, q3, q4, q5)
      
    def jacobian_inverse(self, current_joint_angles, 
                            singularity_threshold=0.1, lambda_max=0.1):
        """
        Computes joint angle velocities from a task-space desired velocity using the
        Damped Least Squares (DLS) pseudo-inverse of the Jacobian (and prevent singularities).

        For that , the DLS replaces 1/σ_i with σ_i/(σ_i² + λ²), which keeps
        the solution bounded at the cost of a small positional error 

        Based on Nakamura–Hanafusa variable damping rule.
        """
        # Compute Jacobian at current configuration — unpack array into 5 positional args
        J = self._numeric_jac(*current_joint_angles)

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

        # Compute target joint velocities.
        # J is (6x5): U is (6x6), S is (5,), Vt is (5x5).
        # Pseudo-inverse: V @ diag(1/σ) @ Ur.T  where Ur = U[:, :5] — the range subspace.
        Ur = U[:, :len(singular_values)]
        inverse_jacobian = Vt.T @ np.diag(damped_inv_sigmas) @ Ur.T

        return inverse_jacobian, is_singular

