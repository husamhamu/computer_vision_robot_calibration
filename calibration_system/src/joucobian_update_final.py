#!/usr/bin/env python3
import numpy as np
from scipy.linalg import logm
import modern_robotics as mr
import matplotlib.pyplot as plt

def skew(w):
    """Convert vector to skew symmetric matrix."""
    return np.array([[0, -w[2], w[1]],
                    [w[2], 0, -w[0]],
                    [-w[1], w[0], 0]])

def twist_to_se3(xi):
    """Convert twist coordinates to SE(3) element."""
    w = xi[:3]
    v = xi[3:]
    return np.block([
        [skew(w), v.reshape(-1,1)],
        [np.zeros((1, 4))]
    ])

def adjoint_transform(T):
    """Compute adjoint transformation."""
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return np.vstack([
        np.hstack([R, np.zeros((3, 3))]),
        np.hstack([skew(p) @ R, R])
    ])

def compute_A_matrix(xi, theta):
    """Compute A matrix for error model."""
    w = xi[:3]
    v = xi[3:]
    w_norm = np.linalg.norm(w)
    
    if w_norm < 1e-10:
        return np.eye(6)
    
    # Create the Omega matrix in se(3)
    Omega = np.zeros((6, 6))
    w_hat = skew(w)
    v_hat = skew(v)
    Omega[:3, :3] = w_hat
    Omega[3:, :3] = v_hat
    Omega[3:, 3:] = w_hat
    
    theta_normalized = w_norm * theta
    
    # Calculate terms
    term1 = theta_normalized * np.eye(6)
    term2 = ((4 - theta_normalized * np.sin(theta_normalized) - 4 * np.cos(theta_normalized)) / 
             (2 * w_norm**2)) * Omega
    term3 = ((4*theta_normalized - 5*np.sin(theta_normalized) + theta_normalized*np.cos(theta_normalized)) / 
             (2 * w_norm**3)) * (Omega @ Omega)
    term4 = ((2 - theta_normalized*np.sin(theta_normalized) - 2*np.cos(theta_normalized)) / 
             (2 * w_norm**4)) * (Omega @ Omega @ Omega)
    term5 = ((2*theta_normalized - 3*np.sin(theta_normalized) + theta_normalized*np.cos(theta_normalized)) / 
             (2 * w_norm**5)) * (Omega @ Omega @ Omega @ Omega)
    
    return term1 + term2 + term3 + term4 + term5

def forward_kinematics(S, theta, M):
    """Compute forward kinematics using POE formula."""
    T = np.eye(4)
    for i in range(len(theta)):
        T = T @ mr.MatrixExp6(mr.VecTose3(S[:,i] * theta[i]))
    return T @ M

def compute_pose_error(T_measured, T_predicted):
    """Compute pose error as described in equation 10."""
    return mr.se3ToVec(logm(T_measured @ np.linalg.inv(T_predicted)))

def compute_identification_jacobian(S, theta, M):
    """Compute identification Jacobian as per equation 27."""
    n = len(theta)  # number of joints
    J = np.zeros((6, 6*n))
    
    T = np.eye(4)
    for i in range(n):
        # Compute A matrix for current joint
        A_i = compute_A_matrix(S[:,i], theta[i])
        
        # Compute adjoint transformation up to current joint
        if i == 0:
            J[:, 6*i:6*(i+1)] = A_i
        else:
            Ad_T = adjoint_transform(T)
            J[:, 6*i:6*(i+1)] = Ad_T @ A_i
            
        # Update transformation
        T = T @ mr.MatrixExp6(mr.VecTose3(S[:,i] * theta[i]))
    
    return J

def calibrate_robot(joint_states, measured_poses, S_nominal, M_home, max_iterations=2, tol=1e-6):
    """
    Perform kinematic calibration using iterative least squares.
    
    Args:
        joint_states: List of joint configurations
        measured_poses: List of measured TCP poses
        S_nominal: Nominal screw axes
        M_home: Home configuration
        max_iterations: Maximum number of iterations
        tol: Convergence tolerance
    
    Returns:
        S_calibrated: Calibrated screw axes
    """
    n_poses = len(joint_states)
    n_joints = S_nominal.shape[1]
    
    # Check if we have enough measurements (equation 29)
    if n_poses < n_joints + 1:
        raise ValueError(f"Need at least {n_joints + 1} poses for calibration")
    
    S_current = S_nominal.copy()
    
    for iteration in range(max_iterations):
        # Initialize matrices for equation 30
        Y = np.zeros((6 * n_poses, 1))
        J = np.zeros((6 * n_poses, 6 * n_joints))
        
        for i in range(n_poses):
            # Compute predicted pose
            T_predicted = forward_kinematics(S_current, joint_states[i], M_home)
            
            # Compute pose error (equation 26)
            y_i = compute_pose_error(measured_poses[i], T_predicted)
            Y[6*i:6*(i+1)] = y_i.reshape(-1,1)
            
            # Compute identification Jacobian (equation 27)
            J_i = compute_identification_jacobian(S_current, joint_states[i], M_home)
            J[6*i:6*(i+1)] = J_i
        
        # Solve for parameter updates using equation 32
        try:
            dx = np.linalg.pinv(J.T @ J) @ J.T @ Y
        except np.linalg.LinAlgError:
            print("Warning: Singular matrix encountered")
            break
            
        # Update screw axes
        for j in range(n_joints):
            S_current[:,j] += dx[6*j:6*(j+1)].flatten()

        # Check convergence
        if np.linalg.norm(dx) < tol:
            break
            
    return S_current

def perform_incremental_calibration(joint_states, measured_poses, S_nominal, M_home, pose_sets):
    """
    Perform calibration with incremental number of poses.
    
    Args:
        joint_states: List of all joint configurations
        measured_poses: List of all measured poses
        S_nominal: Nominal screw axes
        M_home: Home configuration
        pose_sets: List of numbers of poses to use
    
    Returns:
        List of calibrated screw axes for each set
    """
    results = []
    S_calibrated = S_nominal.copy()
    
    for n_poses in pose_sets:
        # Take subset of poses
        joints_subset = joint_states[:n_poses]
        poses_subset = measured_poses[:n_poses]
        
        # Perform calibration
        S_calibrated = calibrate_robot(joints_subset, poses_subset, S_calibrated, M_home)
        results.append(S_calibrated)
    print(f'S_calibrated: {S_calibrated.T}') 
    return results

import yaml
def load_calibration_data(filename):
    """Load joint states and measured poses from YAML file"""
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    
    joint_data = []
    measured_poses = []
    
    for entry in data:
        joint_data.append(np.array(entry['joints'], dtype=np.float64))
        pose = np.array(entry['tag_pose'], dtype=np.float64).reshape(4, 4)
        # pose[:3, 3] = pose[:3, 3] + 0.008
        measured_poses.append(pose)
    
    return np.array(joint_data, dtype=np.float64), np.array(measured_poses, dtype=np.float64)

# Load data
joint_data, measured_poses = load_calibration_data('measurements_calibration.yaml')

# Example usage:
pose_sets = [60]


# Nominal screw axes in space frame
S_nominal = np.array([
    [0, 0, 1, 0, 0, 0],          # Joint 1
    [0, 1, 0, -0.193, 0, 0],      # Joint 2
    [0, 1, 0, -0.383, 0, 0],      # Joint 3
    [1, 0, 0, 0, 0.443, 0],      # Joint 4
    [0, 1, 0, -0.443, 0, 0.250],      # Joint 5 # 245, 160
    [0, 0, 1, 0, -0.395, 0]       # Joint 6
]).T
# S_calibrated: [
#     [-0.0112297078, -0.0160867045, 0.966771589, 0.00207839644, 0.00647317599, -0.0022480086],
#     [0.0337968215, 0.974157641, 0.0121623961, -0.182563296, 0.0119836019, -0.0125544729],
#     [0.0313878786, 0.987944967, 0.0355253674, -0.379837185, 0.0140029884, -0.00727486733],
#     [1.33221003, 0.0146880591, 0.0326310657, 0.00166308417, 0.593454302, 0.00700175739],
#     [1.68824037, 1.85103143, 0.649686073, -0.747558701, 0.400877813, 0.62683995],
#     [0.0787717108, 0.0285274522, 0.742823868, 0.000273423575, -0.253344385, 0.0141408016]
# ]

# Home configuration
M_home = np.eye(4)
M_home[0:3, 3] = [0.395, 0, 0.443]

# M_home = np.array([[-0.19850023, -0.244237,    0.94918172,  0.4200044 ],
#  [ 0.97721246,  0.02497638,  0.21078899,  0.03129141],
#  [-0.07518959,  0.96939387,  0.23371362,  0.4919935 ],
#  [ 0.0,          0.0,          0.0,          1.0        ]])
# M_home = np.array([
#     [-0.05440330643555502, -0.007427830764184407,    0.9984914158764802,  0.41289909225669874 ],
#     [ 0.9937388840161836,  -0.09813224476733928,  0.05341435135984707,  0.009527955727488324],
#     [0.09758745125859819,  0.9951456626379185,  0.012720042837548018,  0.4423253184999899 ],
#     [ 0.0,          0.0,          0.0,          1.0        ]])

# Assuming joint_states and measured_poses are loaded from your YAML file
results = perform_incremental_calibration(joint_data, measured_poses, 
                                        S_nominal, M_home, pose_sets)

import numpy as np
from scipy import stats

def compute_position_errors(T_measured, T_predicted):
    """
    Compute position errors between measured and predicted poses.
    
    Args:
        T_measured: 4x4 measured transformation matrix
        T_predicted: 4x4 predicted transformation matrix
    
    Returns:
        errors: Dictionary containing errors in x, y, z
    """
    # Extract positions
    p_measured = T_measured[0:3, 3]
    p_predicted = T_predicted[0:3, 3]
    
    # Compute errors
    error_xyz = p_measured - p_predicted
    
    return {
        'x': error_xyz[0],
        'y': error_xyz[1],
        'z': error_xyz[2],
        'absolute': np.linalg.norm(error_xyz)
    }

def validate_position_accuracy(joint_states, measured_poses, S_calibrated, M_home, verbose=True):
    """
    Validate position accuracy of the calibrated robot.
    
    Args:
        joint_states: List of joint configurations
        measured_poses: List of measured TCP poses
        S_calibrated: Calibrated screw axes
        M_home: Home configuration
        verbose: Whether to print detailed statistics
    
    Returns:
        Dictionary containing validation statistics
    """
    errors_x = []
    errors_y = []
    errors_z = []
    absolute_errors = []
    
    # Compute errors for all poses
    for joints, T_measured in zip(joint_states, measured_poses):
        # Forward kinematics with calibrated parameters
        T_predicted = forward_kinematics(S_calibrated, joints, M_home)
        
        # Compute position errors
        errors = compute_position_errors(T_measured, T_predicted)
        
        errors_x.append(errors['x'])
        errors_y.append(errors['y'])
        errors_z.append(errors['z'])
        absolute_errors.append(errors['absolute'])
    
    # Convert to numpy arrays
    errors_x = np.array(errors_x)
    errors_y = np.array(errors_y)
    errors_z = np.array(errors_z)
    absolute_errors = np.array(absolute_errors)
    
    # Compute statistics
    stats_dict = {
        'x': {
            'mean': np.mean(errors_x),
            'std': np.std(errors_x),
            'rms': np.sqrt(np.mean(np.square(errors_x))),
            'max': np.max(np.abs(errors_x))
        },
        'y': {
            'mean': np.mean(errors_y),
            'std': np.std(errors_y),
            'rms': np.sqrt(np.mean(np.square(errors_y))),
            'max': np.max(np.abs(errors_y))
        },
        'z': {
            'mean': np.mean(errors_z),
            'std': np.std(errors_z),
            'rms': np.sqrt(np.mean(np.square(errors_z))),
            'max': np.max(np.abs(errors_z))
        },
        'absolute': {
            'mean': np.mean(absolute_errors),
            'std': np.std(absolute_errors),
            'rms': np.sqrt(np.mean(np.square(absolute_errors))),
            'max': np.max(absolute_errors)
        }
    }
    
    if verbose:
        print("\nPosition Accuracy Validation Results:")
        print("=====================================")
        for axis in ['x', 'y', 'z', 'absolute']:
            print(f"\n{axis.upper()} Direction:")
            print(f"Mean Error: {stats_dict[axis]['mean']:.6f} mm")
            print(f"Std Dev: {stats_dict[axis]['std']:.6f} mm")
            print(f"RMS Error: {stats_dict[axis]['rms']:.6f} mm")
            print(f"Max Error: {stats_dict[axis]['max']:.6f} mm")
    
    return stats_dict

def plot_position_errors(joint_states, measured_poses, S_calibrated, M_home):
    """
    Plot position errors for visual analysis.
    
    Args:
        joint_states: List of joint configurations
        measured_poses: List of measured TCP poses
        S_calibrated: Calibrated screw axes
        M_home: Home configuration
    """
    # import matplotlib.pyplot as plt
    
    # Compute errors for all poses
    errors_x = []
    errors_y = []
    errors_z = []
    absolute_errors = []
    
    for joints, T_measured in zip(joint_states, measured_poses):
        T_predicted = forward_kinematics(S_calibrated, joints, M_home)
        errors = compute_position_errors(T_measured, T_predicted)
        
        errors_x.append(errors['x'])
        errors_y.append(errors['y'])
        errors_z.append(errors['z'])
        absolute_errors.append(errors['absolute'])
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('Position Errors Analysis')
    
    # Plot individual axis errors
    pose_indices = range(len(joint_states))

    axes[0, 0].plot(pose_indices, errors_x, 'b.-')
    axes[0, 0].set_title('X-axis Error')
    axes[0, 0].set_ylabel('Error (mm)')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(pose_indices, errors_y, 'g.-')
    axes[0, 1].set_title('Y-axis Error')
    axes[0, 1].set_ylabel('Error (mm)')
    axes[0, 1].grid(True)
    
    axes[1, 0].plot(pose_indices, errors_z, 'r.-')
    axes[1, 0].set_title('Z-axis Error')
    axes[1, 0].set_xlabel('Iteration')
    axes[1, 0].set_ylabel('Error (mm)')
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(pose_indices, absolute_errors, 'k.-')
    axes[1, 1].set_title('Absolute Error')
    axes[1, 1].set_xlabel('Iteration')
    axes[1, 1].set_ylabel('Error (mm)')
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    plt.show()

# Example usage for validation after calibration:
def validate_calibration_results(results, joint_states, measured_poses, M_home, pose_sets):
    """
    Validate calibration results for different pose sets.
    
    Args:
        results: List of calibrated screw axes for each pose set
        joint_states: List of all joint configurations
        measured_poses: List of all measured poses
        M_home: Home configuration
        pose_sets: List of numbers of poses used
    """
    validation_stats = []
    print(len(pose_sets))
    for S_calibrated, n_poses in zip(results, pose_sets):
        print(f"\nValidating calibration with {n_poses} poses:")
        
        # Use remaining poses as validation set
        validation_joints = joint_states[:]
        validation_poses = measured_poses[:]
        print(len(validation_joints))
        if len(validation_joints) > 0:
            stats = validate_position_accuracy(
                validation_joints, 
                validation_poses, 
                S_calibrated, 
                M_home
            )
            validation_stats.append({
                'n_poses': n_poses,
                'stats': stats
            })
            
            # Plot errors for visual analysis
            plot_position_errors(validation_joints, validation_poses, S_calibrated, M_home)
    
    return validation_stats

# Load data
joint_data_valid, measured_poses_valid = load_calibration_data('measurements_validation.yaml')
pose_sets_valid = [12]
print(len(measured_poses_valid))
# After performing calibration with different pose sets
validation_stats = validate_calibration_results(
    results,              # Results from perform_incremental_calibration
    joint_data_valid,         # Your joint states
    measured_poses_valid,       # Your measured poses
    M_home,              # Home configuration
    pose_sets_valid            # List of pose set sizes
)

# The validation will print statistics and show plots for each pose set