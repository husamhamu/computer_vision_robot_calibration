#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.linalg import logm, expm
import modern_robotics as mr
from geometry_msgs.msg import Transform, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformListener, Buffer
import tf.transformations as tf_trans
import yaml
import joint_position_controller
import cv2

class Mover6CalibrationSystem:
    def __init__(self, offline= True):
        rospy.init_node('mover6_calibration_system')

        # Create joint position publisher instance
        self.offline = offline
        if not self.offline:
            self.joint_controller = joint_position_controller.JointPositionController()
            
            # Initialize TF listener
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)
            
            # Initialize subscribers
            self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
            
            # Initialize publishers
            self.cmd_pub = rospy.Publisher('/trajectory_controller/command', 
                                        Float64MultiArray, queue_size=10)
            
        # Initialize calibration parameters
        self.initialize_parameters()
        
        # Calibration data storage
        self.measurements = []
        self.current_joints = None
        
        # Configuration
        self.num_poses = 4
        self.convergence_threshold = 1e-12
        self.max_iterations = 100
        
    def initialize_parameters(self):
        """Initialize nominal kinematic parameters"""
        # Nominal screw axes in space frame
        self.S_nominal = np.array([
            [0, 0, 1, 0, 0, 0],          # Joint 1
            [0, 1, 0, -0.193, 0, 0],      # Joint 2
            [0, 1, 0, -0.383, 0, 0],      # Joint 3
            [1, 0, 0, 0, 0.443, 0],      # Joint 4
            [0, 1, 0, -0.443, 0, 0.250],      # Joint 5 # 245, 160
            [0, 0, 1, 0, -0.395, 0]       # Joint 6
        ]).T
        
        # Home configuration
        self.M = np.eye(4)
        self.M[0:3, 3] = [0.395, 0, 0.443]
        
        # AprilTag to end-effector transform (to be measured)
        self.T_tag_ee = np.eye(4)
        
        # Define camera to base transformer
        self.T_base_camera = np.eye(4)
        # self.T_base_camera = T_base_tag @ np.linalg.inv(T_camera_tag)
        
    def joint_callback(self, msg):
        """Store current joint positions"""
        self.current_joints = np.array(msg.position)
        if self.current_joints.size > 0 :
            self.current_joints[0] = - self.current_joints[0]
        
    def get_apriltag_pose(self):
        """Get AprilTag pose from TF"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_pose', 'TCP_pose', rospy.Time(0)
            )
            rospy.loginfo("Mesaurment read successfuly")
            return self.transform_to_matrix(trans)
        except Exception as e:
            rospy.logerr(f"Failed to get AprilTag transform: {e}")
            return None

    def get_measurment(self):
        """Get AprilTag pose from TF"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_pose', 'TCP_pose', rospy.Time(0)
            )
            return self.transform_to_matrix(trans)
        except Exception as e:
            rospy.logerr(f"Failed to get AprilTag transform: {e}")
            return None
            
    def transform_to_matrix(self, trans):
        """Convert ROS transform to homogeneous matrix"""
        T = np.eye(4)
        T[0:3, 3] = [
            trans.transform.translation.x,
            trans.transform.translation.y,
            trans.transform.translation.z
        ]
        q = [
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        ]
        T[0:3, 0:3] = tf_trans.quaternion_matrix(q)[:3, :3]
        return T
        
    def collect_calibration_data(self):
        """Collect calibration data by moving robot to different poses"""
        rospy.loginfo("Starting calibration data collection...")
        
        # Generate calibration poses
        calibration_poses = self.generate_calibration_poses()
        
        for pose in calibration_poses:
            # Move robot to pose
            self.move_robot(pose)
            rospy.sleep(3.0)  # Wait for robot to settle

            # # Wait for user to press the 'm' key
            # while True:
            #     key = input("Press 'm' to confirm: ").strip().lower()
            #     if key == 'm':
            #         rospy.loginfo("Key 'm' pressed. Continuing...")
            #         break
            #     else:
            #         rospy.loginfo("Invalid input. Please press 'm'.")

            # Get measurements
            if self.current_joints is None:
                rospy.logwarn("No joint state received")
                continue
            rospy.loginfo(f"Joint states read successfully")
            tag_pose = self.get_apriltag_pose()
            if tag_pose is None:
                continue
                
            # Store measurement
            self.measurements.append({
                'joints': self.current_joints.copy(),
                'tag_pose': tag_pose
            #     'tag_pose': self.forward_kinematics(
            #     self.current_joints.copy(), 
            #     np.zeros(42)
            # )
            })
            
        rospy.loginfo(f"Collected {len(self.measurements)} measurements")


    def save_multiple_measurements_yaml(self, filename="joints_and_tag_pose.yaml"):
        """Save a list of matrices to a YAML file."""
        serializable_data = [
            {
                "joints": pair["joints"].tolist(),
                "tag_pose": pair["tag_pose"].tolist(),
            }
            for pair in self.measurements
        ]
        with open(filename, 'w') as f:
            yaml.dump(serializable_data, f)

    def load_transform_measurements_yaml(self, filename="measurements_calibration.yaml"):
        """Load a list of matrices from a YAML file."""
        with open(filename, 'r') as f:
            serializable_data = yaml.safe_load(f)
        return [
            {
                "joints": np.array(pair["joints"]),
                "tag_pose": np.array(pair["tag_pose"]),
            #     'tag_pose': self.forward_kinematics(
            #     pair["joints"], 
            #     np.zeros(42)
            # )
                # "tag_pose": mr.FKinSpace(self.M, self.S_nominal, pair["joints"])
            }
            for pair in serializable_data
        ]

    def save_transform_matrices_yaml(self, data, filename="transform_matrices.yaml"):
        """Save a list of matrices to a YAML file."""
        serializable_data = [
            {
                "T_measured": pair["T_measured"].tolist(),
                "T_nominal": pair["T_nominal"].tolist(),
            }
            for pair in data
        ]
        with open(filename, 'w') as f:
            yaml.dump(serializable_data, f)

    def load_transform_matrices_yaml(self, filename="transform_matrices.yaml"):
        """Load a list of matrices from a YAML file."""
        with open(filename, 'r') as f:
            serializable_data = yaml.safe_load(f)
        return [
            {
                "T_measured": np.array(pair["T_measured"]),
                "T_nominal": np.array(pair["T_nominal"]),
            }
            for pair in serializable_data
        ]
    def transform_matrices(self):
        """Calculate residual for optimization"""
        matrices = []
        
        for measurement in self.measurements:
            # Forward kinematics with current parameters
            T_nominal = self.forward_kinematics(
                measurement['joints'], 
                np.zeros(42)
            )
            # T_nominal =  mr.FKinSpace(self.M, self.S_nominal, measurement['joints'])
            
            # Measurement chain
            T_measured = measurement['tag_pose']
            # T_measured = T_nominal.copy()
            # T_measured[:3, 3] = T_measured[:3, 3] + 0.08
            # Store data
            matrices.append({
                'T_nominal': T_nominal,
                'T_measured': T_measured
            })          
        return matrices

    def T_measured(self, tag_pose):
        # T_measured = self.T_base_camera @ tag_pose @ np.linalg.inv(self.T_tag_ee)
        T_measured = self.get_measurment()
        return T_measured
    
    def generate_calibration_poses(self):
        """Generate poses for calibration"""
        # Simple strategy: vary each joint within its limits
        poses = []
        joint_ranges = [  # [min, max] for each joint
            [-np.pi, np.pi],      # Joint 1
            [-np.pi/2, np.pi/2],  # Joint 2
            [-np.pi/2, np.pi/2],  # Joint 3
            [-np.pi, np.pi],      # Joint 4
            [-np.pi/2, np.pi/2],  # Joint 5
            [-np.pi, np.pi]       # Joint 6
        ]
        
        
        for _ in range(self.num_poses):
            pose = [np.random.uniform(min_val, max_val) 
                   for min_val, max_val in joint_ranges]
            poses.append(pose)

        # poses = [[0.1, 0.3, 0.3, 0.1], [0.1, 0.4, 0.1, 0.0], [0.4, 0.1, 0.4, 0.0], [0.2, 0.2, 0.2, 0.0], [0.5, 0.2, 0.2, 0.0], 
        #          [0.1, 0.4, 0.3, 0.1], [0.4, 0.4, 0.1, 0.0], [0.2, 0.1, 0.4, 0.0], [0.3, 0.4, 0.2, 0.0], [0.2, 0.4, 0.0, 0.0], 
        #          [0.2, 0.3, 0.1, 0.1], [0.5, 0.3, 0.1, 0.0], [0.2, 0.2, 0.4, 0.0], [0.2, 0.3, 0.2, 0.0], [0.3, 0.3, 0.4, 0.0], 
        #          [0.3, 0.4, 0.1, 0.1], [0.4, 0.1, 0.3, 0.0], [0.1, 0.1, 0.3, 0.0], [0.2, 0.1, 0.3, 0.0], [0.4, 0.4, 0.2, 0.0], 
        #          [0.4, 0.2, 0.2, 0.1], [0.2, 0.3, 0.2, 0.0], [0.5, 0.2, 0.3, 0.0], [0.3, 0.3, 0.1, 0.0], [0.3, 0.4, 0.0, 0.0], 
        #          [0.1, 0.2, 0.3, 0.1], [0.4, 0.1, 0.2, 0.0], [0.1, 0.4, 0.1, 0.0], [0.3, 0.2, 0.2, 0.1], [0.4, 0.2, 0.3, 0.0]
        #          ]
        # poses = [[0.6, 0.3, 0.3, 0.1], [0.4, 0.4, 0.2, 0.0], [0.5, 0.1, 0.4, 0.0], [0.2, 0.2, 0.4, 0.0], [0.1, 0.2, 0.3, 0.0], 
        #          [0.6, 0.4, 0.3, 0.1], [0.4, 0.4, 0.2, 0.0], [0.5, 0.1, 0.4, 0.0], [0.3, 0.4, 0.4, 0.0], [0.2, 0.2, 0.3, 0.0], 
        #          [0.6, 0.3, 0.1, 0.1], [0.4, 0.4, 0.1, 0.0], [0.5, 0.2, 0.4, 0.0], [0.2, 0.3, 0.4, 0.0], [0.3, 0.3, 0.3, 0.0], 
        #          [0.6, 0.4, 0.1, 0.1], [0.4, 0.2, 0.3, 0.0], [0.5, 0.1, 0.3, 0.0], [0.2, 0.1, 0.4, 0.0], [0.4, 0.2, 0.3, 0.0], 
        #          [0.6, 0.2, 0.2, 0.1], [0.4, 0.3, 0.2, 0.0], [0.5, 0.2, 0.3, 0.0], [0.3, 0.3, 0.4, 0.0], [0.3, 0.1, 0.3, 0.0], 
        #          [0.6, 0.2, 0.3, 0.1], [0.4, 0.1, 0.2, 0.0], [0.5, 0.4, 0.1, 0.0], [0.3, 0.2, 0.4, 0.1], [0.4, 0.1, 0.3, 0.0]
        #          ]
        # poses = [[0.7, 0.3, 0.35, 0.1], [0.45, 0.4, 0.25, 0.0], [0.55, 0.15, 0.4, 0.0], [0.25, 0.25, 0.4, 0.0], [0.1, 0.25, 0.35, 0.0], 
        #          [0.7, 0.4, 0.35, 0.1], [0.45, 0.45, 0.2, 0.0], [0.55, 0.1, 0.45, 0.0], [0.35, 0.45, 0.4, 0.0], [0.2, 0.25, 0.35, 0.0], 
        #          [0.7, 0.3, 0.15, 0.1], [0.45, 0.45, 0.1, 0.0], [0.55, 0.2, 0.45, 0.0], [0.25, 0.35, 0.4, 0.0], [0.3, 0.35, 0.35, 0.0], 
        #          [0.7, 0.4, 0.15, 0.1], [0.45, 0.25, 0.3, 0.0], [0.55, 0.1, 0.35, 0.0], [0.25, 0.15, 0.4, 0.0], [0.4, 0.25, 0.35, 0.0], 
        #          [0.7, 0.2, 0.25, 0.1], [0.45, 0.3, 0.2, 0.0], [0.55, 0.2, 0.35, 0.0], [0.35, 0.35, 0.4, 0.0], [0.3, 0.15, 0.35, 0.0], 
        #          [0.7, 0.2, 0.35, 0.1], [0.45, 0.15, 0.2, 0.0], [0.55, 0.4, 0.15, 0.0], [0.35, 0.25, 0.4, 0.1], [0.4, 0.15, 0.35, 0.0]
        #          ]
        poses = [[0.0, 0.0, 0.0, 0.0]]


        return poses
        
    def calibrate(self):
        """Main calibration routine"""
        rospy.loginfo("Starting calibration process...")
        
        if self.offline:
            self.measurements = self.load_transform_measurements_yaml()
            rospy.loginfo(f"Loadded {len(self.measurements)} collected measurements!")

        # Collect data if needed
        if not self.measurements:
            self.collect_calibration_data()

        # Save transform matrices
        if self.measurements:
            self.save_transform_matrices_yaml(self.transform_matrices())
            rospy.loginfo("Saved transform matrices!")
            self.save_multiple_measurements_yaml()
            rospy.loginfo("Saved joint states and measurment pose!")
        print()
        # Setup optimization
        from scipy.optimize import least_squares
        
        # Initial guess for parameter corrections
        p0 = np.zeros(6)  # 36 screw params + 6 home config params
        # Convert M_home to initial parameters
        t_initial = self.M[0:3, 3]
        R_initial = self.M[0:3, 0:3]
        rotation_vector_initial, _ = cv2.Rodrigues(R_initial)
        rotation_vector_initial = rotation_vector_initial.flatten()
        params_initial = np.concatenate([self.S_nominal.flatten(), t_initial, rotation_vector_initial])
        # Run optimization
        result = least_squares(
            self.calibration_residual,
            p0,
            method='lm',
            ftol=self.convergence_threshold,
            # max_nfev=self.max_iterations,
            verbose=2
        )
        print()
        if result.success:
            rospy.loginfo("Calibration successful!")
            self.update_parameters(result.x)
            rospy.loginfo(f"Updated parameters")
            rospy.loginfo(f"Home:{self.M}:")
            rospy.loginfo(f"S_nominal:{self.S_nominal.T}:")
        else:
            rospy.logwarn("Calibration did not converge!")
            rospy.loginfo(f"Home:{self.M}:")
            rospy.loginfo(f"S_nominal:{self.S_nominal.T}:")
            
        return result.success, result.cost
        
    def calibration_residual(self, params):
        """Calculate residual for optimization"""
        residuals = []
        
        for measurement in self.measurements:
            # Forward kinematics with current parameters
            T_nominal = self.forward_kinematics(
                measurement['joints'], 
                params
            )
            
            # Measurement chain
            T_measured = measurement['tag_pose']
            # T_measured = T_nominal.copy()
            # T_measured[:3, 3] = T_measured[:3, 3]  + 0.08
            # T_measured[:, :] = T_measured[:, :]  + 0.008
            # Compute error
            # T_measured = np.round(T_nominal / 0.008) * 0.008
            error = self.pose_error(T_measured, T_nominal)
            residuals.extend(error)
            
        return np.array(residuals)

    def forward_kinematics(self, theta, params):
        """POE forward kinematics with current parameters"""
        S = self.S_nominal.copy()
        M = self.M.copy()

        # Apply parameters
        # for i in range(6):
        #     S[:, i] += params[i*6:(i+1)*6]
        # print(f"params:{params}")
        M[0:3, 3] += params[:3]
        M[0:3, 0:3] = M[0:3, 0:3] @ expm(mr.VecToso3(params[3:]))
        # print(f'M:{M}')
        T = np.eye(4)  # Start with identity matrix
        for i in range(6):
            # Get the screw axis for joint i
            screw_axis = S[:, i]
            angle = theta[i]
            # Compute the twist vector (omega, v) scaled by the angle
            twist = screw_axis * angle
            # Convert twist to se(3) matrix
            se3_mat = mr.VecTose3(twist)
            # Compute matrix exponential of the twist
            exp_mat = expm(se3_mat)
            # Update the transformation matrix
            T = T @ exp_mat
        # Multiply by the home configuration matrix
        T = T @ M
        return T
        
    def pose_error(self, T1, T2):
        """Compute pose error between two transforms"""
        error = logm(np.linalg.inv(T1) @ T2)
        return np.array([
            error[0:3, 3],  # translation error
            mr.so3ToVec(error[0:3, 0:3])  # rotation error
        ]).flatten()
        
    def update_parameters(self, params):
        """Update kinematic parameters with optimized values"""
        # Update screw axes
        # for i in range(6):
        #     self.S_nominal[:, i] += params[i*6:(i+1)*6]
            
        # Update home configuration
        self.M[0:3, 3] += params[:3]
        self.M[0:3, 0:3] = self.M[0:3, 0:3] @ expm(mr.VecToso3(params[3:]))
        
    def move_robot(self, joint_positions):
        """Send joint positions to robot controller"""
        # msg = Float64MultiArray()
        # msg.data = joint_positions
        # self.cmd_pub.publish(msg)
        success = self.joint_controller.move_to_position(joint_positions)
        if success:
            rospy.loginfo("Movement completed successfully")
        else:
            rospy.logerr("Movement failed")
            # return
        
    def validate_calibration(self, num_poses=10):
        """Validate calibration results"""
        print()
        rospy.loginfo("Starting calibration validation...")
        
        errors = []
        validation_poses = self.generate_calibration_poses()[:num_poses]
        if self.offline:
            for measurement in self.measurements:
                # Forward kinematics with current parameters
                T_nominal = self.forward_kinematics(
                    measurement['joints'], 
                    np.zeros(42)
                )
                
                # Measurement chain
                T_measured = measurement['tag_pose']

                error = self.pose_error(T_measured, T_nominal)
                errors.append(np.linalg.norm(error))
        else:
            for pose in validation_poses:
                self.move_robot(pose)
                rospy.sleep(2.0)
                
                if self.current_joints is None:
                    continue
                    
                tag_pose = self.get_apriltag_pose()
                if tag_pose is None:
                    continue
                    
                T_measured = self.T_measured(tag_pose)
                T_nominal = self.forward_kinematics(self.current_joints, np.zeros(42))

                error = self.pose_error(T_measured, T_nominal)
                errors.append(np.linalg.norm(error))
            
        if errors:
            mean_error = np.mean(errors)
            std_error = np.std(errors)
            print(  )
            rospy.loginfo(f"Validation Results:")
            rospy.loginfo(f"Mean Error: {mean_error:.6f}")
            rospy.loginfo(f"Std Error: {std_error:.6f}")
            
        return mean_error, std_error

if __name__ == '__main__':
    try:
        calibrator = Mover6CalibrationSystem(offline=True)
        success, cost = calibrator.calibrate()
        if True:
            mean_error, std_error = calibrator.validate_calibration()
    except rospy.ROSInterruptException:
        pass