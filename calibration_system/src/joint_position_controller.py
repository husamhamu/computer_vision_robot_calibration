#!/usr/bin/env python3

import rospy
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
import numpy as np

class JointPositionController:
    def __init__(self):
        # Publisher for joint positions
        self.joint_pub = rospy.Publisher('/JointJog', JointJog, queue_size=10)
        
        # Subscriber for current joint states
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Current joint positions
        self.current_positions = None
        
        # Define controlled and all joints
        self.controlled_joints = ['joint1', 'joint2', 'joint3', 'joint4']
        self.all_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Movement parameters
        self.position_tolerance = 0.018  # radians
        self.max_velocity = 2.0       # radians/sec
        self.update_rate = 10          # Hz
        
        # Wait for first joint states message
        rospy.loginfo("Waiting for joint states...")
        while self.current_positions is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Received initial joint states!")

    def joint_states_callback(self, msg):
        """Callback for joint states subscriber"""
        self.current_positions = msg.position

    def is_at_target(self, target_positions):
        """Check if controlled joints are at target position within tolerance"""
        for i in range(len(self.controlled_joints)):
            if abs(self.current_positions[i] - target_positions[i]) > self.position_tolerance:
                return False
        return True

    def get_bounded_velocity(self, displacement):
        """Limit velocity based on displacement"""
        return np.clip(displacement, -self.max_velocity, self.max_velocity)

    def send_stop_command(self):
        """Send a command to stop all joint movement"""
        msg = JointJog()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = self.all_joints
        msg.velocities = [0.0] * 6
        msg.displacements = [0.0] * 6
        msg.duration = 0.1
        
        for _ in range(3):
            self.joint_pub.publish(msg)
            rospy.sleep(0.05)

    def move_to_position(self, target_positions):
        """
        Move controlled joints to specified positions.
        
        Args:
            target_positions (list): List of 4 target joint positions in radians for joints 1-4
            
        Returns:
            bool: True if target position was reached, False otherwise
        """
        if len(target_positions) != 4:
            rospy.logerr("Expected 4 joint positions, got %d", len(target_positions))
            return False
            
        if self.current_positions is None:
            rospy.logerr("No joint states received yet!")
            return False

        rospy.loginfo("Moving controlled joints to position: %s", [round(x, 3) for x in target_positions])
        rate = rospy.Rate(self.update_rate)
        
        try:
            while not rospy.is_shutdown() and not self.is_at_target(target_positions):
                # Calculate velocities for controlled joints
                velocities = [0.0] * 6  # Initialize all velocities to 0
                
                # Update velocities only for controlled joints
                for i in range(len(self.controlled_joints)):
                    displacement = target_positions[i] - self.current_positions[i]
                    velocities[i] = self.get_bounded_velocity(displacement)
                
                # Create and send command
                msg = JointJog()
                msg.header.stamp = rospy.Time.now()
                msg.joint_names = self.all_joints
                msg.velocities = velocities
                msg.displacements = [0.0] * 6
                msg.duration = 1.0/self.update_rate
                
                self.joint_pub.publish(msg)
                
                # Log current status for controlled joints
                controlled_current = [self.current_positions[i] for i in range(4)]
                # rospy.loginfo("Current positions (joints 1-4): %s", [round(x, 3) for x in controlled_current])
                # rospy.loginfo("Target positions (joints 1-4): %s", [round(x, 3) for x in target_positions])
                # rospy.loginfo("Velocities (joints 1-4): %s", [round(x, 3) for x in velocities[:4]])
                # rospy.loginfo("---")
                
                rate.sleep()
            
            # Send stop command when target is reached
            self.send_stop_command()
            rospy.loginfo("Target position reached")
            return True
            
        except rospy.ROSInterruptException:
            rospy.logerr("Movement interrupted!")
            self.send_stop_command()
            return False

    def get_current_position(self):
        """Get current positions of controlled joints"""
        if self.current_positions is None:
            return None
        return list(self.current_positions[:4])  # Return only joints 1-4