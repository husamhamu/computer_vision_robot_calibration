#!/usr/bin/env python3

import rospy
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
import numpy as np

class JointPositionPublisher:
    def __init__(self):
        # rospy.init_node('joint_position_publisher', anonymous=True)
        
        # Current joint positions
        self.current_positions = None
        
        # Publisher for joint positions
        self.joint_pub = rospy.Publisher('/JointJog', JointJog, queue_size=10)
        
        # Subscriber for current joint states
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        
        # Joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Movement parameters
        self.position_tolerance = 0.018  # radians
        self.max_velocity = 0.4       # radians/sec
        self.update_rate = 10          # Hz
        
        # Wait for first joint states message
        rospy.loginfo("Waiting for joint states...")
        while self.current_positions is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Received initial joint states!")

    def joint_states_callback(self, msg):
        """Callback for joint states subscriber"""
        self.current_positions = msg.position
        
    def is_at_target(self, target_positions, tolerance=None):
        """Check if robot is at target position within tolerance"""
        if tolerance is None:
            tolerance = self.position_tolerance
            
        return all(abs(current - target) < tolerance 
                  for current, target in zip(self.current_positions, target_positions))
    
    def get_bounded_velocity(self, displacement):
        """Limit velocity based on displacement"""
        return np.clip(displacement, -self.max_velocity, self.max_velocity)

    def send_stop_command(self):
        """Send a command to stop all joint movement"""
        msg = JointJog()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = self.joint_names
        msg.velocities = [0.0] * 6     # Zero velocity for all joints
        msg.displacements = [0.0] * 6  # Zero displacement
        msg.duration = 0.1             # Short duration
        
        # Publish the stop command several times to ensure it's received
        for _ in range(3):
            self.joint_pub.publish(msg)
            rospy.sleep(0.05)
        
    def send_joint_positions(self, target_positions):
        """
        Send joint positions to the robot.
        
        Args:
            target_positions (list): List of 6 target joint positions in radians
        """
        if len(target_positions) != 6:
            rospy.logerr("Expected 6 joint positions, got %d", len(target_positions))
            return
            
        if self.current_positions is None:
            rospy.logerr("No joint states received yet!")
            return
        
        rate = rospy.Rate(self.update_rate)
        
        while not rospy.is_shutdown() and not self.is_at_target(target_positions):
            # Calculate displacements
            displacements = [target - current 
                           for target, current in zip(target_positions, self.current_positions)]
            
            # Calculate velocities (bounded by max_velocity)
            velocities = [self.get_bounded_velocity(d) for d in displacements]
            
            # Create JointJog message
            msg = JointJog()
            msg.header.stamp = rospy.Time.now()
            msg.joint_names = self.joint_names
            msg.velocities = velocities    # Use velocities instead of displacements
            msg.displacements = [0.0] * 6  # Set displacements to 0
            msg.duration = 1.0/self.update_rate
            
            # Publish the message
            self.joint_pub.publish(msg)
            
            # Log current status
            # rospy.loginfo("Current positions: %s", [round(x, 3) for x in self.current_positions])
            # rospy.loginfo("Target positions: %s", [round(x, 3) for x in target_positions])
            # rospy.loginfo("Velocities: %s", [round(x, 3) for x in velocities])
            # rospy.loginfo("---")
            
            rate.sleep()
        
        # Send stop command when target is reached
        rospy.loginfo("Target reached! Stopping robot...")
        self.send_stop_command()
        rospy.loginfo("Robot stopped.")
        return True

def main():
    publisher = JointPositionPublisher()
    
    # Define a sequence of positions to test
    test_positions = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Home position
        # [0.3, 0.0, 0.0, 0.0, 0.0, 0.0],  # Move joint 1
        # [0.3, 0.2, 0.0, 0.0, 0.0, 0.0],  # Move joint 2
        # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Back to home
    ]
    
    try:
        for i, positions in enumerate(test_positions):
            rospy.loginfo(f"\nMoving to position {i + 1}/{len(test_positions)}")
            publisher.send_joint_positions(positions)
            rospy.sleep(1)  # Wait between movements
            
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()