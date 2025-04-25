#!/usr/bin/env python3

import rospy
import yaml
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped

class TwoTransformPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('two_transform_publisher', anonymous=True)
        
        # Create a tf2 broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()
        
        # Load transforms from both files
        self.transform1 = self.load_transform_from_yaml('base_camera_tf.yaml')
        self.transform2 = self.load_transform_from_yaml('TCP_tag_0_tf.yaml')
        
        if self.transform1 is None and self.transform2 is None:
            rospy.logerr("Failed to load both transforms")
            return
            
        # Set up the timer to publish at 5Hz (0.2 seconds)
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publish_transforms)
        rospy.loginfo("Transform publisher started at 5Hz")

    def normalize_quaternion(self, q):
        """Normalize a quaternion to magnitude 1.0"""
        mag = np.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if mag == 0:
            rospy.logwarn("Zero magnitude quaternion, setting to identity")
            q.x = 0.0
            q.y = 0.0
            q.z = 0.0
            q.w = 1.0
        else:
            q.x /= mag
            q.y /= mag
            q.z /= mag
            q.w /= mag
        return q

    def load_transform_from_yaml(self, filename):
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)

            transform = TransformStamped()
            transform.header.frame_id = data['frame_id']
            transform.child_frame_id = data['child_frame_id']

            # Set translation
            transform.transform.translation.x = float(data['translation']['x'])
            transform.transform.translation.y = float(data['translation']['y'])
            transform.transform.translation.z = float(data['translation']['z'])

            # Set rotation and normalize
            transform.transform.rotation.x = float(data['rotation']['x'])
            transform.transform.rotation.y = float(data['rotation']['y'])
            transform.transform.rotation.z = float(data['rotation']['z'])
            transform.transform.rotation.w = float(data['rotation']['w'])
            
            transform.transform.rotation = self.normalize_quaternion(transform.transform.rotation)

            rospy.loginfo(f"Loaded transform from {filename}: {transform.header.frame_id} -> {transform.child_frame_id}")
            return transform

        except Exception as e:
            rospy.logerr(f"Failed to load transform from {filename}: {e}")
            return None

    def publish_transforms(self, event):
        current_time = rospy.Time.now()
        
        # Publish transform1 if loaded
        if self.transform1:
            self.transform1.header.stamp = current_time
            self.broadcaster.sendTransform(self.transform1)
        
        # Publish transform2 if loaded
        if self.transform2:
            self.transform2.header.stamp = current_time
            self.broadcaster.sendTransform(self.transform2)

if __name__ == '__main__':
    try:
        publisher = TwoTransformPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass