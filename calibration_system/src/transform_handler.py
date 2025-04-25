#!/usr/bin/env python3

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class TransformHandler:
    def __init__(self):
        rospy.init_node('transform_handler', anonymous=True)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_transform(self, source_frame, target_frame):
        try:
            # Wait for transform to become available
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rospy.Time(0),
                rospy.Duration(5.0)
            )
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Failed to get transform: {e}")
            return None

    def save_transform_to_yaml(self, transform, filename):
        if transform is None:
            rospy.logerr("No transform to save")
            return False

        # Extract position and orientation
        transform_data = {
            'translation': {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'z': transform.transform.translation.z
            },
            'rotation': {
                'x': transform.transform.rotation.x,
                'y': transform.transform.rotation.y,
                'z': transform.transform.rotation.z,
                'w': transform.transform.rotation.w
            },
            'frame_id': transform.header.frame_id,
            'child_frame_id': transform.child_frame_id
        }

        # Save to YAML file
        try:
            with open(filename, 'w') as file:
                yaml.dump(transform_data, file, default_flow_style=False)
            rospy.loginfo(f"Transform saved to {filename}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to save transform: {e}")
            return False

    def load_transform_from_yaml(self, filename):
        try:
            with open(filename, 'r') as file:
                transform_data = yaml.safe_load(file)

            # Create TransformStamped message
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = transform_data['frame_id']
            transform.child_frame_id = transform_data['child_frame_id']

            # Set translation
            transform.transform.translation.x = transform_data['translation']['x']
            transform.transform.translation.y = transform_data['translation']['y']
            transform.transform.translation.z = transform_data['translation']['z']

            # Set rotation
            transform.transform.rotation.x = transform_data['rotation']['x']
            transform.transform.rotation.y = transform_data['rotation']['y']
            transform.transform.rotation.z = transform_data['rotation']['z']
            transform.transform.rotation.w = transform_data['rotation']['w']

            return transform
        except Exception as e:
            rospy.logerr(f"Failed to load transform: {e}")
            return None

if __name__ == '__main__':
    try:
        handler = TransformHandler()
        
        # Example usage for saving transform
        # transform = handler.get_transform('camera_frame', 'base_pose') # Fixed tf of camera relative to base
        # transform = handler.get_transform('tag_0', 'TCP_pose') # Fixed tf of tag_0 relative to tag_1
        # transform = handler.get_transform('tag_0', 'tag_1') # Fixed tf of tag_0 relative to tag_1
        transform = handler.get_transform('base_pose', 'tag_0') # Fixed tf of camera relative to base
        # transform = handler.get_transform('tag_0', 'tag_1') 

        if transform:
            handler.save_transform_to_yaml(transform, 'transform.yaml')

        # Example usage for loading transform
        loaded_transform = handler.load_transform_from_yaml('transform.yaml')
        if loaded_transform:
            rospy.loginfo("Successfully loaded transform")
            # Print the loaded transform
            rospy.loginfo(f"Translation: {loaded_transform.transform.translation}")
            rospy.loginfo(f"Rotation: {loaded_transform.transform.rotation}")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass