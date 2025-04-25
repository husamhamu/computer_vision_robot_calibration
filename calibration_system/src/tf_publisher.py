#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml

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

def publish_transforms():
    rospy.init_node('transform_publisher', anonymous=True)
    
    # Create a broadcaster
    br = tf2_ros.TransformBroadcaster()

    # Define the transform for pose1 with respect to tag_0
    t2 = TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "tag_0"
    t2.child_frame_id = "TCP_pose"
    t2.transform.translation.x = -0.5  # Example translation
    t2.transform.translation.y = 0.5
    t2.transform.translation.z = 0.0
    t2.transform.rotation.x = 0.0  # Example rotation (identity quaternion)
    t2.transform.rotation.y = 0.0
    t2.transform.rotation.z = 0.0
    t2.transform.rotation.w = 1.0

    # Define the transform for pose2 with respect to camera_frame
    t3 = TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "apriltag_bundle"
    t3.child_frame_id = "base_pose"
    t3.transform.translation.x = -0.167  # Example translation x = -167mm
    t3.transform.translation.y = 0.053   # y = 58mm
    t3.transform.translation.z = 0.0
    t3.transform.rotation.x = 0.0  # Example rotation (identity quaternion)
    t3.transform.rotation.y = 0.0
    t3.transform.rotation.z = 0.0
    t3.transform.rotation.w = 1.0

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Update the timestamp and publish all transforms
        # t1.header.stamp = rospy.Time.now()
        # t2.header.stamp = rospy.Time.now()
        t3.header.stamp = rospy.Time.now()

        # br.sendTransform(t1)
        # br.sendTransform(t2)
        br.sendTransform(t3)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_transforms()
    except rospy.ROSInterruptException:
        pass
