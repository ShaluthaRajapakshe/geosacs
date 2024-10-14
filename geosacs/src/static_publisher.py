#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage

class StaticTFPublisher:
    def __init__(self):
        rospy.init_node('static_tf_publisher')

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transforms = []

        # Define the static transforms
        self.transforms = [
            ("lio_link6G", "lio_gripper_interface_link", [0, 0, 0.046], [0, 0, 0]),
            ("lio_gripper_interface_link", "lio_gripper_link", [-0.052, 0, 0.052], [0, -1.57, 3.14]),
            ("lio_gripper_link", "lio_left_finger", [0.0645, 0, 0.052], [0, 0, 0]),
            ("lio_left_finger", "lio_tcp_link", [0.189, 0, 0.052], [0, 0, 0])
        ]

        # Subscribe to the TF topic
        self.tf_sub = rospy.Subscriber("/tf_static", TFMessage, self.tf_callback)  #debug with teh real robot
        self.missing_transforms = set([child for _, child, _, _ in self.transforms])

    def tf_callback(self, msg):
        for transform in msg.transforms:
            child_frame_id = transform.child_frame_id
            if child_frame_id in self.missing_transforms:
                self.missing_transforms.remove(child_frame_id)

        # If all required transforms are present, no need to publish static transforms
        if not self.missing_transforms:
            rospy.loginfo("All required transforms are present. No need to publish static transforms.")
            rospy.signal_shutdown("All required transforms are present.")
        else:
            rospy.loginfo(f"Missing transforms: {self.missing_transforms}")
            self.publish_static_transforms()

    def publish_static_transforms(self):
        static_transforms = []

        for parent, child, translation, rotation in self.transforms:
            static_transform = geometry_msgs.msg.TransformStamped()

            static_transform.header.stamp = rospy.Time.now()
            static_transform.header.frame_id = parent
            static_transform.child_frame_id = child

            static_transform.transform.translation.x = translation[0]
            static_transform.transform.translation.y = translation[1]
            static_transform.transform.translation.z = translation[2]

            quat = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
            static_transform.transform.rotation.x = quat[0]
            static_transform.transform.rotation.y = quat[1]
            static_transform.transform.rotation.z = quat[2]
            static_transform.transform.rotation.w = quat[3]

            static_transforms.append(static_transform)

        self.broadcaster.sendTransform(static_transforms)
        rospy.loginfo("Static transforms published.")

if __name__ == '__main__':
    try:
        StaticTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
