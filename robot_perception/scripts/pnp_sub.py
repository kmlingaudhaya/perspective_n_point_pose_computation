#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class PoseEstimationNode:
    def __init__(self):
        rospy.init_node('pose_estimation_node', anonymous=True)
        self.bridge = CvBridge()

        # Replace these with your actual values
        # Replace these values with your actual camera calibration parameters
        fx = 1068.0500
        fy = 1067.7200
        cx = 1157.4301
        cy = 633.6560

        k1 = -0.0464
        k2 = 0.0184
        p1 = 0.0005
        p2 = -0.0002
        k3 = -0.0079
        self.camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.distortion_coefficients = np.array([k1, k2, p1, p2, k3])

        # Object points in the arrow mark board's coordinate system
        self.object_points = np.array([[0, 0, 0], [0, 5, 0], [12.68, 0, 0], [12.68, 5, 0]], dtype=np.float32)

        self.image_points_subscriber = rospy.Subscriber('/image_points', Float32MultiArray, self.image_points_callback)

    def image_points_callback(self, image_points_msg):
        # Convert the received Float32MultiArray to a NumPy array
        image_points = np.array(image_points_msg.data, dtype=np.float32).reshape(-1, 2)

        # Solve for pose using solvePnP
        _, rvecs, tvecs = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.distortion_coefficients)

        # Calculate the rotation matrix from the rotation vector
        R, _ = cv2.Rodrigues(rvecs)

        # Extract roll, pitch, and yaw angles from the rotation matrix
        roll_angle, pitch_angle, yaw_angle = cv2.RQDecomp3x3(R)[0]

        # Display intermediate results for debugging
        rospy.loginfo("Received Image Points:")
        rospy.loginfo(image_points)
        rospy.loginfo("Translation vector (X, Y, Z): {:.2f}, {:.2f}, {:.2f} units (e.g., centimeters)".format(*tvecs.ravel()))
        rospy.loginfo("Rotation matrix:")
        rospy.loginfo(R)
        rospy.loginfo("Rotation angles (Roll, Pitch, Yaw): {:.2f}, {:.2f}, {:.2f} degrees".format(np.degrees(roll_angle),
                                                                                                    np.degrees(pitch_angle),
                                                                                                    np.degrees(yaw_angle)))


if __name__ == '__main__':
    try:
        node = PoseEstimationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
