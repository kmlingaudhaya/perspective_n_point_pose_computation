#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray

def image_points_publisher():
    rospy.init_node('image_points_publisher', anonymous=True)
    image_points_pub = rospy.Publisher('/image_points', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        # Provide specific image points
        image_points = [
            [1154.8, 451.4],
            [1159.3, 406.9],
            [1342.5, 458.5],
            [1344.5, 417.8],  
        ]
        print(image_points)

        # Create Float32MultiArray message
        image_points_msg = Float32MultiArray()
        image_points_msg.data = [coord for point in image_points for coord in point]

        # Publish the message
        image_points_pub.publish(image_points_msg)

        # Sleep for a while before publishing the next set of points
        rate.sleep()

if __name__ == '__main__':
    try:
        image_points_publisher()
    except rospy.ROSInterruptException:
        pass
