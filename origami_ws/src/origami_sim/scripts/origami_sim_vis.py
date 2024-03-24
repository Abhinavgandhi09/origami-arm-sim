#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

is_shape = False
cur_shape = []


def getShape(msg):
    global cur_shape, is_shape
    cur_shape = msg.data
    is_shape = True

def main():
    global bridge, cur_shape, lam
    
    # ROS node init
    rospy.init_node("sim_fk")

    # Read sim parameters
    module_num = rospy.get_param("origami_sim/num_of_modules")
    rate = rospy.get_param("origami_sim/control_rate")
    points_per_segment = rospy.get_param("origami_sim/points_per_segment")
    
    # Subscribers
    origami_shape_sub = rospy.Subscriber("origami_vs/robot_shape", Float64MultiArray, getShape, queue_size=1)
    
    # Publishers
    img_pub = rospy.Publisher("origami_vs/camera_image", Image, queue_size=1)
    
    # Colors for drawing robot segments in image
    color = [(0, 97, 230), (155, 58, 93), (255, 255, 255)]
    
    # Loop rate
    r = rospy.Rate(rate)
    
    # Wait for sim to be ready
    while not is_shape:
        r.sleep()

    while not rospy.is_shutdown():
        # Define blank image
        img = np.zeros((480, 640, 3), np.uint8)
    
        # Draw base
        base_point = [cur_shape[0], cur_shape[1]]
        cv2.rectangle(img, (int(base_point[0]-25), int(base_point[1])-2),(int(base_point[0]+25), int(base_point[1])+2),(255,255,255),2)

        # Remove base point 
        cur_shape = cur_shape[2::]

        # draw robot
        for seg in range(module_num):
            for i in range(points_per_segment):
                point = [cur_shape[seg*2*points_per_segment+(2*i)], cur_shape[seg*2*points_per_segment+(2*i+1)]]
                cv2.circle(img, (int(point[0]), int(point[1])), 5, color[seg], -1)

        # Convert image to ros_msg and publish
        img_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        img_pub.publish(img_msg)
        
        # ROS sleep
        r.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()