#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo, Imu
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImagePublisherNode:
    def __init__(self):
        self.counter = 0
        self.left_pub = rospy.Publisher('/output/left/image_rect_color', Image, queue_size=10)
        self.right_pub = rospy.Publisher('/output/right/image_rect_color', Image, queue_size=10)
        self.left_info_pub = rospy.Publisher('/output/left/camera_info', CameraInfo, queue_size=10)
        self.right_info_pub = rospy.Publisher('/output/right/camera_info', CameraInfo, queue_size=10)
        self.left_orig_pub = rospy.Publisher('/zed2i/zed_node/left_orig/image_rect_color', Image, queue_size=10)
        self.right_orig_pub = rospy.Publisher('/zed2i/zed_node/right_orig/image_rect_color', Image, queue_size=10)
        self.left_orig_info_pub = rospy.Publisher('/zed2i/zed_node/left_orig/camera_info', CameraInfo, queue_size=10)
        self.right_orig_info_pub = rospy.Publisher('/zed2i/zed_node/right_orig/camera_info', CameraInfo, queue_size=10)


        self.imu_pub = rospy.Publisher('/output/imu/data', Imu, queue_size=10)

        self.left_sub = rospy.Subscriber('/zed2i/zed_node/left/image_rect_color', Image, self.left_image_callback)
        self.right_sub = rospy.Subscriber('/zed2i/zed_node/right/image_rect_color', Image, self.right_image_callback)
        self.left_info_sub = rospy.Subscriber('/zed2i/zed_node/left/camera_info', CameraInfo, self.left_image__info_callback)
        self.right_info_sub = rospy.Subscriber('/zed2i/zed_node/right/camera_info', CameraInfo, self.right_image_info_callback)


        #subscibe to imu data
        self.imu_sub = rospy.Subscriber('/zed2i/zed_node/imu/data', Imu, self.imu_callback)

        self.bridge = CvBridge()
    
    def left_image_callback(self, msg):


        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)




        self.counter += 1
        if self.counter >= 1000 and self.counter < 1060: # and self.counter % 2 == 0:
                height, width, _ = cv_image.shape

                
                # empty_image = np.zeros((height, width, 3), np.uint8)
                empty_image = np.ones((height, width, 3), np.uint8)
                empty_msg = self.bridge.cv2_to_imgmsg(empty_image, "bgr8")
                # add time from the original image
                empty_msg.header.stamp = msg.header.stamp
                
                print("Publishing empty left image ", self.counter)
                if self.counter%2 ==0:
                    self.left_pub.publish(empty_msg)
                    self.left_orig_pub.publish(msg)
        elif self.counter == 1060:
            self.counter = 0
        else:
            # print that image is being published
            print("Publishing image")

            if self.counter%2 ==0:
                self.left_pub.publish(msg)
                self.left_orig_pub.publish(msg)

    def right_image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)



        if self.counter >= 1000 and self.counter < 1060: # and self.counter % 2 == 0:
            height, width, _ = cv_image.shape
            empty_image = np.zeros((height, width, 3), np.uint8)
            empty_msg = self.bridge.cv2_to_imgmsg(empty_image, "bgr8")

            # add time from the original image
            empty_msg.header.stamp = msg.header.stamp  
            print("Publishing empty right image ", self.counter)
            if self.counter%2 ==0:
                self.right_pub.publish(empty_msg)
                self.right_orig_pub.publish(msg)

        else:
            if self.counter%2 ==0:
                self.right_pub.publish(msg)
                self.right_orig_pub.publish(msg)


    def left_image__info_callback(self, msg):
        self.left_info_pub.publish(msg)
        self.left_orig_info_pub.publish(msg)

    def right_image_info_callback(self, msg):
        self.right_info_pub.publish(msg)
        self.right_orig_info_pub.publish(msg)


    def imu_callback(self, msg):
        self.imu_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('image_publisher_node')
    node = ImagePublisherNode()
    rospy.spin()