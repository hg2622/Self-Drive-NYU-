#! /usr/bin/python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import CompressedImage
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import threading
import numpy

class ImageSubscriber:
    def __init__(self, topic_name):
        rospy.init_node('image_listener')
        self.image_topic = topic_name #/turtlebot3_burger_camera/image_raw/compressed
        # Instantiate CvBridge
        self.bridge = CvBridge()
        self.cv2_img = None

        rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)

    def image_callback(self, msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            # self.cv2_img = cv2.cvtColor(self.cv2_img, cv2.COLOR_BGR2RGB)


        except CvBridgeError as e:
            print(e)

    def get_frame(self):
        return self.cv2_img

    def process_image(self):
        while True:
            if self.cv2_img is not None:
                # Do some processing with the cv2_img variable
                # For example, you can display it or save it to a file
                cv2.imshow("Processed Image", numpy.array(list(reversed(self.cv2_img))))
                # cv2.imshow("Processed Image", self.cv2_img)
                cv2.waitKey(1)
                # cv2.destroyAllWindows()
            else:
                print("No image received yet. Please wait for the image callback.")

# uncomment it for testing
if __name__ == '__main__':
    img_subscriber = ImageSubscriber("/camera/image/compressed")
    rospy.sleep(1.0)
    
    # Start the process_image() method in a separate thread
    process_thread = threading.Thread(target=img_subscriber.process_image)
    process_thread.start()

    # Wait for the process_thread to finish (which won't happen in this case)
    process_thread.join()

    cv2.destroyAllWindows()