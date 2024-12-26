import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import os
import datetime
import time
import json
import pandas as pd


from pynput import keyboard
import threading

class ImageSubscriber:
    def __init__(self, topic_name):
        rospy.init_node('image_listener', anonymous=True)
        self.image_topic = topic_name
        self.bridge = CvBridge()
        self.cv2_img = None
        self.time_stamp = "demo_2" #datetime.datetime.now().strftime("%Y%m%d_%H%M%S")  # Create a time stamp for the current time
        self.images_folder_path, self.annotations_folder_path = self.create_folders()
        self.keyboard_value = 'None'  # Initialize the keyboard value to None
        self.start_keyboard_listener()

        rospy.Subscriber(self.image_topic, CompressedImage, self.image_callback)
    
    def on_press(self, key):
        try:
            self.keyboard_value = key.char  
        except AttributeError:
            pass
    
    def start_keyboard_listener(self):
        listener = keyboard.Listener(on_press=self.on_press)
        listener.start()


    def image_callback(self, msg):
        try:
            self.cv2_img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def get_frame(self):
        return self.cv2_img

    def create_folders(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        base_path = os.path.join(script_dir, "../dataset")
        
        time_stamp_folder = self.time_stamp  # Name the folder with the time stamp

        images_folder_path = os.path.join(base_path, "images", time_stamp_folder)
        annotations_folder_path = os.path.join(base_path, "annotations", time_stamp_folder)  
        # Create the folders if they don't exist
        if not os.path.exists(images_folder_path):
            os.makedirs(images_folder_path)
        if not os.path.exists(annotations_folder_path):
            os.makedirs(annotations_folder_path)
        
        return images_folder_path, annotations_folder_path

    def process_image(self):
        img_count = 0
        while True:
            start_time = time.time()
            if self.cv2_img is not None:
                img_height, img_width = self.cv2_img.shape[:2]
                for _ in range(2):
                    img_name = f"{img_count:08d}.jpg"
                    json_name = f"{img_count:08d}.json"
                    img_path = os.path.join(self.images_folder_path, img_name)
                    json_path = os.path.join(self.annotations_folder_path, json_name)
                    img_time = str(pd.Timestamp.now())
                    cv2.imwrite(img_path, self.cv2_img)
                    cv2.imshow("Processed Image", self.cv2_img)
                    cv2.waitKey(1)

                    data = {
                        "image_id": img_count,
                        "file_name": os.path.join("images", self.time_stamp, img_name),
                        "height": img_height,
                        "width": img_width,
                        "annotations": [
                            {
                                "keyboard_value": self.keyboard_value,
                                "timestamp": img_time
                                
                            }
                        ]
                    }

                    with open(json_path, 'w') as json_file:
                        json.dump(data, json_file, indent=4)

                    img_count += 1
                    if img_count % 2 == 0:  
                        time_elapsed = time.time() - start_time
                        time_to_wait = max(0, 1 - time_elapsed)
                        time.sleep(time_to_wait)
            else:
                print("No image received yet. Please wait for the image callback.")
                time.sleep(0.5)

if __name__ == '__main__':
    img_subscriber = ImageSubscriber("/camera/image/compressed")
    rospy.sleep(1.0)
    
    process_thread = threading.Thread(target=img_subscriber.process_image)
    process_thread.start()
    rospy.spin()  # Using rospy.spin() to keep your application for exiting until the node is stopped
    process_thread.join()

    cv2.destroyAllWindows()
