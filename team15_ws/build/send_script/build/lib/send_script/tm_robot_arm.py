import rclpy
from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os

PATH_CAPTURE_IMG = "./captured_imgs"
PATH_BLOCK_IMG = "./block_imgs"
PATH_CALIB_INFO = './calibration_info'

class TM_ROBOT_ARM(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.arm_cli = self.create_client(SendScript, 'send_script')
        self.gripper_cli = self.create_client(SetIO, 'set_io')
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
        self.image_mode = None # "calibration", "blocks", "hw4_blocks"
        self.get_image = False
        self.idx = 0

    def image_callback(self, data):
        self.get_logger().info('Received image')

        # TODO (write your code here)
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        cv2.namedWindow("techman_image", cv2.WINDOW_FULLSCREEN)
        cv2.imshow("techman_image", img)

        if self.image_mode == "calibration": # save image for calibration
            # create save directory if not exists
            os.makedirs(PATH_CAPTURE_IMG, exist_ok=True)
            save_img_path = os.path.join(PATH_CAPTURE_IMG, f'cap_{self.idx}.jpg')
            cv2.imwrite(save_img_path, img)
            print(f'>>>> Saved calibration image! idx = {self.idx}')
            self.idx += 1
        elif self.image_mode == "blocks": # save image for blocks
            # create save directory if not exists
            os.makedirs(PATH_BLOCK_IMG, exist_ok=True)
            save_img_path = os.path.join(PATH_BLOCK_IMG, f'block_{self.idx}.jpg')
            cv2.imwrite(save_img_path, img)
            print(f'>>>> Saved block image! idx = {self.idx}')
            self.idx += 1
        elif self.image_mode == "hw4_blocks": # save image for HW4 operation
            cv2.imwrite("hw4_blocks.jpg", img)
            print(f'>>>> Saved hw4_blocks image!')
        else:
            print("image_mode need to be 'calibration', 'blocks' or 'hw4_blocks'.")
            print("continue for waiting image......")

        cv2.waitKey(1000) # wait 1 second
        cv2.destroyWindow("techman_image")
        self.get_image = True

    # arm client
    def send_script(self, script):
        while not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not availabe, waiting again...')

        move_cmd = SendScript.Request()
        move_cmd.script = script
        self.future = self.arm_cli.call_async(move_cmd)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    # gripper client
    def set_io(self, state):
        while not self.gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not availabe, waiting again...')
        
        io_cmd = SetIO.Request()
        io_cmd.module = 1
        io_cmd.type = 1
        io_cmd.pin = 0
        io_cmd.state = state
        self.future = self.gripper_cli.call_async(io_cmd)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()