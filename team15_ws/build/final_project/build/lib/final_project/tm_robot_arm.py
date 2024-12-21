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

PATH_CALIB_IMG = "./calibration_imgs"
PATH_BLOCK_IMG = "./block_imgs"
PATH_CALIB_INFO = './calibration_info'
PATH_TEMP_FILE = './temp_files'

class TM_ROBOT_ARM(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.arm_cli = self.create_client(SendScript, 'send_script')
        self.gripper_cli = self.create_client(SetIO, 'set_io')
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
        self.image_mode = None # "calibration", "blocks", "hw4_blocks", "canvas"
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
            os.makedirs(PATH_CALIB_IMG, exist_ok=True)
            save_img_path = os.path.join(PATH_CALIB_IMG, f'cap_{self.idx}.jpg')
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
        elif self.image_mode == "canvas": # save image of ArUco markers to estimate canvas location
            # create save directory if not exists
            os.makedirs(PATH_TEMP_FILE, exist_ok=True)
            save_img_path = os.path.join(PATH_TEMP_FILE, 'canvas.jpg')
            cv2.imwrite(save_img_path, img)
            print(f'>>>> Saved canvas image!')
        else:
            print("image_mode need to be 'calibration', 'blocks', 'hw4_blocks' or 'canvas'.")
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
    
    def move_robot_arm(self, pos: list):
        """
        Move robot arm to the given position.

        Parameters:
            pos (list): A list in the format [x, y, z, Rx, Ry, Rz].
        """
        #          "   X,        Y,        Z,       Rx,        Ry,       Rz   "
        pos_str = f"{pos[0]}, {pos[1]}, {pos[2]}, {pos[3]}, {pos[4]}, {pos[5]}"
        script = "PTP(\"CPP\","+pos_str+",100,200,0,false)"
        self.send_script(script)
        print(f"send script: {pos_str}")

    def capture_image(self):
        self.send_script("Vision_DoJob(job1)")
        print(f"send Vision_DoJob")
        self.get_image = False