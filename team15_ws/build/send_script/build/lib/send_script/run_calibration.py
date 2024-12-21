#!/usr/bin/env python
RUN_WITH_TMROBOT = True
if RUN_WITH_TMROBOT:
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
from send_script.camera import Camera
from send_script.tm_robot_arm import TM_ROBOT_ARM

PATH_CAPTURE_IMG = "./captured_imgs"
PATH_CALIB_INFO = './calibration_info'

# define the size in mm in chessboard
SQR_SIZE = 25 ### MANUAL EDIT ###

# define the number of chessboard corners
SQR_NUM = (8, 6) ### MANUAL EDIT ###
 
#             "X, Y, Z, Rx, Ry, Rz"
ARM_POINTS = ["230.00, 230, 730, -180.00, 0.0, 135.00", 
              "300.00, 100, 500, -180.00, 0.0, 135.00", 
              "272.00, 230, 687, -180.00, 0.0, 135.00", 
              "235.00, 283, 683, -180.00, 0.0, 135.00", 
              "309.00, 206, 671, -180.00, 0.0, 135.00", 
              "325.00, 278, 645, -180.00, 0.0, 135.00", 
              "307.00, 205, 623, -180.00, 0.0, 135.00", 
              "340.00, 237, 588, -180.00, 0.0, 135.00", 
              "324.00, 230, 622, -180.00, 0.0, 135.00", 
              "214.00, 309, 620, -180.00, 0.0, 135.00", 
              "227.00, 309, 585, -180.00, 0.0, 135.00", 
              "239.00, 304, 552, -180.00, 0.0, 135.00", 
              "243.00, 235, 493, -180.00, 0.0, 135.00", 
              "285.00, 296, 731, -180.00, 0.0, 135.00", 
              "320.00, 209, 726, -180.00, 0.0, 135.00", 
              "355.00, 248, 685, -180.00, 0.0, 135.00", 
              "325.00, 200, 683, -180.00, 0.0, 135.00", 
              "171.00, 312, 673, -180.00, 0.0, 135.00", 
              "231.00, 360, 636, -180.00, 0.0, 135.00", 
              "252.00, 258, 621, -180.00, 0.0, 135.00"]

def main(args=None):
    print("run_calibration START")
    camera = Camera(PATH_CALIB_INFO, 1280, 960)
    # create save directory if not exists
    os.makedirs(PATH_CAPTURE_IMG, exist_ok=True)

    take_photo = False

    if RUN_WITH_TMROBOT:
        rclpy.init(args=args)
        node = TM_ROBOT_ARM('calibration')
        node.image_mode = "calibration"

        # move arm and take photos
        if take_photo:
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            node.get_image = True
            for i, arm_point in enumerate(ARM_POINTS):
                while(not node.get_image):
                    print("waiting for image")
                    rclpy.spin_once(node)
                script = "PTP(\"CPP\","+arm_point+",100,200,0,false)"
                node.send_script(script)
                print(f"send script: {arm_point}")
                cv2.waitKey(0) # wait for arm to reach target, and then press esc key to capture image
                node.send_script("Vision_DoJob(job1)")
                print(f"send Vision_DoJob")
                node.get_image = False
                cv2.waitKey(1000) # wait 1 second
            cv2.destroyAllWindows()

    if not take_photo:
        # ---------- run calibration ----------
        # Intrinsic Calibration
        camera.intrinsic.run_calibration(PATH_CAPTURE_IMG, SQR_SIZE, SQR_NUM)

        # Extrinsic Calibration
        # camera.intrinsic.load_param()
        mtx = camera.intrinsic.mat_cam
        dist = camera.intrinsic.dist_coeff
        camera.extrinsic.run_calibration_eye_in_hand(ARM_POINTS, mtx, dist, PATH_CAPTURE_IMG, SQR_SIZE, SQR_NUM, False)

        # check result using ARM_POINT "230.00, 230, 730, -180.00, 0.0, 135.00" and image center
        XYZ = camera.compute_XYZ_eye_in_hand(ARM_POINTS[0], camera.width // 2, camera.height // 2)
        print("XYZ: ", XYZ)

    if RUN_WITH_TMROBOT:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
