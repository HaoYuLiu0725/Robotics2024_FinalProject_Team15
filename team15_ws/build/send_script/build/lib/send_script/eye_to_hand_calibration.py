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
    from send_script.tm_robot_arm import TM_ROBOT_ARM

import cv2
import numpy as np
import os
import glob
from math import atan2, cos, sin
from send_script.camera import Camera

PATH_CAPTURE_IMG = "./captured_imgs"
PATH_BLOCK_IMG = "./block_imgs"
PATH_CALIB_INFO = './calibration_info'

# define the size in mm in chessboard
SQR_SIZE = 25 ### MANUAL EDIT ###

# define the number of chessboard corners
SQR_NUM = (8, 6) ### MANUAL EDIT ###

PICK_Z = 102            # height to pick the block
PICK_READY_OFFSET = 30  # offset Z for a ready pos, aim to perform a vertical movement to pick up the block 
LIFT_HEIGHT = 200       # lift block to a safty height
DROP_Z = 102            # height to drop the block

#             "X, Y, Z, Rx, Ry, Rz"
INITIAL_POS = "230.00, 230, 730, -180.00, 0.0, 135.00"

def create_grid(p1, p2, grid_num=5):
    p1 = np.array(p1)
    p2 = np.array(p2)
    center = (p1 + p2) / 2
    theta_line = atan2(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))
    theta = theta_line - np.pi / 4  # rotate 45 degrees
    
    # Vectors from center to each point
    vector1 = np.array([p1[0] - center[0], p1[1] - center[1]])
    vector2 = np.array([p2[0] - center[0], p2[1] - center[1]])
    
    # Function to rotate a vector
    def rotate_2d(vector, angle):
        rotation_matrix = np.array([
            [cos(angle), -sin(angle)],
            [sin(angle), cos(angle)]
        ])
        return np.dot(rotation_matrix, vector)
    
    # Rotate both vectors by the calculated angle
    rotated_vector1 = rotate_2d(vector1, -theta * 2)
    rotated_vector2 = rotate_2d(vector2, -theta * 2)
    
    # Get the rotated points
    p3 = center + rotated_vector1
    p4 = center + rotated_vector2

    print(f"p3: {p3}")
    print(f"p4: {p4}")

    # Create line segments
    L14 = np.linspace(p1, p4, grid_num)
    L32 = np.linspace(p3, p2, grid_num)
    grid_points = []
    for i in range(grid_num):
        L_temp = np.linspace(L14[i], L32[i], grid_num)
        if(i%2 == 0):
            grid_points.extend(L_temp)
        else:
            grid_points.extend(np.flip(L_temp, axis=0))
        
    print(grid_points)
    return grid_points

P1 = (535, 257)
P2 = (100, 334)
BLOCK_POINTS = create_grid(P1, P2, 5)

def main(args=None):
    print("eye TO hand calibration START")
    camera = Camera(PATH_CALIB_INFO, 1280, 960)

    get_calibration_image = True
    get_block_image = True
    run_intrinsic_calibration = True
    run_extrinsic_calibration = True

    if RUN_WITH_TMROBOT:
        rclpy.init(args=args)
        tm_robot = TM_ROBOT_ARM('eye_to_hand_calibration')

        if get_calibration_image: 
            # take photos at INITIAL_POS for Intrinsic Calibration
            tm_robot.image_mode = "calibration"
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            script = "PTP(\"CPP\","+INITIAL_POS+",100,200,0,false)"
            tm_robot.send_script(script)
            print(f"send script: {INITIAL_POS}")

            print("\nTake photos for Intrinsic Calibration......")
            print("Press X to capture image")
            print("Press Q to exit")
            tm_robot.idx = 0
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('x'):
                    tm_robot.send_script("Vision_DoJob(job1)")
                    print(f"send Vision_DoJob")
                    tm_robot.get_image = False
                    while(not tm_robot.get_image):
                        print("waiting for image...")
                        rclpy.spin_once(tm_robot)
                    print(f"Saved {tm_robot.idx} images ~ ~ ~")
                elif key == ord('q'):
                    print('>>>> Exit')
                    break
            cv2.destroyAllWindows()

        if get_block_image: 
            # Use the robot arm to place the blocks at the designated poses on the table, 
            # then move to the INITIAL_POS to take photos for Extrinsic calibration.
            tm_robot.image_mode = "blocks"
            tm_robot.idx = 0
            def move_robot_arm(XY_pos, Z):
                """
                Move robot arm to the given position.

                Parameters:
                    XY_pos: [X, Y]
                    Z: Z height
                """
                #          "    X,            Y,       Z,   Rx,  Ry, Rz"
                pos_str = f"{XY_pos[0]}, {XY_pos[1]}, {Z}, -180, 0, 135"
                script = "PTP(\"CPP\","+pos_str+",100,200,0,false)"
                tm_robot.send_script(script)
                print(f"send script: {pos_str}")
            
            ###### robot arm control
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            tm_robot.set_io(0.0) # 0.0: open gripper
            script = "PTP(\"CPP\","+INITIAL_POS+",100,200,0,false)"
            tm_robot.send_script(script)
            print(f"send script: {INITIAL_POS}")
            print(">>>>> Press any key to close gripper ~ ~ ~")
            cv2.waitKey(0) # wait for arm to reach target, and then press esc key to close gripper
            tm_robot.set_io(1.0) # 1.0: close gripper
            cv2.waitKey(500) # delay
            
            for i, block_pos in enumerate(BLOCK_POINTS):
                print(f"Go to placed block {i}: {block_pos}")
                move_robot_arm(block_pos, DROP_Z)
                cv2.waitKey(0) # delay
                tm_robot.set_io(0.0) # 0.0: open gripper
                cv2.waitKey(0) # delay
                move_robot_arm(block_pos, LIFT_HEIGHT)

                print(f"Back to INITIAL_POS")
                script = "PTP(\"CPP\","+INITIAL_POS+",100,200,0,false)"
                tm_robot.send_script(script)
                print(f"send script: {INITIAL_POS}")

                print(">>>>> Press any key to capture image ~ ~ ~")
                cv2.waitKey(0) # wait for arm to reach target, and then press esc key to capture image
                tm_robot.send_script("Vision_DoJob(job1)")
                print(f"send Vision_DoJob")
                tm_robot.get_image = False
                cv2.waitKey(1000) # wait 1 second
                while(not tm_robot.get_image):
                    print("waiting for image")
                    rclpy.spin_once(tm_robot)

                print(f"Go to grab block {i}: {block_pos}")
                move_robot_arm(block_pos, PICK_Z + PICK_READY_OFFSET) # offset Z for a ready pos, aim to perform a vertical movement to pick up the block
                move_robot_arm(block_pos, PICK_Z)
                cv2.waitKey(0) # delay
                print(f"Pick block {i} !!!")
                tm_robot.set_io(1.0) # 1.0: close gripper
                cv2.waitKey(0) # delay
                move_robot_arm(block_pos, LIFT_HEIGHT) # lift block
            cv2.destroyAllWindows()
            
    # ---------- run calibration ----------
    if run_intrinsic_calibration:
        # Intrinsic Calibration
        camera.intrinsic.run_calibration(PATH_CAPTURE_IMG, SQR_SIZE, SQR_NUM, True)

    if run_extrinsic_calibration:
        # Get IMAGE_POINT
        print('>>>> Reading Blocks images...')
        find_img_path = os.path.join(PATH_BLOCK_IMG, '*.jpg')
        img_names = glob.glob(find_img_path)

        # Define a sorting key to extract the numeric part of the filename
        def _extract_number(file_path):
            base_name = os.path.basename(file_path)  # Get the file name (e.g., cap_0.jpg)
            number = os.path.splitext(base_name)[0].split('_')[-1]  # Extract the numeric part
            return int(number)  # Convert to integer for numerical sorting

        # Sort the list
        img_names_sorted = sorted(img_names, key=_extract_number)
        print(img_names_sorted)

        IMAGE_POINTS = []
        for fname in img_names_sorted:
            img = cv2.imread(str(fname))
            centroid = camera.get_centroid(img, timeout=100)
            temp = [int(x) for x in centroid[0][:2]]
            IMAGE_POINTS.append(temp)
        
        IMAGE_POINTS = np.array(IMAGE_POINTS, dtype=np.float32)
        print(f"Get {len(IMAGE_POINTS)} image points:")
        print(IMAGE_POINTS)

        WORLD_POINT = []
        for point in BLOCK_POINTS:
            temp = point.tolist()
            temp.append(PICK_Z)
            WORLD_POINT.append(temp)

        WORLD_POINT = np.array(WORLD_POINT, dtype=np.float32)
        print(f"Get {len(WORLD_POINT)} world points:")
        print(WORLD_POINT)
        
        # Extrinsic Calibration
        camera.intrinsic.load_param()
        mtx = camera.intrinsic.mat_cam
        dist = camera.intrinsic.dist_coeff
        mtx_new_inv = camera.intrinsic.mat_cam_new_inv
        camera.extrinsic.run_calibration_eye_to_hand(WORLD_POINT, IMAGE_POINTS, mtx, dist, mtx_new_inv)

        print("Finish")

    if RUN_WITH_TMROBOT:
        rclpy.spin(tm_robot)
        tm_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
