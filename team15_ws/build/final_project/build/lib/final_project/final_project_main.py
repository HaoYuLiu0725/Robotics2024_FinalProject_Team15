#!/usr/bin/env python
RUN_WITH_TMROBOT = True
import sys
if RUN_WITH_TMROBOT:
    import rclpy
    from rclpy.node import Node

    sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
    from tm_msgs.msg import *
    from tm_msgs.srv import *

    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from final_project.tm_robot_arm import TM_ROBOT_ARM

import cv2
import numpy as np
import os
import csv
import copy
import subprocess

# Camera
from final_project.camera import Camera
# canvas_localization
from final_project.canvas_localization_new import CanvasLocalization
# generate_path
from final_project.edgegroup_pure import EdgeGroup

PATH_CALIB_INFO = './calibration_info'
PATH_TEMP_FILE = './temp_files'

#             "  X,     Y,   Z,     Rx,    Ry,   Rz  "
INITIAL_POS = [230.00, 230, 730, -180.00, 0.0, 135.00]

# define the size in meters in aruco marker
ARUCO_MARKER_SIZE = 0.019

DRAW_Z = 212    # height to draw (The pen is on the paper, and the spring is slightly compressed)
LIFT_Z = 225    # height to lift pen

class FinalProject:
    def __init__(self):
        # --------------- TM-Robot Arm setup ---------------
        if RUN_WITH_TMROBOT:
            self.tm_robot = TM_ROBOT_ARM('tm_robot')
            self.tm_robot.image_mode = "canvas"

        # --------------- Camera setup ---------------
        self.camera = Camera(PATH_CALIB_INFO, 1280, 960)
        self.camera.load_param("EyeToHand")

        # --------------- Canvas Localization setup ---------------
        self.canvas_localization = CanvasLocalization(ARUCO_MARKER_SIZE)
        self.camera_matrix = self.camera.intrinsic.mat_cam
        self.dist_coeffs = self.camera.intrinsic.dist_coeff
        self.canvas_img_path = PATH_TEMP_FILE + "/canvas.jpg"
        self.target_corners = np.array([[0, 0], [1280, 0], [1280, 960], [0, 960]]) # Default target corner (full canvas image size)

        # --------------- Path Generation setup ---------------
        self.simplified_img_path = PATH_TEMP_FILE + "/simplified_strokes.png"
        self.path_gen = EdgeGroup(use_canny_edge=False, closing=False, show_term=True, circle=False, radius=2, min_path=10)
        self.output_csv_path = PATH_TEMP_FILE + "/path.csv"

    def get_canvas(self):
        if RUN_WITH_TMROBOT:
            # Move arm to initial pose and take photos
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            self.tm_robot.move_robot_arm(INITIAL_POS)
            print(">>>>> Press any key to capture image ~ ~ ~")
            cv2.waitKey(0) # wait for arm to reach target, and then press ESC key to capture image
            self.tm_robot.capture_image()
            cv2.waitKey(1000) # wait 1 second
            while(not self.tm_robot.get_image):
                print("waiting for image")
                rclpy.spin_once(self.tm_robot)
            cv2.destroyAllWindows()
        # Get canvas location
        corners = self.canvas_localization.get_canvas_location(self.canvas_img_path, self.camera_matrix, self.dist_coeffs)
        self.target_corners = np.round(corners).astype(int)

    def record_audio(self):
        subprocess.run("cd ~/workspace2/team15_ws/src/final_project/final_project/; source ~/anaconda3/bin/activate ; conda activate team15; python3 speech_to_text.py", shell=True)
        return

    def generate_image(self):
        subprocess.run("cd ~/workspace2/team15_ws/src/final_project/final_project/; source ~/anaconda3/bin/activate ; conda activate team15; python3 text_to_image.py", shell=True)
        return

    def generate_path(self):
        self.path_gen.generate_path(self.simplified_img_path, self.output_csv_path, self.target_corners)

    def generate_script(self):
        # Read CSV file (path)
        scripts = []
        pen_lifted = True
        last_pos = []
        with open(self.output_csv_path, newline='') as csvfile:
            rows = csv.reader(csvfile)
            for points in rows:
                if int(points[0]) == -1 and int(points[1]) == -1: # lift pen
                    # print("lift_pen")
                    scripts.append([last_pos[0], last_pos[1], LIFT_Z, -180, 0, 135])
                    pen_lifted = True
                else:
                    xyz = self.camera.compute_XYZ_eye_to_hand(int(points[0]), int(points[1]))
                    if pen_lifted:
                        scripts.append([xyz[0][0], xyz[1][0], LIFT_Z, -180, 0, 135])
                        pen_lifted = False
                    pos = [xyz[0][0], xyz[1][0], DRAW_Z, -180, 0, 135]
                    scripts.append(pos)
                    last_pos = copy.deepcopy(pos)

        # print(scripts)
        with open(self.script_path, "w", newline="") as csvfile:
            spamwriter = csv.writer(csvfile, delimiter=",", quotechar="\n")
            for pos in scripts:
                spamwriter.writerow(pos)

    def draw(self):
        if RUN_WITH_TMROBOT:
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            pen_lifted = True
            last_pos = []
            with open(self.output_csv_path, newline='') as csvfile:
                rows = csv.reader(csvfile)
                for points in rows:
                    if int(points[0]) == -1 and int(points[1]) == -1: # lift pen
                        # print("lift_pen")
                        self.tm_robot.move_robot_arm([last_pos[0], last_pos[1], LIFT_Z, -180, 0, 135])
                        cv2.waitKey(500) # wait delay
                        pen_lifted = True
                    else:
                        xyz = self.camera.compute_XYZ_eye_to_hand(int(points[0]), int(points[1]))
                        if pen_lifted:
                            self.tm_robot.move_robot_arm([xyz[0][0], xyz[1][0], LIFT_Z, -180, 0, 135])
                            cv2.waitKey(500) # wait delay
                            pen_lifted = False
                        pos = [xyz[0][0], xyz[1][0], DRAW_Z, -180, 0, 135]
                        self.tm_robot.move_robot_arm(pos)
                        cv2.waitKey(50) # wait delay
                        last_pos = copy.deepcopy(pos)
            self.tm_robot.move_robot_arm(INITIAL_POS) # go BACK to initial position
            cv2.destroyAllWindows()

    def main(self):
        print("Final Project START")
        # create save directory if not exists
        os.makedirs(PATH_TEMP_FILE, exist_ok=True)

        # >>>>>>>>>> Initial TM-Robot Arm setup <<<<<<<<<<#
        if RUN_WITH_TMROBOT:
            cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
            self.tm_robot.set_io(0.0) # 0.0: open gripper
            self.tm_robot.move_robot_arm(INITIAL_POS) # go to initial position
            print(">>>>> Press any key to close gripper ~ ~ ~") 
            cv2.waitKey(0) # wait for arm to reach target, and then press esc key to close gripper, grap 3D printed pen holder module
            self.tm_robot.set_io(1.0) # 1.0: close gripper
            cv2.waitKey(1000) # delay
            cv2.destroyAllWindows()            
        
        # >>>>>>>>>> Start running <<<<<<<<<<
        while True:
            print("\nPlease enter following commands:")
            command = input("auto(1) / get_canvas(2) / record_audio(3) / generate_image(4) / generate_path(5) / draw(6) / exit(q): ")
            if(command == "auto" or command == "1"):
                print("\n~ Automate the completion of all processes...... ~")
                self.get_canvas()
                self.record_audio()
                self.generate_image()
                self.generate_path()
                self.draw()
                print("\n~ Auto mode Finished ~")
                continue
            elif(command == "get_canvas" or command == "2"):
                print("\n~ Get Canvas ~")
                self.get_canvas()
                print("\n~ Get Canvas Finished ~")
                continue
            elif(command == "record_audio" or command == "3"):
                print("\n~ Record Audio ~")
                self.record_audio()
                print("\n~ Record Audio Finished ~")
                continue
            elif(command == "generate_image" or command == "4"):
                print("\n~ Generate Image ~")
                self.generate_image()
                print("\n~ Generate Image Finished ~")
                continue
            elif(command == "generate_path" or command == "5"):
                print("\n~ Generate Path ~")
                self.generate_path()
                print("\n~ Generate Path Finished ~")
                continue
            elif(command == "draw" or command == "6"):
                print("\n~ Draw ~")
                self.draw()
                print("\n~ Draw Finished ~")
                continue
            elif(command == "exit" or command == "q"):
                break
            else:
                sys.stderr.write("[ERROR]: Invalid command, please enter again !\n")

        print("Final Project FINISHED !!!!!")
        cv2.destroyAllWindows()
        if RUN_WITH_TMROBOT:
            self.tm_robot.destroy_node()

def main(args=None):
    if RUN_WITH_TMROBOT:
        rclpy.init(args=args)
    
    final_project = FinalProject()
    final_project.main()
    
    if RUN_WITH_TMROBOT:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
