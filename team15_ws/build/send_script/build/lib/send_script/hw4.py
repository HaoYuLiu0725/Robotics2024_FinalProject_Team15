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
import math
import copy
from send_script.camera import Camera

PATH_CALIB_INFO = './calibration_info'

#             "X, Y, Z, Rx, Ry, Rz"
INITIAL_POS = "230.00, 230, 730, -180.00, 0.0, 135.00"

PICK_Z = 102            # height to pick the block
PICK_READY_OFFSET = 30  # offset Z for a ready pos, aim to perform a vertical movement to pick up the block 
LIFT_HEIGHT = 200       # lift block to a safty height
DROP_Z = 102            # height to drop the block
STACK_OFFSET = 30       # offset for each step of stack block

def plan_stack_and_pick(blocks, safety_distance=50):
    """
    Calculate the Fermat Point for a set of 3 block centroids, minimizing the total distance,
    adjust the stack point if it's too close to any block, and determine pick order.

    Parameters:
        blocks (list): A list of 3 positions in the format [x, y, z, angle].
        safety_distance (float): Minimum safe distance from any block to the stack point.

    Returns:
        tuple: Adjusted stack point as (x, y), and the pick order as a list of indices.
    """
    def angle(a, b, c):
        """Calculate the angle at vertex b (in degrees)."""
        ab = math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)  # Distance from a to b
        bc = math.sqrt((c[0] - b[0])**2 + (c[1] - b[1])**2)  # Distance from b to c
        ac = math.sqrt((c[0] - a[0])**2 + (c[1] - a[1])**2)  # Distance from a to c
        return math.degrees(math.acos((ab**2 + bc**2 - ac**2) / (2 * ab * bc)))

    def equilateral_point(p1, p2):
        """
        Find the third vertex of an equilateral triangle constructed outward
        from the line segment defined by two points p1 and p2.
        """
        dx, dy = p2[0] - p1[0], p2[1] - p1[1]  # Vector from p1 to p2
        return (
            p1[0] + 0.5 * (dx - math.sqrt(3) * dy),  # x-coordinate of the third vertex
            p1[1] + 0.5 * (dy + math.sqrt(3) * dx)   # y-coordinate of the third vertex
        )

    def is_too_close(point, blocks, safety_distance):
        """Check if the point is too close to any block."""
        for block in blocks:
            distance = math.sqrt((point[0] - block[0])**2 + (point[1] - block[1])**2)
            if distance < safety_distance:
                return True
        return False

    # Extract the 2D (x, y) positions from the input list, ignoring z and angle
    a, b, c = [block[:2] for block in blocks]

    # Calculate the angles at each vertex of the triangle
    angle_a = angle(b, a, c)
    angle_b = angle(a, b, c)
    angle_c = angle(a, c, b)

    # Determine the initial stack point
    if angle_a > 120:
        stack_point = a  # Fermat Point is at vertex A
    elif angle_b > 120:
        stack_point = b  # Fermat Point is at vertex B
    elif angle_c > 120:
        stack_point = c  # Fermat Point is at vertex C
    else:
        # Construct equilateral triangles outward and find intersection
        p_ab = equilateral_point(a, b)  # Third vertex for side AB
        p_bc = equilateral_point(b, c)  # Third vertex for side BC
        p_ca = equilateral_point(c, a)  # Third vertex for side CA

        # Approximate the Fermat Point as the centroid of these candidate points
        stack_point = (
            (p_ab[0] + p_bc[0] + p_ca[0]) / 3,
            (p_ab[1] + p_bc[1] + p_ca[1]) / 3,
        )

    # Adjust stack point if too close to any block
    if is_too_close(stack_point, [a, b, c], safety_distance):
        stack_point = (stack_point[0] + safety_distance, stack_point[1])

    # Compute distances from each block to the adjusted stack point
    distances = [
        (i, math.sqrt((block[0] - stack_point[0])**2 + (block[1] - stack_point[1])**2))
        for i, block in enumerate([a, b, c])
    ]

    # Sort blocks by distance to the stack point
    pick_order = [i for i, _ in sorted(distances, key=lambda x: x[1])]

    return stack_point, pick_order

def main(args=None):
    print("HW4 START")
    camera = Camera(PATH_CALIB_INFO, 1280, 960)
    camera.load_param("EyeToHand")

    if RUN_WITH_TMROBOT:
        rclpy.init(args=args)
        tm_robot = TM_ROBOT_ARM('tm_robot')
        tm_robot.image_mode = "hw4_blocks"

        ##### move arm to initial pose and take photos
        cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
        script = "PTP(\"CPP\","+INITIAL_POS+",100,200,0,false)"
        tm_robot.send_script(script)
        print(f"send script: {INITIAL_POS}")
        print(">>>>> Press any key to capture image ~ ~ ~")
        cv2.waitKey(0) # wait for arm to reach target, and then press ESC key to capture image
        tm_robot.send_script("Vision_DoJob(job1)")
        print(f"send Vision_DoJob")
        tm_robot.get_image = False
        cv2.waitKey(1000) # wait 1 second
        while(not tm_robot.get_image):
            print("waiting for image")
            rclpy.spin_once(tm_robot)
        cv2.destroyAllWindows()

    ##### read image and find centroids
    img = cv2.imread("hw4_blocks.jpg")
    # centroid_list = camera.get_centroid(img, timeout=0, show_process=True, save_result=True) # timeout=0: pressing ESC to close window
    centroid_list = camera.get_centroid_and_corner(img, timeout=0, show_process=False, save_result=True) # timeout=0: pressing ESC to close window
    # centroid_list = camera.get_centroid(img, timeout=3000) # wait 3 second and then close window

    ##### compute centroids XYZ in eye-in-hand coordinate
    XYZ_list = []
    for i, cent in enumerate(centroid_list):
        print(f"centroid {i}: Cx = {cent[0]:.4f}, Cy = {cent[1]:.4f}, principal_angle = {cent[2]:.4f} degrees")
        xyz = camera.compute_XYZ_eye_to_hand(int(cent[0]), int(cent[1]))
        XYZ = [xyz[0][0], xyz[1][0], PICK_Z, cent[2]]
        # print(XYZ)
        print(f"WORLD X: {XYZ[0]:.4f}, Y: {XYZ[1]:.4f}, Z: {XYZ[2]:.4f}, angle: {XYZ[3]:.4f}")
        XYZ_list.append(XYZ)

    ##### compute stack point and pick order
    stack_point, pick_order = plan_stack_and_pick(XYZ_list, safety_distance = 50)
    print(f"Stack Point: X = {stack_point[0]:.4f}, Y = {stack_point[1]:.4f}")
    print(f"Pick Order: {pick_order}")

    ##### choose to continue or abort
    print("\nPlease choose to continue or abort......")
    print("Press C to CONTINUE picking and stacking blocks")
    print("Press Q to ABORT")
    cv2.namedWindow("wait", cv2.WINDOW_FULLSCREEN)
    while True:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            print('>>>> Continue picking and stacking blocks')
            break
        elif key == ord('q'):
            print('>>>> Abort')
            cv2.destroyAllWindows()
            # rclpy.spin(tm_robot)
            tm_robot.destroy_node()
            rclpy.shutdown()
            return

    ##### robot arm control
    if RUN_WITH_TMROBOT:
        def move_robot_arm(pos: list):
            """
            Move robot arm to the given position.

            Parameters:
                pos (list): A list in the format [x, y, z, angle].
            """
            while (pos[3] > 45):
                pos[3] -= 90
            while (pos[3] < -45):
                pos[3] += 90
            #           "   X,        Y,        Z,       Rx,    Ry,   Rz"
            pos_str = f"{pos[0]}, {pos[1]}, {pos[2]}, -180.00, 0.0, {135 + pos[3]}"
            script = "PTP(\"CPP\","+pos_str+",100,200,0,false)"
            tm_robot.send_script(script)
            print(f"send script: {pos_str}")

        stacked_block = 0
        for idx in pick_order:
            block_pos = XYZ_list[idx]
            block_ready_pos = copy.deepcopy(block_pos)
            block_ready_pos[2] += PICK_READY_OFFSET # offset Z for a ready pos, aim to perform a vertical movement to pick up the block
            block_lift_pos = copy.deepcopy(block_pos)
            block_lift_pos[2] = LIFT_HEIGHT         # lift block to a safty height
            stack_pos = [stack_point[0], stack_point[1], DROP_Z + STACK_OFFSET*stacked_block, 0] # stack point
            stack_ready_pos = copy.deepcopy(stack_pos)
            stack_ready_pos[2] = LIFT_HEIGHT        # ready to stack block from safty height

            tm_robot.set_io(0.0) # 0.0: open gripper
            print(f"Go to block {idx} ...")
            move_robot_arm(block_ready_pos)
            cv2.waitKey(3000) # delay
            move_robot_arm(block_pos)
            cv2.waitKey(1000) # delay
            print(f"Pick block {idx} !!!")
            tm_robot.set_io(1.0) # 1.0: close gripper
            cv2.waitKey(1000) # delay
            move_robot_arm(block_lift_pos)
            cv2.waitKey(1000) # delay
            print("Go to stack point ...")
            move_robot_arm(stack_ready_pos)
            cv2.waitKey(1000) # delay
            move_robot_arm(stack_pos)
            cv2.waitKey(1000) # delay
            tm_robot.set_io(0.0) # 0.0: open gripper
            stacked_block += 1
            print(f"stacked block number = {stacked_block}")
            move_robot_arm(stack_ready_pos)
            cv2.waitKey(1000) # delay

    print("HW4 FINISHED !!!!!")
    cv2.destroyAllWindows()
    # rclpy.spin(tm_robot)
    tm_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
