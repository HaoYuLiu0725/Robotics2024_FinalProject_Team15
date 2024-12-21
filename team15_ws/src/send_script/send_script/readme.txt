##### Please place team15_ws folder in ~/workspace2
# ---------- Process For HW4 ------------------------------------------------
# Step 1: Run tm_driver
# Terminal A
cd ~/colcon_ws
source install/setup.bash
ros2 run tm_driver tm_driver 192.168.0.69
# We use TM robot arm #2 (near door), ip address is 192.168.0.69
# If you use TM robot arm #1, ip address is 192.168.0.102

# Step 2: Run image_talker
# Terminal B
cd ~/colcon_ws
source install/setup.bash
ros2 run tm_get_status image_talker

# Step 3: Run hw4
# Terminal C
cd ~/workspace2/team15_ws/src/send_script/send_script
source ~/workspace2/team15_ws/install/setup.bash
ros2 run send_script hw4

# HW4 Process:
- Randomly place 3 blocks on the table, then start the program.
- The robot arm will move to a preset position. Once the robot arm stops moving, press any key on the keyboard to capture an image.
- The program will then calculate the centroid and principal angle of the 3 blocks and display the result image on the screen.
- After this, press any key to close the image display.
- Once the image is closed, you will be prompted to choose an action:
-   Press 'C' to CONTINUE picking and stacking blocks.
-   Press 'Q' to ABORT.
- If you press 'C', the robot arm will automatically proceed to pick and stack the 3 blocks.

# ---------- Process For Eye-To-Hand Calibration -----------------------------------------------------
# Step 1: Run tm_driver
# Terminal A
cd ~/colcon_ws
source install/setup.bash
ros2 run tm_driver tm_driver 192.168.0.69
# We use TM robot arm #2 (near door), ip address is 192.168.0.69
# If you use TM robot arm #1, ip address is 192.168.0.102

# Step 2: Run image_talker
# Terminal B
cd ~/colcon_ws
source install/setup.bash
ros2 run tm_get_status image_talker

# Step 3: Run eye_to_hand_calibration
# Terminal C
cd ~/workspace2/team15_ws/src/send_script/send_script
source ~/workspace2/team15_ws/install/setup.bash
ros2 run send_script eye_to_hand_calibration

# Eye-To-Hand Calibration Process:
### After starting the program, the robot arm will move to a preset position. Follow the steps below:
### 1. Intrinsic Calibration:
- Ensure that **get_calibration_image** and **run_intrinsic_calibration** are set to **True**.  
- Place the chessboard in various positions and orientations, then press **'X'** to capture an image.  
- Once enough images are captured (e.g., 20 images), press **'Q'** to exit.  
### 2. Extrinsic Calibration:
- Ensure that **get_block_image** and **run_extrinsic_calibration** are set to **True**.  
- After the robot arm moves to a preset position, place a block in the gripper, then press any key to close the gripper.  
- The robot arm will automatically place and pick the block at predefined 5x5 grid points.
- Between each placement and pick-up, the robot arm will return to the preset position to capture an image.  
- To ensure that the placement, picking, and image capture process works correctly, you will need to press any key to proceed to the next movement.  