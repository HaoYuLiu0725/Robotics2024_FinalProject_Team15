# Robotics2024_FinalProject_Team15

This document provides setup instructions and usage guidelines for running the final project, HW4, and the eye-to-hand calibration processes with the TM robot arm.  

> **Important:** Please place the `team15_ws` folder in `~/workspace2`.

---

## 1. Final Project

### 1.1 Prerequisites
- The TM robot arm’s IP address must be known.  
  - TM robot arm #2 (near door): `192.168.0.69`  
  - TM robot arm #1: `192.168.0.102`
- ROS 2 workspace named `colcon_ws` (or equivalent) must be sourced properly.
- The project workspace `team15_ws` should be located at `~/workspace2/team15_ws`.
- Print out **canvas.pdf** on A4 paper and clip it onto an A4 document clipboard.
  
### 1.2 Steps

1. **Run `tm_driver`**  
   **Terminal A**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_driver tm_driver 192.168.0.69
   ```
   > Change the IP address to `192.168.0.102` if you are using robot arm #1.

2. **Run `image_talker`**  
   **Terminal B**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_get_status image_talker
   ```

3. **Run `final_project`**  
   **Terminal C**  
   ```bash
   cd ~/workspace2/team15_ws/src/final_project/final_project
   source ~/workspace2/team15_ws/install/setup.bash
   ros2 run final_project final_project_main
   ```

### 1.3 Final Project Process

1. Place the canvas with the clipboard on the table, then start the program.  
2. The robot arm will move to a preset position. Once it stops, place the 3D-printed pen-holding module onto the gripper, then **press any key** to close the gripper and secure the pen-holding module.  
3. You will be prompted to enter one of the following commands:  
   - auto(1) / get_canvas(2) / record_audio(3) / generate_image(4) / generate_path(5) / draw(6) / exit(q):  
4. Enter "auto" or "1" to start the full automated process, which includes get_canvas, record_audio, generate_image, generate_path, and draw. Each process will be explained below.  
5. Enter "get_canvas" or "2" to start the get canvas process:  
   - **Press any key** on the keyboard to capture an image.  
   - The program will calculate the four corners of the canvas and display the image on-screen.  
   - **Press any key** again to close the image display.  
6. Enter "record_audio" or "3" to start the record audio process:  
   - After you see "Recording..." on the terminal screen, you have 5 seconds to issue a command.  
   - For example, say "我想要畫一朵花" or "I want to draw a fish."  
   - The program will record the audio, automatically translate it into English, display the translation on-screen, and save the text.  
7. Enter "generate_image" or "4" to start the generate image process:  
   - The program will generate and save an image based on the voice command from the previous step.  
8. Enter "generate_path" or "5" to start the generate path process:  
   - The program will calculate the best path using the image generated in the previous step and the four corners of the canvas from the get_canvas step, then save it as a CSV file.  
9. Enter "draw" or "6" to start the drawing process:  
   - The robot arm will use the path generated in the previous step to automatically draw the image.  
---

## 2. HW4

### 2.1 Prerequisites
- Same environment and folders as described above.
- Randomly place **3 blocks** on the table before starting the program.

### 2.2 Steps

1. **Run `tm_driver`**  
   **Terminal A**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_driver tm_driver 192.168.0.69
   ```
   > Again, adjust IP address as necessary.

2. **Run `image_talker`**  
   **Terminal B**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_get_status image_talker
   ```

3. **Run `hw4`**  
   **Terminal C**  
   ```bash
   cd ~/workspace2/team15_ws/src/send_script/send_script
   source ~/workspace2/team15_ws/install/setup.bash
   ros2 run send_script hw4
   ```

### 2.3 HW4 Process:
1. Randomly place **3 blocks** on the table, then start the program.  
2. The robot arm moves to a preset position. Once the robot arm stops moving, **press any key** on the keyboard to capture an image.  
3. The program computes the centroid and principal angle of the 3 blocks and displays the result image on-screen.  
4. Press **any key** to close the image display.  
5. You will be prompted:
   - **Press 'C'** to continue picking and stacking blocks.  
   - **Press 'Q'** to abort.  
6. If you press **'C'**, the robot arm automatically picks and stacks the 3 blocks.

---

## 3. Eye-To-Hand Calibration

### 3.1 Prerequisites
- Same environment setup as above.
- TM robot arm IP and driver running.

### 3.2 Steps

1. **Run `tm_driver`**  
   **Terminal A**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_driver tm_driver 192.168.0.69
   ```

2. **Run `image_talker`**  
   **Terminal B**  
   ```bash
   cd ~/colcon_ws
   source install/setup.bash
   ros2 run tm_get_status image_talker
   ```

3. **Run `eye_to_hand_calibration`**  
   **Terminal C**  
   ```bash
   cd ~/workspace2/team15_ws/src/send_script/send_script
   source ~/workspace2/team15_ws/install/setup.bash
   ros2 run send_script eye_to_hand_calibration
   ```

### 3.3 Calibration Process

1. **Intrinsic Calibration**  
   - Ensure `get_calibration_image` and `run_intrinsic_calibration` are set to **True** in your code.  
   - Place the chessboard in various positions/orientations, then **press 'X'** to capture each image.  
   - After collecting enough images (e.g., 20), press **'Q'** to exit the intrinsic calibration stage.

2. **Extrinsic Calibration**  
   - Set `get_block_image` and `run_extrinsic_calibration` to **True**.  
   - The robot arm moves to a preset position. Place a block in the gripper, then **press any key** to close the gripper.  
   - The robot arm automatically places and picks the block at predefined 5×5 grid points.  
   - Between each placement and pick-up, the robot arm returns to the preset position to capture an image.  
   - Press **any key** each time to proceed to the next movement.

In doing so, you collect the data required for both intrinsic and extrinsic calibration, allowing the system to accurately interpret the visual data and align it with the robot arm’s coordinate space.

---

### Notes
- Always source the correct workspace setup file (e.g., `source ~/colcon_ws/install/setup.bash`) before running ROS 2 commands.  
- If you use TM robot arm #1, replace `192.168.0.69` with `192.168.0.102` in all relevant commands.

---
