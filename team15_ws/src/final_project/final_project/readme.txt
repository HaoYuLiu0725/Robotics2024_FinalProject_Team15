#### Please place team15_ws folder in ~/workspace2
# ---------- Process For Final Project ------------------------------------------------
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

# Step 3: Run final_project
# Terminal C
cd ~/workspace2/team15_ws/src/final_project/final_project
source ~/workspace2/team15_ws/install/setup.bash
ros2 run final_project final_project_main

# Final Project Process: