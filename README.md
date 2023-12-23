**Linux Commands**
------------------
Developing the robot using URDF
----------------------------------
1. copy the SSH link of the repository I am going to clone into my workspace
2. create new folder called mangobee_robot and inside that create another folder called src.
3. go to the directory created.
4. git clone SSH link
5. build the package

![Screenshot from 2023-12-23 14-24-08](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/2692cc8f-3149-4fc1-a01c-1aaf92b43e77)

![Screenshot from 2023-12-23 14-19-08](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/ff9f0aab-4731-4922-8344-35a004ddc234)

Simulating the robot in Gazebo
----------------------------------
1. The below code will publish the full URDF to /robot_description
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot rsp.launch.py use_sim_time:=true`
2. launch gazebo with ROS compatability
   `ros2 launch gazebo_ros gazebo.launch.py`
3. uploading the robot in to gazebo
   `ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity robot_name`
4. create a launch file with the name of launch_sim.py in the launch directory and copy the code
  `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.py`
6. add colors using gazebo tag and the friction of the caster wheel
7. create new file under description, called gazebo_control.xacro
8. iclude the above file in robot.urdf.xacro
9. build the package
   `colcon build --symlink-install`
10. source the package
11. launch the launch_sim.launch.py
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py`
12. run the tele operation
    `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
13. create a world with obstacles with construction cones and barrels, save this in the following directory, MangoBee_Robot/src/Mangobee_DifferentialDriveWheel_Robot/world
14. launch the program inside the world,
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot    launch_sim.launch.py    world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
15. check the velcity if the robot do not move
    `ros2 topic echo /cmd_vel`
16. check the status of the changes made to document    
     ~/MangoBee_Robot/src/Mangobee_DifferentialDriveWheel_Robot
    `git status`
17. `git add .`
18. `git commit -m "resolving the problems"`
19. `git push`

Adding LiDAR to the robot
-------------------------
The following types of LiDAR vailable,
   - point Lidar (1D Lidar) : Depth or distance
   - Laser Scanners (2D Lidar) : mapping out the floor
   - 3D Lidar

Message type for 2D Lidar is sensor_msgs/LaserScan
Message type for 3D Lidar is sensor_msgs/PointCloud2

1. update the description file with lidar.xacro
   `colcon build --symlink-install`
   `source install/setup.bash`
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot rsp.launch.py`
2. update the lidar.xacro with ros plugins with sensor details
3. build the package and source the code, run the launch file
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot    launch_sim.launch.py    world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
4. try to move the robot to see the working concept of lidar
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
5. visulaize the output using rviz2. open rviz2, add LaserScan, select topic /scan and then increase the size to 0.04
6. left to do, integrate the lidar into rel robot

Adding Camera to the system
----------------------------
1. create camera.xacro file in description directory.
   `colcon build --symlink-install`
   `source install/setup.bash`
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
2. turn off the laser visualize and re-launch the above launch file
3. launch rviz2 and add Image, select topic as /camera/image_raw
4. add in camera lo, to view the poitioned image in the world
5. install the following package to vie image and compression.
   `sudo apt install ros-humble-image-transport-plugins`
6. source the package above
   `source /opt/ros/humble/setup.bash`
7. re run the gazebo simulator
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
8. install the following package too
   `sudo apt install ros-humble-rqt-image-view`
9. launch the following and select the type of image description needed in the rqt image view
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
   `ros2 run rqt_image_view rqt_image_view`

Adding depth camera to the robot
----------------------------------
we have three types of technology used in depth camera. These camera return the depth value to each pixel in the photo.
   1. Structured light: This creates a structure on the wall. based on the distprtions the camera detects the depth.
   2. Time of flight: This uses the time taken for the pulses to return to its origin position
   3. Stereo: It uses two camera to detect the depth

Integrate the depth camera with ROS,
1. Copy the camera.xacro file and rename it to depth_camera.xacro
2. include the above the xacro file into robot.urdf.xacro
3. change the sensor type to depth in the depth_camera.xacro, set the image format to BBGBRB
4. build the package and run the launch_sim.launch.py file
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
5. open rviz and add robotmodel, image, poincloud2. Also, set the following /robotdescription, /camera/depth/image/raw, /camera/points
6. Toggle around RVIZ to analysis the depth camera

ros2_control Concept and Simulation
-----------------------------------
- Always remeber to check the hardware components bing used in the robot to identify the ros2_control.
- Hardware interface has two types of interfaces
     1. Command Interfaces - Read/Write
     2. State Interfaces - Read Only
   to check the interface use the following codes `ros2 control list_hardware_interfaces`
- Control manager and resource manager uses the URDF file to identify the hardware interfaces
- To setup the controllers YAML file is being used. multiple controllers can be used.
- We have two ways to run the control manager,
     1. own node containing the controller inside (use controller_manager::ControllerManager class)
     2. Use the provided node controller_manager/ros2_control_node
- Whatever the method is we need to provide the hardware interface through URDF and the controller information through YAML


Control Manager
   - Interacting with Control Manager
        1. via ROS services
        2. via ros2 control CLI tool
        3. via specialised nodes/scripts
           
1. upgrade gazebo simulation to use ros2_control
   `sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control`
2. copy gazebo_control.xacro and rename it as ros2_control.xacro
3. comment out the gazebo_control.xacro in the robot.urdf.xacro and include the ros2_control.xacro
4. Plugin is a small code installed and registered with ros that tells how to talk with gazebo simulation
5. when not using gazebo, we will need to run ros2 run controller_manager ros2_control_node
6. create a new file in the config directory named as my_controllers.yaml
7. run the launch file,
   `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world`
8. May notice irrelevant logs while the above command is running.
9. `ros2 run controller_manager ros2_control_node`,`ros2 run controller_manager spawner diff_cont`
