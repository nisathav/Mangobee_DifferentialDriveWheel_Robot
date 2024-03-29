**Linux Commands**
------------------
Developing the robot using URDF
----------------------------------
1. copy the SSH link of the repository I am going to clone into my workspace
2. create new folder called mangobee_robot and inside that create another folder called src.
3. go to the directory created.
4. git clone SSH link
5. build the package
6. When opening the robot in rviz2 some of the models may not be showing up like the wheels here. To solve this issue `ros2 run joint_state_publisher_gui joint_state_publisher_gui`

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

![lidar_gazebo](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/d0c1c41d-7809-4dbb-a231-8afa06a3cd39)
![lidar_rviz](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/e166a2f4-108d-432f-92b4-07f826afbc8e)
![lidar_gazebo1](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/75c86567-97d9-4a89-89ae-083597369a1e)
![lidar_rviz1](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/776b5ac7-e3e6-49ee-b34a-7b0735dab40f)

1. adding real `ydlidar` with the pc
2. `git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git`
3. `cd ydlidar_ros2_driver` then `colcon build --symlink-install` you will receive error
4. `sudo apt install cmake pkg-config`
5. `sudo apt-get install swig`
6. `sudo apt-get install python3-pip`
7. go to this link `https://github.com/YDLIDAR/YDLidar-SDK.git` and fork the repo to your github and then clone the repo to your local host
8. `mkdir YDLidar-SDK/build` `cd YDLidar-SDK/build` `cmake ..` `make` `sudo make install`
9. `cd YDLidar-SDK` `pip install .`
10. plug in the lidar and use `sudo chmod 777 /dev/ttyUSB0`
11. `cd ~/YDLidar-SDK/build` `./tri_test`
12. after this changes being done. try to the changes below
13. under path `~/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp` change from `node->declare_parameter("port");` to `node->declare_parameter<std::string>("port");`
14. you have few changes to be done as like above use the following data type for different data types. `<std::string>
<std::int16_t>
<std::float_t>
<bool>` save and exit
15. now build the package `colcon build --symlink-install`
16. when its done. few changes to be done `~/ydlidar_ros2_driver/params/ydlidar.yaml` and `~/ydlidar_ros2_driver/launch/ydlidar_launch_view.py` according to `https://qiita.com/porizou1/items/57edeeda15bbec76a462` and `https://qiita.com/Yuya-Shimizu/items/c516b076ecc15864c0c5`
17. then build the package, source the package and run the launch file `ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py`
18. In rviz2 select the `laserscan->reliabilitypolicy->besteffort`

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

![camera_gazebo](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/623f5d90-f21b-43ee-ab19-0374974fe0d8)
![camera_rviz](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/afa029d7-8a7d-4ac8-a314-2c9838444cd6)

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

![Screenshot from 2023-12-23 17-45-16](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/e5e1c6d1-2aa3-4bbd-ae8a-c2f9cc90b539)
![Screenshot from 2023-12-23 17-44-54](https://github.com/nisathav/Mangobee_DifferentialDriveWheel_Robot/assets/129756080/2dbe31f3-141e-44c3-a485-383145913646)


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

SLAM (Simultaneous Localisation and Mapping) with 2D YDLidar
------------------------------------------------------------
1. insert the following in the robot_core.xacro
   `<!-- BASE_FOOTPRINT LINK -->
    <joint name="base_footprint_joint" type="fixed">
        <parent link="base_link"/>
        <child link="base_footprint"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    <link name="base_footprint">
    </link>`
3. install the slam toolbox `sudo apt install ros-humble-slam-toolbox`
4. run the slam toolbox on online ans asynchronous
5. copy the param file `cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_sync.yaml mangobee_robot/src/Mangobee_DifferentialDriveWheel_Robot/config/`
6. rebuild the package and source it
7. open the gazebo `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world` 
8. also open the rviz2 `rviz2 -d src/Mangobee_DifferentialDriveWheel_Robot/config/view_bot.rviz`
9. open new terminal and run `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/Mangobee_DifferentialDriveWheel_Robot/config/mapper_params_online_async.yaml use_sim_time:=true`
10. add map to the rviz and change the fixed frame to `map` so that map can stay fixed, the robot can move freely.
11. now map out the entire platform
12. `ros2 service list` will give the services including the slam toolbox offer
13. you can open slam toolbox terminal in rviz2 aso to save the map and for other stuff related to slam
14. panel -> add new panel -> slamtoolbox
15. you can see bunch of options. save map button used to save the map. serialize map used to save it and use it with slam toolbox again.
16. rerun the `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/Mangobee_DifferentialDriveWheel_Robot/config/mapper_params_online_async.yaml use_sim_time:=true`
17. do the change in mapper_params_online_async.yaml. change mode `from mapper to localization`, change map_file_name from `test_steve to /home/mangobee_robot/my_map_serial`, also un comment the `map_start_at_dock`
18. rerun `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/Mangobee_DifferentialDriveWheel_Robot/config/mapper_params_online_async.yaml use_sim_time:=true` to open the saved map and then continue on it.
19. we have many localization method. one of them is `AMCL: Adaptive Monte Carlo Localisation`
20. Install Nav2 `sudo apt install ros-humble-navigation2`
21. create a map node and publish it. `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=my_map_save.yaml -p use_sim_time:=true`
22. Open new tap and run `ros2 run nav2_util lifecycle_bringup map_server`
23. rerun rviz `rviz2 -d src/Mangobee_DifferentialDriveWheel_Robot/config/view_bot.rviz`and gazebo
24. once the rviz2 opened you may face different problem. map -> durability ability -> Transient Local
25. new tap, `ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true` and in another tap run the following `ros2 run nav2_util lifecycle_bringup amcl`
26. also set the place of the robot in the map using `2D pose estimate` and then the problem should have been sorted out

Navigation
----------
1. install the following `sudo apt install ros-humble-twist-mux`
2. open a file in the config folder named as `twist_mux.yaml`
3. run the twist_mux.yaml file `ros2 run twist_mux twist_mux --ros-args --params-file ./src/Mangobee_DifferentialDriveWheel_Robot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped`
4. rebuild package and launch gazebo `ros2 launch Mangobee_DifferentialDriveWheel_Robot launch_sim.launch.py world:=./src/Mangobee_DifferentialDriveWheel_Robot/worlds/obstacles.world` also open rviz2
5. source the file and run the slam toolbox `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/Mangobee_DifferentialDriveWheel_Robot/config/mapper_params_online_async.yaml use_sim_time:=true`
6. `ros2 launch nav2_bringup navigation_launch.py use_sime_time:=true`
7. add another map in the rviz, `topic -> /global_costmap/costmap` then `color scheme -> cost_map`
8. select `2D Goal Pose` it will automatically move to the point

Navigation using ACML:
1. run twist_mux, gazebo and rviz2
2. run `ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true`
3. set the `2D Pose Estimate` and select `map -> Topic -> Durability Policy -> Transient Local`
4. for navigation `ros2 launch nav2_bringup navigation_launch.py use_sime_time:=true map_subscribe_transient_local:=true`
5. add another map in the rviz, `topic -> /global_costmap/costmap` then `color scheme -> cost_map`
6. when using navigation through this, the base map wont be updated with any new obstacle but the cost map will update in real time
7. select `2D Goal Pose` it will automatically move to the point

Making the launch file and param file available within our own package:
1. `cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml src/Mangobee_DifferentialDriveWheel_Robot/config/`
2. `cp /opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py src/Mangobee_DifferentialDriveWheel_Robot/launch/`
3. `cp /opt/ros/humble/share/nav2_bringup/launch/localization_launch.py src/Mangobee_DifferentialDriveWheel_Robot/launch/`
4. `cp /opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py src/Mangobee_DifferentialDriveWheel_Robot/launch/`
5. Do the following changes in navigation_launch.py `bringup_dir = get_package_share_directory('Mangobee_DifferentialDriveWheel_Robot') #refernece to the package changed from nav2_bringup` and `default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'), #param changed to config`
6. do the same thing to localiation_launch.py too
7. now we can use the following code `ros2 launch nav2_bringup localization_launch.py map:=./my_map_save.yaml use_sim_time:=true` as `ros2 launch Mangobee_DifferentialDriveWheel_Robot localization_launch.py map:=./my_map_save.yaml use_sim_time:=true`
8. do the changes in online_async_launch.py `default_value=os.path.join(get_package_share_directory("Mangobee_DifferentialDriveWheel_Robot"), #changed from slam_toolbox to Mangobee_DifferentialDriveWheel_Robot`
9. try use `localization_launch.py` instead of `online_async_launch.py` copied it to the Mangobee_DifferentialDriveWheel_Robot package
10. add the twist_mux file into the launch file

pushing to git:
1. `directory - mangobee_robot/src/Mangobee_DifferentialDriveWheel_Robot` `git status`status
2. `git add .`
3. `git commit -m "adding the nav stack and the slam toolbox"`
4. `git pull origin main`
5. `git merge`
6. `git push origin main`

    
Object Tracking
---------------
1. install open CV `sudo apt install python3-opencv`
2. edit the twist_mux and insert the topic `tracker`
3. source the workspace and open the robot in the gazebo
4. add a tennis ball in the gazebo using sphere and change the ball size and the collision
5. Open rviz2

Detecting the ball:
1. centre of the frame is (0,0)
2. clone the github `git clone git@github.com:joshnewans/ball_tracker` within the src on the package
3. build the package
4. source the workspace
5. `ros2 run ball_tracker detect_ball --ros-args -p tuning_mode:=true -r image_in:=camera/image_raw`
6. in rviz2, open image display -> topic -> /image_tuning
7. insert specktrum plane after the tennis ball vertically
8. adjust the parameters on the `Tuning` window. parameter value `0,100,39,100,21,30,41,255,0,255,0,20`
9. to get the point measurements of the ball `ro2 topic echo /detected_ball`

3D position estimation:
1. `ros2 run ball_tracker detect_ball_3d`
2. rviz2 -> Marker -> topic -> /ball_3d_marker
3. navigate the ball in gazebo and track the changes in the rviz2

Follow the ball:
1. `ros2 run ball_tracker follow_ball --ros-args -r cmd_vel:=cmd_vel_tracker`
2. launch file `ros2 launch ball_tracker ball_tracker.launch.py --show-args` it will show the available parameters
3. copy the params file from ball_tracker to Mangobee_DifferentialDriveWheel_Robot
4. copy the tuning window paramters
5. `ros2 launch ball_tracker ball_tracker.launch.py params_file:=src/Mangobee_DifferentialDriveWheel_Robot/config/ball_tracker_params_sim.yaml`
6. or use this `ros2 launch ball_tracker ball_tracker.launch.py params_file:=src/Mangobee_DifferentialDriveWheel_Robot/config/ball_tracker_params_sim.yaml enable_3d_tracker:=true`
7. after creating launch file in Mangobee_DifferentialDriveWheel_Robot, `ros2 launch Mangobee_DifferentialDriveWheel_Robot ball_tracker.launch.py sim_mode:=true`
