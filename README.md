## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `Mangobee_DifferentialDriveWheel_Robot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

**Linux Commands**
------------------
Developing the robot using URDF
----------------------------------
1. 

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


   
   
