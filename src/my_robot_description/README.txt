colcon build
source install/setup.bash


Pour tester mon urdf :
ros2 launch urdf_tutorial display.launch.py model:=/root/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro
