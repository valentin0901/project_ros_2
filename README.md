test

pour mettre sur gazebo il faut juste taper res 
puis mettre le fichier dans 
aruco_box# tree
.
├── aruco_marker.blend
├── materials
│   └── textures
│       └── marker_tiles_square.png
├── model.config
├── model.dae
└── model.sdf

et copier dans le fichier /root/gazebo_models ou /home/ubunu/gazebo_models


Pour opencv aruco il faut modifier aruco_tracker.yaml

    cam_base_topic: camera/image
    output_frame: ''

    marker_dict: ARUCO_ORIGINAL


Pour lancer


colcon build

source install/setup.bash

ros2 launch bme_gazebo_sensors spawn_robot.launch.py 


ros2 launch bme_gazebo_sensors spawn_robot.launch.py 

ros2 launch aruco_opencv aruco_tracker.launch.xml


ros2 launch assignment_1 assignment_1.launch.py