https://robostack.github.io/GettingStarted.html

pixi shell -e kilted
source install/local_setup.zsh 

pixi search -c https://prefix.dev/robostack-kilted "*plugin*"


inspired by https://github.com/odriverobotics/ros_odrive/tree/main
https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html#

# Generate VSCode completion
https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cp build/compile_commands.json .vscode

# Robot description generation
https://cad.onshape.com/documents/238e8faca9c7214bccace665/w/ac168b104948c1f839976186/e/24599102151386cccc6b72ba?resourceType=resourceuserowner&nodeId=663350b99d750015af97830c

onshape -> ros urdf https://onshape-to-robot.readthedocs.io/en/latest/design.html#workflow-overview


# Used commands
rqt -s rqt_reconfigure
ros2 launch spider_ros_control spider.launch.py

# Mujoco integration
https://github.com/isri-aist/MujocoRosUtils/tree/main
https://github.com/tenfoldpaper/mujoco_ros_pkgs/tree/wip_ros_control_humble
https://github.com/moveit/mujoco_ros2_control/tree/main