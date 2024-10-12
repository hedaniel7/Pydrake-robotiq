# Pydrake-robotiq

### Conversion of robotiq_arg2f_140_model.xacro to robotiq_arg2f_140_model.urdf

#### build and run Ubuntu 16.04 Ros Kinetic docker image. Then convert

- `~/Projects/Pydrake-robotiq140$ docker-compose build`
- `~/Projects/Pydrake-robotiq140$ docker-compose run ros-kinetic`
- `root@223ff2e07fdd:/catkin_ws catkin_make` (I commented out unncessary ROS packages not needed for conversion with CATKIN_IGNORE files in this forked version)
- `root@223ff2e07fdd:/catkin_ws source devel/setup.bash`
- `root@223ff2e07fdd:/catkin_ws/src/robotiq/robotiq_2f_140_gripper_visualization/urdf# xacro robotiq_arg2f_140_model.xacro -o robotiq_arg2f_140_model.urdf`
