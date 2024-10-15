# drive system stack overview
This repo contains links to all the packages which are essential and necessary for the [autonomous software stack](https://github.com/Adipks/autonomous_navigation/tree/main).
This all these packages essentialy are what you would have in your ros_workspace.The autonomous navigation package essentialy uses [move_base](https://github.com/Adipks/rover_repo/blob/main/README_move_base.md) a software architecture for autonomous navigation and planning.
The implementation of this architecture to perform autonomous navigation in your rover is described in the [autonomous_navigation_repo](https://github.com/Adipks/autonomous_navigation/tree/main).

### Packages used

├── [slam_gmapping](https://github.com/Adipks/rover_repo/blob/main/README_slam_gmapping.md)

├── [depthimage_to_lasersan](https://github.com/Adipks/rover_repo/blob/main/README_depthimage.md)

├── [frontier_navigation](https://github.com/Adipks/rover_repo/blob/main/README_frontier_navigation.md)

├── [rtabmap_ros](https://github.com/Adipks/rover_repo/blob/main/README_rtabmap_ros.md)

├── [robot_pose_ekf](https://github.com/Adipks/rover_repo/blob/main/REAME_robot_pose_ekf.md)

├── [navigation](https://github.com/Adipks/rover_repo/blob/main/README_navigation.md)

├── [Zed2i_ws](https://github.com/Adipks/rover_repo/blob/main/zed.md)

## teb-local-planner
the [teb local planner](https://wiki.ros.org/teb_local_planner) is used by our autonomous navigation stack to serve the purpose of a local planner ,global planning is done by navfn_ros which is already included as a part of the [navigation package](https://github.com/ros-planning/navigation/tree/9ad644198e132d0e950579a3bc72c29da46e60b0).The [teb_local_planner_tutorials](https://github.com/rst-tu-dortmund/teb_local_planner_tutorials) helps you get started on how to use the teb local planner.
