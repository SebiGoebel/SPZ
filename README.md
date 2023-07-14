# SPZ

## Anzeigen einer pointcloud in rviz

Terminal 1:
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

Terminal 2:
rivz

## Abspeichern einer pointcloud in ein .pcd-File

Terminal 1:
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

Terminal 2:
rosrun beginner_tutorials listener



TODO:
- installation guid
- cleaning up package/code

## wichtige Links

open3D Doku:
http://www.open3d.org/docs/release/getting_started.html

PCL Doku:
https://pcl.readthedocs.io/projects/tutorials/en/master/walkthrough.html#visualization

STL2PCD:
https://github.com/ktgiahieu/STL2PCL/blob/master/STL2PCD.cpp
