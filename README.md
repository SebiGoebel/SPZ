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
