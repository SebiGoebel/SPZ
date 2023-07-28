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


## wichtige Links

open3D Doku:
http://www.open3d.org/docs/release/getting_started.html

PCL Doku:
https://pcl.readthedocs.io/projects/tutorials/en/master/walkthrough.html#visualization

STL2PCD:
https://github.com/ktgiahieu/STL2PCL/blob/master/STL2PCD.cpp





stl2pcd:

zuerst stl zu ply:
https://imagetostl.com/convert/file/stl/to/ply#google_vignette

dann mit einem binary file:

```bash
pcl_mesh2pcd
```

-> convert a CAD model to a PCD (Point Cloud Data) file, using ray tracing operations.

**Syntax is: pcl_mesh2pcd input.{ply,obj} output.pcd <options>**, where options are:

  -level X = tessellated sphere level (default: 2)

  -resolution X = the sphere resolution in angle increments (default: 100 deg)

  -leaf_size X = the XYZ leaf size for the VoxelGrid – for data reduction (default: 0.010000 m)
