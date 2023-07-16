#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "ros/package.h"

// converting stl to point cloud
#include "pcl/io/vtk_io.h"
#include "pcl/io/vtk_lib_io.h"
#include "pcl/point_types.h"

// showing point cloud
#include "pcl/visualization/pcl_visualizer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_stl2pcd");
    ros::NodeHandle n;
    std::string topic = n.resolveName("point_cloud");
    uint32_t queue_size = 1;

    // Loading stl file
    pcl::PolygonMesh mesh;
    std::string file_path_stl = ros::package::getPath("beginner_tutorials") + "/data/input.stl";
    pcl::io::loadPolygonFileSTL(file_path_stl, mesh);

    // converting mesh into point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // saving point cloud
    pcl::io::savePCDFileBinary("data/output.pcd", *cloud);

    // get your point cloud message from somewhere
    sensor_msgs::PointCloud2 cloud_msg;

    // publisher
    ros::Publisher pub_point_cloud = n.advertise<sensor_msgs::PointCloud2>(topic, queue_size);

    ros::Rate loop_rate(10); // updating with 10 Hz

    while(n.ok())
	{
        // publishing the cloud message
        pub_point_cloud.publish(cloud_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

/*  // showing point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("input.pcd", *cloud);

    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
*/
    
    return 0;
}