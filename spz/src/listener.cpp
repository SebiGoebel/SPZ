#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "ros/package.h"

// Callback-Funktion, um die PointCloud2-Nachricht zu empfangen
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // PointCloud2-Nachricht in pcl::PointCloud umwandeln
  pcl::PointCloud<pcl::PointXYZ> pcl_pc;
  pcl::fromROSMsg(*msg, pcl_pc);

  // Pfad zur PointCloud-Datei
  std::string file_path = ros::package::getPath("beginner_tutorials") + "/data/pointcloud.pcd";

  // PointCloud speichern
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>(file_path, pcl_pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1000, pointCloudCallback);

  ros::spin();

  return 0;
}