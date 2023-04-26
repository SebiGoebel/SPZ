#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl_ros/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "ros/package.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void testCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    //ROS_INFO("I heard: [%s]", msg.data.tostring());
    std::cout << "test" << std::endl;
}

// Callback-Funktion, um die PointCloud2-Nachricht zu empfangen
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
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
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisitiatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1000, pointCloudCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}