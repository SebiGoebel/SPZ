#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// converting stl to point cloud
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>

// Normalestimation
#include <pcl/common/io.h>
#include <pcl/features/integral_image_normal.h>

// Normalestimation test
// #include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// visualizer
#include <pcl/visualization/pcl_visualizer.h>

// ================ converting STL2PCD ================

/*
int main()
{
    // Loading stl file
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFileSTL("../data/morobot-s_Achse-2B.STL", mesh);

    // converting mesh into point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // saving point cloud
    pcl::io::savePCDFileBinary("../data/output.pcd", *cloud);

    return 0;
}
*/

// ================ creating a random pointcloud with 5 points and saving it ================

/*
int main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 5;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.resize(cloud.width * cloud.height);

    for (auto &point : cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("../data/test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto &point : cloud)
    {
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    }

    std::cout << "\n TEST " << std::endl;

    return (0);
}
*/

// ================ Normal estimation ================

int main()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile("../data/table_scene_mug_stereo_textured.pcd", *cloud); // organised PCD
    pcl::io::loadPCDFile("../data/pointcloud_test_face.pcd", *cloud); // unorganised PCD

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // ----------------------- Normal Estimation Using Integral Images -----------------------

    // --> way faster
    // point cloud must be organised !!!

    /*
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT); // COVARIANCE_MATRIX,
                                                              // AVERAGE_3D_GRADIENT,
                                                              // AVERAGE_DEPTH_CHANGE,
                                                              // SIMPLE_3D_GRADIENT
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);
    */
    // ----------------------- Normal Estimation -----------------------

    // --> more accurate
    // PCD must noch be organised

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);

    // ----------------------- compute and visualize normals -----------------------

    // Compute the features
    ne.compute(*normals);

    // normals->size () should have the same size as the input cloud->size ()

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}