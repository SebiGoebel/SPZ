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
#include <pcl/features/normal_3d.h>

// visualizer
#include <pcl/visualization/pcl_visualizer.h>

// PFH - Point Feature Histogram
// #include <pcl/point_types.h>
#include <pcl/features/pfh.h>

// FPFH - Fast Point Feature Histogram
// #include <pcl/point_types.h>
#include <pcl/features/fpfh.h>

// VFH - Viewpoint Feature Histogram
// #include <pcl/point_types.h>
#include <pcl/features/vfh.h>

// Downsampling with VoxelGrid
// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// plane removing
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// extracting indices
#include <pcl/filters/extract_indices.h>

// statistical outlier removal
#include <pcl/filters/statistical_outlier_removal.h>

// pass through filter
#include <pcl/filters/passthrough.h>

#define convertingSTL2PCD false

// #define filename_pcd "../data/pointcloud_1_down_turned.pcd"
// #define filename_pcd "../data/pointcloud_1_down.pcd"
#define filename_pcd "../data/pointcloud_1_up.pcd"
// #define filename_pcd "../data/pointcloud_1_up_turned.pcd"
// #define filename_pcd "../data/pointcloud_2.pcd"
// #define filename_pcd "../data/pointcloud_3.pcd"
// #define filename_pcd "../data/pointcloud_6.pcd"
// #define filename_pcd "../data/teil_default.pcd"
//  #define filename_pcd "../data/teil_leafSize5.pcd"

#define downsamplingWithVoxelGrid false
#define leafSize 0.005f // Voxelgröße von 1 cm (0.01 Meter)

#define normalEstimationMethod 2 // 1 -> Normal Estimation Using Integral Images (faster, less accurate, PCD must be organised !!!)
                                 // 2 -> Normal Estimation                       (slow, more accurate, PCDs don't have to be organised)

#define featureMethod 2 // 1 -> PFH  - Point Feature Histograms
                        // 2 -> FPFH - Fast Point Feature Histograms
                        // 3 -> VFH  - Viewpoint Feature Histogram

// Use all neighbors in a sphere of radius 3cm
#define normalNeigborRadius 0.005 // tutorial: 0.03

// Use all neighbors in a sphere of radius 5cm
#define featureNormalNeighborRadius 0.025 // tutorial: 0.05

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

int main()
{
    // ================ creating Point CLouds ================

    // ================ converting STL2PCD ================

    if (convertingSTL2PCD)
    {
        std::cout << "--- converting STL2PCD ---" << std::endl;

        // Loading stl file
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFileSTL("../teil/morobot-s_Achse-2B.STL", mesh);

        // converting mesh into point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr convertedCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh.cloud, *convertedCloud);

        // saving point cloud
        std::string filename_savingPCD = "../data/output_convertingSTL2PCD.pcd";
        // pcl::io::savePCDFileBinary(filename_savingPCD, *convertedCloud);
        pcl::io::savePCDFileASCII(filename_savingPCD, *convertedCloud);
        std::cout << "Saved " << convertedCloud->size() << " data points to " << filename_savingPCD << std::endl;

        // visualize converted cloud
        pcl::visualization::PCLVisualizer viewer_0("PCL Viewer");
        viewer_0.setBackgroundColor(0.0, 0.0, 0.0);
        viewer_0.addPointCloud<pcl::PointXYZ>(convertedCloud);
        viewer_0.addCoordinateSystem(0.1);
        viewer_0.initCameraParameters();

        while (!viewer_0.wasStopped())
        {
            viewer_0.spinOnce();
        }

        return 0; // to stop code here
    }

    // ================ loading Pointcloud ================

    std::cout << "--- loading Pointcloud ---" << std::endl;

    // load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_unfiltered(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::io::loadPCDFile("../data/table_scene_mug_stereo_textured.pcd", *cloud_unfiltered); // organised PCD
    // pcl::io::loadPCDFile(filename_pcd, *cloud_unfiltered);                                  // unorganised PCD

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd, *cloud_unfiltered) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd);
        PCL_ERROR("\n");
        return (-1);
    }

    // visualize input cloud
    pcl::visualization::PCLVisualizer viewer_1("Input Cloud");
    viewer_1.setBackgroundColor(0.0, 0.0, 0.0);
    viewer_1.addPointCloud<pcl::PointXYZ>(cloud_unfiltered);
    // viewer_1.addCoordinateSystem (0.1);
    viewer_1.initCameraParameters();

    while (!viewer_1.wasStopped())
    {
        viewer_1.spinOnce();
    }

    // ================ removing plane ================

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.0025);
    extract_plane.setNegative(true);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
    const auto nr_points = cloud_unfiltered->size();

    while (cloud_unfiltered->size() > 0.3 * nr_points)
    {
        seg.setInputCloud(cloud_unfiltered);
        seg.segment(*inliers_plane, *coefficients);
        PCL_INFO("inliers_plane: %zu\n", static_cast<std::size_t>(inliers_plane->indices.size()));
        if (inliers_plane->indices.size() < 50000)
            break;

        extract_plane.setInputCloud(cloud_unfiltered);
        extract_plane.setIndices(inliers_plane);
        extract_plane.filter(*cloud_unfiltered);
    }

    // visualize cloud after plane removing
    pcl::visualization::PCLVisualizer viewer_planeRemover("After plane remover");
    viewer_planeRemover.setBackgroundColor(0.0, 0.0, 0.0);
    viewer_planeRemover.addCoordinateSystem(0.1);
    viewer_planeRemover.initCameraParameters();

    viewer_planeRemover.addPointCloud<pcl::PointXYZ>(cloud_unfiltered);

    while (!viewer_planeRemover.wasStopped())
    {
        viewer_planeRemover.spinOnce();
    }

    // ================ statistical outlier removal ================
    /*
        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_unfiltered);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_unfiltered);

        // visualize statistical outlier removal
        pcl::visualization::PCLVisualizer viewer_outlierRemover("After outlier remover");
        viewer_outlierRemover.setBackgroundColor(0.0, 0.0, 0.0);
        viewer_outlierRemover.addCoordinateSystem(0.1);
        viewer_outlierRemover.initCameraParameters();

        viewer_outlierRemover.addPointCloud<pcl::PointXYZ>(cloud_unfiltered);

        while (!viewer_outlierRemover.wasStopped())
        {
            viewer_outlierRemover.spinOnce();
        }
    */

    // ================ passthrough filter ================

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_unfiltered);

    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.18, 0.165);
    //pass.setNegative (true);
    pass.filter(*cloud_unfiltered);

    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.095, 0.1);
    //pass.setNegative (true);
    pass.filter(*cloud_unfiltered);

    // visualize cloud after pass through filter
    pcl::visualization::PCLVisualizer viewer_passThrough("After pass through filter");
    viewer_passThrough.setBackgroundColor(0.0, 0.0, 0.0);
    viewer_passThrough.addCoordinateSystem(0.1);
    viewer_passThrough.initCameraParameters();

    viewer_passThrough.addPointCloud<pcl::PointXYZ>(cloud_unfiltered);

    while (!viewer_passThrough.wasStopped())
    {
        viewer_passThrough.spinOnce();
    }

    // ================ self coded pass through filter ================
/*
    pcl::PointIndices::Ptr inliers_wall(new pcl::PointIndices());
    pcl::ExtractIndices<pcl::PointXYZ> extract_wall;

    float x_limit = 0.16f;
    float y_limit = 0.095f;

    int index = 0;
    for (auto &point : *(cloud_unfiltered))
    {
        if (point.x > x_limit || point.x < -x_limit)
        {
            inliers_wall->indices.push_back(index);
        }

        if (point.y > y_limit || point.y < -y_limit)
        {
            inliers_wall->indices.push_back(index);
        }

        // test mit z
        // if(point.z > 0.3f){
        //    inliers_wall->indices.push_back(index);
        //}
        index++;
    }

    extract_wall.setInputCloud(cloud_unfiltered);
    extract_wall.setIndices(inliers_wall);
    extract_wall.setNegative(true);
    extract_wall.filter(*cloud_unfiltered);

    // visualize cloud after wall removing
    pcl::visualization::PCLVisualizer viewer_wallRemover("After wall remover");
    viewer_wallRemover.setBackgroundColor(0.0, 0.0, 0.0);
    viewer_wallRemover.addCoordinateSystem(0.1);
    viewer_wallRemover.initCameraParameters();

    viewer_wallRemover.addPointCloud<pcl::PointXYZ>(cloud_unfiltered);

    while (!viewer_wallRemover.wasStopped())
    {
        viewer_wallRemover.spinOnce();
    }
*/
    // ==================================== Downsampling with VoxelGrid ====================================

    // pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (downsamplingWithVoxelGrid == true)
    {
        std::cout << "\n--- Downsampling with VoxelGrid ---" << std::endl;
        // Create the filtering object
        // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud_unfiltered);
        voxel_grid.setLeafSize(leafSize, leafSize, leafSize); // Voxelgröße von 1 cm (0.01 Meter)
        voxel_grid.filter(*cloud);
    }
    else
    {
        *cloud = *cloud_unfiltered;
    }

    // visualize filtered cloud
    pcl::visualization::PCLVisualizer viewer_2("filtered Cloud with VoxelGrid");
    viewer_2.setBackgroundColor(0.0, 0.0, 0.0);
    viewer_2.addPointCloud<pcl::PointXYZ>(cloud);
    // viewer_2.addCoordinateSystem (0.1);
    viewer_2.initCameraParameters();

    while (!viewer_2.wasStopped())
    {
        viewer_2.spinOnce();
    }

    // ==================================== Normal estimation ====================================

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    std::cout << "\n--- Normal Estimation ---" << std::endl;

    // ----------------------- Normal Estimation Using Integral Images -----------------------

    // --> way faster
    // point cloud must be organised !!!

    if (normalEstimationMethod == 1)
    {
        std::cout << "Computing Normal Estimation using Integral Images ..." << std::endl;

        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT); // COVARIANCE_MATRIX,
                                                              // AVERAGE_3D_GRADIENT,
                                                              // AVERAGE_DEPTH_CHANGE,
                                                              // SIMPLE_3D_GRADIENT
        ne.setMaxDepthChangeFactor(0.02f);
        ne.setNormalSmoothingSize(10.0f);
        ne.setInputCloud(cloud);

        // Compute the features
        ne.compute(*normals);
    }

    // ----------------------- Normal Estimation -----------------------

    // --> more accurate
    // PCD must not be organised

    if (normalEstimationMethod == 2)
    {
        std::cout << "Computing Normal Estimation ..." << std::endl;

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree_normals);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(normalNeigborRadius);

        // Compute the features
        ne.compute(*normals);
    }

    // ----------------------- visualize normals -----------------------

    // normals->size () should have the same size as the input cloud->size ()
    if (cloud->size() == normals->size())
    {
        std::cout << "normals_size correct !!!" << std::endl;
        std::cout << "cloud_size = normals_size" << std::endl;
    }
    else
    {
        std::cout << "normals_size NOT correct !!!" << std::endl;
        std::cout << "cloud_size != normals_size sollten aber gleich sein" << std::endl;
        std::cout << "Normals_Size: " << normals->size() << std::endl;
        std::cout << "Cloud_Size:   " << cloud->size() << std::endl;
    }

    // check if normals are finite (not NaN or infinite)
    int notFiniteNormalsCounter = 0;
    for (int i = 0; i < normals->size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>((*normals)[i]))
        {
            // PCL_WARN("normals[%d] is not finite\n", i);
            notFiniteNormalsCounter++;
        }
    }

    std::cout << "Anz. der nicht finiten Normalen: " << notFiniteNormalsCounter << std::endl;

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("normal estimation");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);
    // viewer.addCoordinateSystem (0.1);
    viewer.initCameraParameters();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    // ==================================== Features ====================================

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

    // ... read, pass in or create a point cloud with normals...
    // ... (note: you can create a single PointCloud<PointNormal> if you want)...

    // taking pointcloud and normals from before

    // ----------------------- PFH - Point Feature Histograms -----------------------

    // way too slow !!!!!!!!!!!!!!!! (never finisched) --> 눈_눈

    if (featureMethod == 1)
    {
        std::cout << "\n--- PFH - Point Feature Histograms ---" << std::endl;

        // Create the PFH estimation class, and pass the input dataset + normals to it
        pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
        pfh.setInputCloud(cloud);
        pfh.setInputNormals(normals); // alternatively, if cloud is of type PointNormal, do pfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the PFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_pfh(new pcl::search::KdTree<pcl::PointXYZ>());
        // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree_pfh (new pcl::KdTreeFLANN<pcl::PointXYZ> ()); // older call for PCL 1.5-

        pfh.setSearchMethod(tree_pfh);

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        pfh.setRadiusSearch(featureNormalNeighborRadius); // surface normals --> with 3 cm

        // Output datasets
        pcl::PointCloud<pcl::PFHSignature125>::Ptr pfhs(new pcl::PointCloud<pcl::PFHSignature125>());

        // Compute the features
        std::cout << "Computing features with PFH - Point Feature Histograms ..." << std::endl;
        pfh.compute(*pfhs);

        // pfhs->size () should have the same size as the input cloud->size ()*
        if (pfhs->size() == cloud->size())
        {
            std::cout << "PFH Size correct !!!" << std::endl;
        }
        else
        {
            std::cout << "!!! PFHs Size NOT correct !!!" << std::endl;
            std::cout << "PFHs sollte gleich wie cloud size sein:" << std::endl;
            std::cout << "Cloud_size: " << cloud->size() << std::endl;
            std::cout << "PFHs_size: " << pfhs->size() << std::endl;
        }
    }

    // ----------------------- FPFH - Fast Point Feature Histograms -----------------------

    // way faster --> ♥‿♥

    if (featureMethod == 2)
    {
        std::cout << "\n--- FPFH - Fast Point Feature Histograms ---" << std::endl;

        // Create the FPFH estimation class, and pass the input dataset + normals to it
        pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals); // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);

        fpfh.setSearchMethod(tree_fpfh);

        // Use all neighbors in a sphere of radius 5cm
        // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
        fpfh.setRadiusSearch(featureNormalNeighborRadius);

        // Output datasets
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());

        // Compute the features
        std::cout << "Computing features with FPFH - Fast Point Feature Histograms ..." << std::endl;
        fpfh.compute(*fpfhs);

        // fpfhs->size () should have the same size as the input cloud->size ()*
        if (fpfhs->size() == cloud->size())
        {
            std::cout << "FPFH Size correct !!!" << std::endl;
        }
        else
        {
            std::cout << "!!! FPFHs Size NOT correct !!!" << std::endl;
            std::cout << "FPFHs sollte gleich wie cloud size sein:" << std::endl;
            std::cout << "Cloud_size: " << cloud->size() << std::endl;
            std::cout << "FPFHs_size: " << fpfhs->size() << std::endl;
        }
    }

    // ----------------------- VFH - Viewpoint Feature Histogram -----------------------

    // the absolut fastest (no latency at all !!! --> o_o) -->（　ﾟДﾟ

    if (featureMethod == 3)
    {
        std::cout << "\n--- VFH - Viewpoint Feature Histogram ---" << std::endl;

        // Create the VFH estimation class, and pass the input dataset+normals to it
        pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
        vfh.setInputCloud(cloud);
        vfh.setInputNormals(normals); // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

        // Create an empty kdtree representation, and pass it to the FPFH estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_vfh(new pcl::search::KdTree<pcl::PointXYZ>());

        vfh.setSearchMethod(tree_vfh);

        // Output datasets
        pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());

        // Compute the features
        std::cout << "Computing features with VFH - Viewpoint Feature Histogram ..." << std::endl;
        vfh.compute(*vfhs);

        // vfhs->size () should be of size 1*
        if (vfhs->size() == 1)
        {
            std::cout << "VFHS Size correct !!!" << std::endl;
            std::cout << "VFHS_size: " << vfhs->size() << std::endl;
        }
        else
        {
            std::cout << "!!! vfhs size NOT correct !!!" << std::endl;
            std::cout << "vfhs sollte 1 sein:" << std::endl;
            std::cout << "vfhs_size: " << vfhs->size() << std::endl;
        }
    }

    return 0;
}