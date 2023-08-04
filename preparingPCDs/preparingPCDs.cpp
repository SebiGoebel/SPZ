#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Visualizer
#include <pcl/visualization/pcl_visualizer.h>

// Voxelgrid
#include <pcl/filters/voxel_grid.h>

// Transformations
#include <pcl/common/transforms.h>

// Plane Segmentation for Removing
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Extracting Indices
#include <pcl/filters/extract_indices.h>

// pass through filter
#include <pcl/filters/passthrough.h>

// Normal Estimation
#include <pcl/features/normal_3d.h>

// PPF
#include <pcl/features/ppf.h>

// HashMap Search
#include <pcl/registration/ppf_registration.h>

// ___INPUT:___

// #define filename_pcd_scene "../data/pointcloud_1_down_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_down.pcd"
#define filename_pcd_scene "../data/pointcloud_1_up.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_up_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_2.pcd"
// #define filename_pcd_scene "../data/pointcloud_3.pcd"
// #define filename_pcd_scene "../data/pointcloud_6.pcd"
#define filename_pcd_model "../data/teil_default.pcd"
// #define filename_pcd_model "../data/teil_leafSize5.pcd"

// ___OUTPUT:___
#define filename_pcd_result_scene "../results/pointcloud_1_up_prepared.pcd"
#define filename_pcd_result_model "../results/teil_default_prepared.pcd"

// ___RESIZING MODEL:___

// Object-Abmessungen
#define objectLenght 0.1238 // m
#define objectWidth 0.055   // m
#define objectHight 0.016   // m

#define resizingWithCalculatedRatio true // [true/false]
                                         // true  --> resizing factor for the object will be callculated based on Object-Abmessungen
                                         // false --> resizing factor is [scalingFactorForResizingObject]

#define scalingFactorForResizingObject 0.001 // resizing factor for manual resizing

// ___VOXELGRID:___
#define voxelLeafSize 0.005f // Voxelgröße in m (Tutorial: 1 cm --> 0.01)
// 0.005f
// 0.0025f --> beste

//  ___NORMAL-ESTIMATION:___
#define normalNeigborRadius 0.01f // tutorial: 0.03 --> Use all neighbors in a sphere of radius 3cm
// 0.01f
// 0.005f --> beste

double computeCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, float leafSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    // pcl::VoxelGrid<PointNT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leafSize, leafSize, leafSize);
    vg.setDownsampleAllData(false);
    vg.filter(*cloud_downsampled);

    double diameter_sqr = 0;
    for (size_t i = 0; i < cloud_downsampled->points.size(); i += 10)
    {
        for (size_t j = 0; j < cloud_downsampled->points.size(); j += 10)
        {
            if (i == j)
                continue;
            double distance_sqr = (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x) * (cloud_downsampled->points[i].x - cloud_downsampled->points[j].x) + (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y) * (cloud_downsampled->points[i].y - cloud_downsampled->points[j].y) + (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z) * (cloud_downsampled->points[i].z - cloud_downsampled->points[j].z);
            if (distance_sqr > diameter_sqr)
            {
                diameter_sqr = distance_sqr;
            }
        }
    }
    return sqrt(diameter_sqr);
}

double computeObjectDiameter(float length, float width, float hight)
{
    double diameter_sqr = length * length + width * width + hight * hight;
    return sqrt(diameter_sqr);
}

void littleViewer(const char *windowTitle, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    // Terminal message
    pcl::console::print_highlight("Visualizing ");
    std::cout << windowTitle << "..." << std::endl;

    // Visualiser
    pcl::visualization::PCLVisualizer viewer(windowTitle);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    viewer.addPointCloud<pcl::PointXYZ>(cloud);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void littleViewerPointNormal(const char *windowTitle, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &cloud)
{
    // Terminal message
    pcl::console::print_highlight("Visualizing ");
    std::cout << windowTitle << "..." << std::endl;

    // Visualiser
    pcl::visualization::PCLVisualizer viewer(windowTitle);
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();
    viewer.addPointCloud<pcl::PointNormal>(cloud);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

void comparisonViewer(const char *windowTitle, std::string text1, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud1, std::string text2, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud2)
{
    // Terminal message
    pcl::console::print_highlight("Visualizing ");
    std::cout << windowTitle << "..." << std::endl;

    // Visualiser
    pcl::visualization::PCLVisualizer viewer(windowTitle);
    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.0, 0.0, 0.0);

    int v1(0);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.addText(text1, 10, 10, "v1 text", v1);
    viewer.addPointCloud<pcl::PointXYZ>(cloud1, "cloud1", v1);

    int v2(0);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
    viewer.addText(text2, 10, 10, "v2 text", v2);
    viewer.addPointCloud<pcl::PointXYZ>(cloud2, "cloud2", v2);

    viewer.addCoordinateSystem(0.1);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}

int main()
{
    // ================ loading Pointclouds ================

    // point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::io::loadPCDFile("../data/table_scene_mug_stereo_textured.pcd", *cloud_unfiltered); // organised PCD
    // pcl::io::loadPCDFile(filename_pcd, *cloud_unfiltered);                                  // unorganised PCD

    // reading scene
    PCL_INFO("Reading scene ...\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd_scene, *cloud_scene) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd_scene);
        PCL_ERROR("\n");
        return (-1);
    }

    // reading model
    PCL_INFO("Reading models ...\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd_model, *cloud_model) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd_model);
        PCL_ERROR("\n");
        return (-1);
    }

    // visualizing input clouds
    comparisonViewer("Input Clouds", "Model", cloud_model, "Scene", cloud_scene);

    // =========================================================== Preparing Model ===========================================================

    // ---------------- centering the model ----------------

    pcl::console::print_highlight("Centering Model...\n");

    Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
    for (const auto &p : *(cloud_model))
        sum_of_pos += p.getVector3fMap().cast<double>();

    Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
    transform_centering.topRightCorner<3, 1>() = -sum_of_pos / cloud_model->size();

    pcl::transformPointCloud(*cloud_model, *cloud_model, transform_centering);
    // turning 90° um y
    // pcl::transformPointCloud(*cloud_model, *cloud_model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));

    // visualizing centered model
    littleViewer("centered model", cloud_model);

    // ---------------- resizing the model ----------------

    if (resizingWithCalculatedRatio)
    {
        pcl::console::print_highlight("Calculating Scalingfactor...\n");

        // computing the cloud diameter from the object
        double diameter_beforeResizing = computeCloudDiameter(cloud_model, 1.0);
        std::cout << "Object cloud diameter: " << diameter_beforeResizing << std::endl;

        // computing the diameter from the real object using abmessungen
        double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
        std::cout << "Object abmessungen diameter: " << diameterAbmessung << std::endl;

        // calculating the scaling ratio
        double scalingRatio = diameterAbmessung / diameter_beforeResizing;
        pcl::console::print_highlight("Scalingfactor: %f\n", scalingRatio);

        // actual resizing
        pcl::console::print_highlight("Resizing Object...\n");

        for (auto &point : *(cloud_model))
        {
            point.x *= scalingRatio;
            point.y *= scalingRatio;
            point.z *= scalingRatio;
        }

        // computing the diameter for checking
        double diameterCheck_cloud = computeCloudDiameter(cloud_model, 0.005);
        std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
        std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }
    else
    {
        pcl::console::print_highlight("Manual Scalingfactor: %f \n", scalingFactorForResizingObject);

        // resizing with manually set factor
        pcl::console::print_highlight("Resizing Model...\n");

        for (auto &point : *(cloud_model))
        {
            point.x *= scalingFactorForResizingObject;
            point.y *= scalingFactorForResizingObject;
            point.z *= scalingFactorForResizingObject;
        }

        // computing the diameter for checking
        double diameterCheck_cloud = computeCloudDiameter(cloud_model, 0.005);
        std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
        double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
        std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }

    // visualizing resized model
    littleViewer("resized model", cloud_model);

    // =========================================================== Preparing Scene ===========================================================

    // visualizing input cloud_scene
    littleViewer("Input cloud", cloud_scene);

    // ---------------- removing plane ----------------

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.0025);
    extract_plane.setNegative(true);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
    const auto nr_points = cloud_scene->size();

    while (cloud_scene->size() > 0.3 * nr_points)
    {
        seg.setInputCloud(cloud_scene);
        seg.segment(*inliers_plane, *coefficients_plane);
        PCL_INFO("inliers_plane: %zu\n", static_cast<std::size_t>(inliers_plane->indices.size()));
        if (inliers_plane->indices.size() < 50000)
            break;

        extract_plane.setInputCloud(cloud_scene);
        extract_plane.setIndices(inliers_plane);
        extract_plane.filter(*cloud_scene);
    }

    // visualizing cloud after plane removing
    littleViewer("plane removed", cloud_scene);

    // ---------------- removing walls ----------------

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_scene);

    // X:
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.18, 0.165);
    // pass.setNegative (true);
    pass.filter(*cloud_scene);

    // Y:
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.095, 0.1);
    // pass.setNegative (true);
    pass.filter(*cloud_scene);

    // visualizing cloud after wall removing
    littleViewer("walls removed", cloud_scene);

    // ---------------- self coded pass through filter ----------------
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

        // visualizing self coded pass through filter
        littleViewer("self coded passthrough filter", cloud_scene);
    */

    // visualizing the prepared model and scene
    comparisonViewer("prepared Clouds", "Model", cloud_model, "Scene", cloud_scene);

    // ================ saving Pointclouds ================

    // scene
    pcl::io::savePCDFileASCII(filename_pcd_result_scene, *cloud_scene);
    std::cerr << "Saved " << cloud_scene->size() << " data points to " << filename_pcd_result_scene << "." << std::endl;

    // model
    pcl::io::savePCDFileASCII(filename_pcd_result_model, *cloud_model);
    std::cerr << "Saved " << cloud_model->size() << " data points to " << filename_pcd_result_model << "." << std::endl;

    return 0;
}