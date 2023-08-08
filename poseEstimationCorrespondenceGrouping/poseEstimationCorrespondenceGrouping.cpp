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

// normal estimation
#include <pcl/features/normal_3d_omp.h>

// uniform sampling
#include <pcl/filters/uniform_sampling.h>

// SHOT Descriptors:
#include <pcl/features/shot_omp.h>

// ___INPUT:___

// #define filename_pcd_scene "../data/pointcloud_1_down_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_down.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_up.pcd"
#define filename_pcd_scene "../data/pointcloud_1_up_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_2.pcd"
//  #define filename_pcd_scene "../data/pointcloud_3.pcd"
//  #define filename_pcd_scene "../data/pointcloud_6.pcd"
#define filename_pcd_model "../data/teil_default.pcd"
// #define filename_pcd_model "../data/teil_leafSize5.pcd"

// test
//#define filename_pcd_scene "../data/milk_cartoon_all_small_clorox.pcd"
//#define filename_pcd_model "../data/milk.pcd"

// ___RESIZING MODEL:___

// Object-Abmessungen
#define objectLenght 0.1238 // m
#define objectWidth 0.055   // m
#define objectHight 0.016   // m

#define resizingWithCalculatedRatio true // [true/false]
                                         // true  --> resizing factor for the object will be callculated based on Object-Abmessungen
                                         // false --> resizing factor is [scalingFactorForResizingObject]

#define scalingFactorForResizingObject 0.001 // resizing factor for manual resizing

// ___UNIFORM SAMPLING:___
#define uniformSamplingSearchRadiusModel 0.002f

#define uniformSamplingSearchRadiusScene 0.006f

// ___DESCRIPTORS WITH SHOT:___
#define descriptorRadius 0.004f

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

double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        if (!std::isfinite((*cloud)[i].x))
        {
            continue;
        }
        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt(sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

int main()
{
    // ================ loading Pointclouds ================

    // point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>());
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr scene_copy(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model_copy(new pcl::PointCloud<pcl::PointXYZ>);

    // pcl::io::loadPCDFile("../data/table_scene_mug_stereo_textured.pcd", *cloud_unfiltered); // organised PCD
    // pcl::io::loadPCDFile(filename_pcd, *cloud_unfiltered);                                  // unorganised PCD

    // reading scene
    PCL_INFO("Reading scene ...\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd_scene, *scene) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd_scene);
        PCL_ERROR("\n");
        return (-1);
    }

    // reading model
    PCL_INFO("Reading models ...\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd_model, *model) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd_model);
        PCL_ERROR("\n");
        return (-1);
    }

    // visualizing input clouds
    comparisonViewer("Input Clouds", "Model", model, "Scene", scene);

    // =========================================================== Preparing Model ===========================================================

    // ---------------- centering the model ----------------

    pcl::console::print_highlight("Centering Model...\n");

    Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
    for (const auto &p : *(model))
        sum_of_pos += p.getVector3fMap().cast<double>();

    Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
    transform_centering.topRightCorner<3, 1>() = -sum_of_pos / model->size();

    pcl::transformPointCloud(*model, *model, transform_centering);
    // turning 180° um y
    // pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 1, 0, 0));

    // visualizing centered model
    littleViewer("centered model", model);

    // ---------------- resizing the model ----------------

    if (resizingWithCalculatedRatio)
    {
        pcl::console::print_highlight("Calculating Scalingfactor...\n");

        // computing the cloud diameter from the object
        double diameter_beforeResizing = computeCloudDiameter(model, 1.0);
        std::cout << "Object cloud diameter: " << diameter_beforeResizing << std::endl;

        // computing the diameter from the real object using abmessungen
        double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
        std::cout << "Object abmessungen diameter: " << diameterAbmessung << std::endl;

        // calculating the scaling ratio
        double scalingRatio = diameterAbmessung / diameter_beforeResizing;
        pcl::console::print_highlight("Scalingfactor: %f\n", scalingRatio);

        // actual resizing
        pcl::console::print_highlight("Resizing Object...\n");

        for (auto &point : *(model))
        {
            point.x *= scalingRatio;
            point.y *= scalingRatio;
            point.z *= scalingRatio;
        }

        // computing the diameter for checking
        double diameterCheck_cloud = computeCloudDiameter(model, 0.005);
        std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
        std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }
    else
    {
        pcl::console::print_highlight("Manual Scalingfactor: %f \n", scalingFactorForResizingObject);

        // resizing with manually set factor
        pcl::console::print_highlight("Resizing Model...\n");

        for (auto &point : *(model))
        {
            point.x *= scalingFactorForResizingObject;
            point.y *= scalingFactorForResizingObject;
            point.z *= scalingFactorForResizingObject;
        }

        // computing the diameter for checking
        double diameterCheck_cloud = computeCloudDiameter(model, 0.005);
        std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
        double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
        std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }

    // visualizing resized model
    littleViewer("resized model", model);

    // =========================================================== Preparing Scene ===========================================================

    // visualizing input scene
    littleViewer("Input cloud", scene);

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
    const auto nr_points = scene->size();

    while (scene->size() > 0.3 * nr_points)
    {
        seg.setInputCloud(scene);
        seg.segment(*inliers_plane, *coefficients_plane);
        PCL_INFO("inliers_plane: %zu\n", static_cast<std::size_t>(inliers_plane->indices.size()));
        if (inliers_plane->indices.size() < 50000)
            break;

        extract_plane.setInputCloud(scene);
        extract_plane.setIndices(inliers_plane);
        extract_plane.filter(*scene);
    }

    // visualizing cloud after plane removing
    littleViewer("plane removed", scene);

    // ---------------- removing walls ----------------

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(scene);

    // X:
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.15, 0.165);
    // pass.setNegative (true);
    pass.filter(*scene);

    // Y:
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.09, 0.1);
    // pass.setNegative (true);
    pass.filter(*scene);

    // visualizing cloud after wall removing
    littleViewer("walls removed", scene);

    // ---------------- visualization ----------------

    // visualizing the prepared model and scene
    comparisonViewer("prepared Clouds", "Model", model, "Scene", scene);

    // resolution
    float resolution = static_cast<float>(computeCloudResolution(model));
    std::cout << "Model resolution:       " << resolution << std::endl;

    float resolution_scene = static_cast<float>(computeCloudResolution(scene));
    std::cout << "Scene resolution:       " << resolution_scene << std::endl;

    // =========================================================== separate Preparation DONE ===========================================================

    // vielleicht noch einmal VoxelGrid anwenden!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // ----------------------- Estamating Normals -----------------------

    pcl::console::print_highlight("Estimating Normals...\n");

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch(10);
    norm_est.setInputCloud(model);
    norm_est.compute(*model_normals);

    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    // ----------------------- Downsample Clouds to Extract keypoints -----------------------

    pcl::console::print_highlight("Extracting keypoints...\n");

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(uniformSamplingSearchRadiusModel);
    uniform_sampling.filter(*model_keypoints);
    std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(uniformSamplingSearchRadiusScene);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    // ----------------------- Compute Descriptors for keypoints with SHOT -----------------------

    pcl::console::print_highlight("Computing Descriptors for keypoints with SHOT...\n");

    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch(descriptorRadius);

    descr_est.setInputCloud(model_keypoints);
    descr_est.setInputNormals(model_normals);
    descr_est.setSearchSurface(model);
    descr_est.compute(*model_descriptors);

    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);

    // ----------------------- Finding Model-Scene Correspondences with KdTree -----------------------

    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (std::size_t i = 0; i < scene_descriptors->size(); ++i)
    {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) // skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
            pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
            model_scene_corrs->push_back(corr);
        }
    }
    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
}