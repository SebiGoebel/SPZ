#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>

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

// Hough Clustering
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>

// Geometric Clustering
#include <pcl/recognition/cg/geometric_consistency.h>

// GlobalHypothesis Verification
#include <pcl/recognition/hv/hv_go.h>

// ICP
#include <pcl/registration/icp.h>

// ___INPUT:___

// Scene
//#define filename_pcd_scene "../data/pointcloud_1_down_turned.pcd"
//#define filename_pcd_scene "../data/pointcloud_1_down.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_up.pcd"
//#define filename_pcd_scene "../data/pointcloud_1_up_turned.pcd"
//#define filename_pcd_scene "../data/pointcloud_2.pcd"
//#define filename_pcd_scene "../data/pointcloud_3.pcd"
 #define filename_pcd_scene "../data/pointcloud_6.pcd"

// Model
// #define filename_pcd_model "../data/teil_default.pcd"
// #define filename_pcd_model "../data/teil_leafSize5.pcd"
#define filename_pcd_model "../data/aufgenommen/teil_aufgenommen_up.pcd"
// #define filename_pcd_model "../data/aufgenommen/teil_aufgenommen_down.pcd"

// Model-List
#define filename_model_list "../data/modelList.txt"

// ___CENTERING MODEL:___

#define centeringModel true

// ___NORMAL ESTIMATION:___
#define pointNeighborsNormalEstimation 10 // ist die anzahl an nachbar Punkten die für eine Normal Estimation verwendet werden sollen
                                          // Tutorial: 10
// ___UNIFORM SAMPLING:___
#define uniformSamplingSearchRadiusModel 0.002f // 0.002f
// 0.002f --> ohne VoxelGrid

#define uniformSamplingSearchRadiusScene 0.006f // 0.006f
// 0.006f --> ohne VoxelGrid

// ___DESCRIPTORS WITH SHOT:___
#define descriptorRadius 0.006f // 0.006f // 0.006f
// 0.004f --> ohne VoxelGrid

// ___CLUSTERING:___
#define usingHough false
#define referenceFrameRadius 0.004f // is only needed when usingHough == true

#define clusteringSize 0.01f // wie groß soll der Bereich sein der geclustert werden soll in Meter
                             // 0.01 => 1 cm
// 0.1f --> ohne VoxelGrid mit Geometric Clustering

#define clusteringTH 5.0f
// 11.0f --> ohne VoxelGrid mit Geometric Clustering

// ____ICP:___
#define icpMaxIterations 30
#define icpCorrespondenceDistance 0.02375f // 0.005f
                                           // 0.02375 = 2.5 * resolution

// ___poseHypothesisVerification:____

#define poseHypothesisVerification false // [true/false] --> decides if pose Hypothesis Verification should be done
                                         // [true]  --> Global Hypotheses Verification is used
                                         // [false] --> ICP lowest Fitness Score is used

#define hvResolution 0.002f               // Tutorial: 0.005f
#define hvOccupancyGridResolution 0.0005f // Tutorial: 0.01
#define hvClutterRegularizer 1.0f         // default: 5.0
#define hvInlierTH 0.05f                  // default: 0.005
#define hvOcclusionTH 0.1f                // default: 0.01
#define hvClutterRadius 0.003f            // default: 0.03
#define hvRegulizer 3.0f                  // default: 3.0
#define hvNormalRadius 0.005              // default: 0.05
#define hvDetectClutter true              // Tutorial: true --> all good / false --> all bad

// ___VISUALIZATION:___
#define showingCorresopences true

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

void comparisonViewer(const char *windowTitle, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_list, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud_scene)
{
    // Terminal message
    pcl::console::print_highlight("Visualizing ");
    std::cout << windowTitle << "..." << std::endl;

    // Visualiser
    pcl::visualization::PCLVisualizer viewer(windowTitle);
    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(0.1);

    if (cloud_list.size() == 1)
    {
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(0), "cloud1", v1);

        int v2(0);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v2);
    }

    if (cloud_list.size() == 2)
    {
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.333, 1.0, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(0), "cloud1", v1);

        int v2(0);
        viewer.createViewPort(0.333, 0.0, 0.666, 1.0, v2);
        viewer.setBackgroundColor(0, 0, 0, v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(1), "cloud2", v2);

        int v3(0);
        viewer.createViewPort(0.666, 0.0, 1.0, 1.0, v3);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v3);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v3);
    }

    if (cloud_list.size() == 3)
    {
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.5, 0.5, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(0), "cloud1", v1);

        int v2(0);
        viewer.createViewPort(0.5, 0.0, 1.0, 0.5, v2);
        viewer.setBackgroundColor(0, 0, 0, v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(1), "cloud2", v2);

        int v3(0);
        viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v3);
        viewer.setBackgroundColor(0, 0, 0, v3);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(2), "cloud3", v3);

        int v4(0);
        viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v4);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v4);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v4);
    }

    if (cloud_list.size() == 4)
    {
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.333, 0.5, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(0), "cloud1", v1);

        int v2(0);
        viewer.createViewPort(0.333, 0.0, 0.666, 0.5, v2);
        viewer.setBackgroundColor(0, 0, 0, v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(1), "cloud2", v2);

        int v3(0);
        viewer.createViewPort(0.666, 0.0, 1.0, 0.5, v3);
        viewer.setBackgroundColor(0, 0, 0, v3);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(2), "cloud3", v3);

        int v4(0);
        viewer.createViewPort(0.0, 0.5, 0.5, 1.0, v4);
        viewer.setBackgroundColor(0, 0, 0, v4);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(3), "cloud4", v4);

        int v5(0);
        viewer.createViewPort(0.5, 0.5, 1.0, 1.0, v5);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v5);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v5);
    }

    if (cloud_list.size() == 5)
    {
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 0.333, 0.5, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(0), "cloud1", v1);

        int v2(0);
        viewer.createViewPort(0.333, 0.0, 0.666, 0.5, v2);
        viewer.setBackgroundColor(0, 0, 0, v2);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(1), "cloud2", v2);

        int v3(0);
        viewer.createViewPort(0.666, 0.0, 1.0, 0.5, v3);
        viewer.setBackgroundColor(0, 0, 0, v3);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(2), "cloud3", v3);

        int v4(0);
        viewer.createViewPort(0.0, 0.5, 0.333, 1.0, v4);
        viewer.setBackgroundColor(0, 0, 0, v4);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(3), "cloud4", v4);

        int v5(0);
        viewer.createViewPort(0.333, 0.5, 0.666, 1.0, v5);
        viewer.setBackgroundColor(0, 0, 0, v5);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_list.at(4), "cloud5", v5);

        int v6(0);
        viewer.createViewPort(0.666, 0.5, 1.0, 1.0, v6);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v6);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v6);
    }

    if (cloud_list.size() > 5)
    {
        std::cout << "der comparison Viewer unterstützt nicht so viele Ansichten" << std::endl;

        // nur Scene zeigen
        int v1(0);
        viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
        viewer.setBackgroundColor(0.3, 0.3, 0.3, v1);
        viewer.addPointCloud<pcl::PointXYZ>(cloud_scene, "cloud_scene", v1);
    }

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

bool spaceKeyPressed = false;
bool coordinateSystemsAreShown = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *viewer_void)
{
    // pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *>(viewer_void);
    if (event.getKeySym() == "space" && event.keyDown())
    {
        std::cout << "space was pressed ";
        spaceKeyPressed = true;
    }
}

int main()
{
    // ================ loading Pointclouds ================

    // model cloud vectors
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_models;
    std::vector<pcl::PointCloud<pcl::Normal>::Ptr> cloud_models_normals;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_models_keypoints;
    std::vector<pcl::PointCloud<pcl::SHOT352>::Ptr> cloud_models_descriptors;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>()); ///// ----> wird man am ende nicht mehr brauchen

    // scene clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>());

    // pcl::PointCloud<pcl::PointXYZ>::Ptr scene_copy(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model_copy(new pcl::PointCloud<pcl::PointXYZ>);

    // reading scene
    PCL_INFO("Reading scene ...\n");
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename_pcd_scene, *scene) == -1)
    {
        PCL_ERROR("\nCouldn't read file ");
        PCL_ERROR(filename_pcd_scene);
        PCL_ERROR("\n");
        return (-1);
    }
    PCL_INFO("Scene read: %s\n", filename_pcd_scene);

    // reading models
    PCL_INFO("Reading models ...\n");
    std::ifstream pcd_file_list(filename_model_list);
    while (!pcd_file_list.eof())
    {
        char str[512];
        pcd_file_list.getline(str, 512);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(str, *cloud) == -1)
        {
            PCL_ERROR("\nCouldn't read file ");
            PCL_ERROR(str);
            PCL_ERROR("\n");
            return (-1);
        }
        cloud_models.push_back(cloud);
        PCL_INFO("Model read: %s\n", str);
    }

    // resolution
    std::cout << "Model list size: " << cloud_models.size() << std::endl;

    for (int i = 0; i < cloud_models.size(); i++)
    {
        float resolution_1 = static_cast<float>(computeCloudResolution(cloud_models.at(i)));
        std::cout << "Model " << i << " resolution:       " << resolution_1 << std::endl;
    }

    float resolution_1_scene = static_cast<float>(computeCloudResolution(scene));
    std::cout << "Scene resolution:       " << resolution_1_scene << std::endl;

    // visualizing input clouds
    comparisonViewer("Input Clouds", cloud_models, scene);

    // =========================================================== Preparing Model ===========================================================

    // ---------------- centering the model ----------------

    if (centeringModel)
    {
        pcl::console::print_highlight("Centering Models...\n");

        for (int i = 0; i < cloud_models.size(); i++)
        {
            Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
            for (const auto &p : *(cloud_models.at(i)))
                sum_of_pos += p.getVector3fMap().cast<double>();

            Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
            transform_centering.topRightCorner<3, 1>() = -sum_of_pos / cloud_models.at(i)->size();

            pcl::transformPointCloud(*cloud_models.at(i), *cloud_models.at(i), transform_centering);
            // turning 180° um y
            // pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 1, 0, 0));

            // turning 90° um z
            // pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071068, 0, 0, 0.7071068));

            // visualizing centered model
            littleViewer("centered model", cloud_models.at(i));
        }
    }

    // hier würde resizing the model sein

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
    comparisonViewer("prepared Clouds", cloud_models, scene);

    // resolution
    for (int i = 0; i < cloud_models.size(); i++)
    {
        float resolution = static_cast<float>(computeCloudResolution(cloud_models.at(i)));
        std::cout << "Model " << i << " resolution:       " << resolution << std::endl;
    }

    float resolution_scene = static_cast<float>(computeCloudResolution(scene));
    std::cout << "Scene resolution:       " << resolution_scene << std::endl;

    // =========================================================== separate Preparation DONE ===========================================================

    // ----------------------- Estimating Normals -----------------------

    pcl::console::print_highlight("Estimating Normals...\n");

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch(pointNeighborsNormalEstimation); // 10

    // model
    for (int i = 0; i < cloud_models.size(); i++)
    {
        norm_est.setInputCloud(cloud_models.at(i));
        pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());
        norm_est.compute(*model_normals);

        cloud_models_normals.push_back(model_normals);
    }

    // scene
    norm_est.setInputCloud(scene);
    norm_est.compute(*scene_normals);

    // ----------------------- Downsample Clouds to Extract keypoints -----------------------

    pcl::console::print_highlight("Extracting keypoints...\n");

    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

    // model
    for (int i = 0; i < cloud_models.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
        uniform_sampling.setInputCloud(cloud_models.at(i));
        uniform_sampling.setRadiusSearch(uniformSamplingSearchRadiusModel);
        uniform_sampling.filter(*model_keypoints);
        std::cout << "Model " << i << " total points: " << cloud_models.at(i)->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

        cloud_models_keypoints.push_back(model_keypoints);
    }

    // scene
    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(uniformSamplingSearchRadiusScene);
    uniform_sampling.filter(*scene_keypoints);
    std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

    // ----------------------- Compute Descriptors for keypoints with SHOT -----------------------

    pcl::console::print_highlight("Computing Descriptors for keypoints with SHOT...\n");

    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
    descr_est.setRadiusSearch(descriptorRadius);

    // model
    for (int i = 0; i < cloud_models.size(); i++)
    {
        pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>());
        descr_est.setInputCloud(cloud_models_keypoints.at(i));
        descr_est.setInputNormals(cloud_models_normals.at(i));
        descr_est.setSearchSurface(cloud_models.at(i));
        descr_est.compute(*model_descriptors);

        cloud_models_descriptors.push_back(model_descriptors);
    }

    // scene
    descr_est.setInputCloud(scene_keypoints);
    descr_est.setInputNormals(scene_normals);
    descr_est.setSearchSurface(scene);
    descr_est.compute(*scene_descriptors);

    // ----------------------- Finding Model-Scene Correspondences with KdTree -----------------------

    pcl::console::print_highlight("Computing Model-Scene Correspondences...\n");

    std::vector<pcl::CorrespondencesPtr> cloud_models_scene_corrs;

    for (int j = 0; j < cloud_models.size(); j++)
    {
        pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

        pcl::KdTreeFLANN<pcl::SHOT352> match_search;
        match_search.setInputCloud(cloud_models_descriptors.at(j));

        std::vector<int> model_good_keypoints_indices;
        std::vector<int> scene_good_keypoints_indices;

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

                model_good_keypoints_indices.push_back(corr.index_query);
                scene_good_keypoints_indices.push_back(corr.index_match);
            }
        }

        // nur für visualisierung
        // pcl::PointCloud<pcl::PointXYZ>::Ptr model_good_kp(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::PointCloud<pcl::PointXYZ>::Ptr scene_good_kp(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::copyPointCloud(*cloud_models_keypoints.at(j), model_good_keypoints_indices, *model_good_kp);
        // pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

        std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

        cloud_models_scene_corrs.push_back(model_scene_corrs);
    }

    // ----------------------- Clustering -----------------------

    std::vector<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>> cloud_models_allTransformations; // von allen models
    std::vector<std::vector<pcl::Correspondences>> cloud_models_clustered_corrs;                                          // von allen models

    for (int i = 0; i < cloud_models.size(); i++)
    {
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> allTransformations_singleModel; // von einem Model
        std::vector<pcl::Correspondences> clustered_corrs;                                                      // von einem Model

        pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
        gc_clusterer.setGCSize(clusteringSize);
        gc_clusterer.setGCThreshold(clusteringTH);

        gc_clusterer.setInputCloud(cloud_models_keypoints.at(i));
        gc_clusterer.setSceneCloud(scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences(cloud_models_scene_corrs.at(i));

        // gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize(allTransformations_singleModel, clustered_corrs);

        cloud_models_allTransformations.push_back(allTransformations_singleModel);
        cloud_models_clustered_corrs.push_back(clustered_corrs);
    }

    // ----------------------- print if no instances where found -----------------------

    // code could be stopped here

    for (int i = 0; i < cloud_models.size(); i++)
    {
        if (cloud_models_allTransformations.at(i).size() <= 0)
        {
            std::cout << " !!!!!!!!!! No instances from Model " << i << " where found !!!!!!!!!!" << std::endl;
        }
        else
        {
            std::cout << "Model " << i << " instances found: " << cloud_models_allTransformations.at(i).size() << std::endl;
        }
    }

    // ----------------------- Output after Clustering (printing every Transformation for every model) -----------------------

    pcl::console::print_highlight("Outputting all instances...\n");

    for (int model_number = 0; model_number < cloud_models.size(); model_number++)
    {
        std::cout << "\n---------------------------------------------" << std::endl;
        std::cout << "  ----------------  Model " << model_number << " --------------" << std::endl;
        std::cout << "---------------------------------------------" << std::endl;

        for (int i = 0; i < cloud_models_allTransformations.at(model_number).size(); i++)
        {
            std::cout << "\n    Instance " << i << ":" << std::endl;
            std::cout << "        Correspondences belonging to this instance: " << cloud_models_clustered_corrs.at(model_number).at(i).size() << std::endl;

            // Print the rotation matrix and translation vector
            Eigen::Matrix3f rotation = cloud_models_allTransformations.at(model_number).at(i).block<3, 3>(0, 0);
            Eigen::Vector3f translation = cloud_models_allTransformations.at(model_number).at(i).block<3, 1>(0, 3);

            printf("\n");
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
            printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
            printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
            printf("\n");
            printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
        }

        /*
        // filtering model with most correspondence
        int highestChecker = 0;
        int indexChecker = -1;
        for (int i = 0; i < cloud_models_allTransformations.at(model_number).size(); i++)
        {
            if (cloud_models_clustered_corrs.at(model_number).at(i).size() > highestChecker)
            {
                highestChecker = cloud_models_clustered_corrs.at(model_number).at(i).size();
                indexChecker = i;
            }
        }
        std::cout << "highestChecker = " << highestChecker << std::endl;
        std::cout << "indexChecker = " << indexChecker << std::endl;
        */
    }

    // ----------------------- Visualisierung von allen gefundenen Instanzen -----------------------

    pcl::console::print_highlight("Visualising Results of all found Instances...\n");

    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.initCameraParameters();
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addCoordinateSystem(0.1);
    viewer.addPointCloud(scene, "scene_cloud");

    for (int model_number = 0; model_number < cloud_models.size(); model_number++)
    {
        // model color handler
        double cloud_color_r, cloud_color_g, cloud_color_b;
        if (model_number == 0) // red
        {
            cloud_color_r = 255;
            cloud_color_g = 0;
            cloud_color_b = 0;
        }
        if (model_number == 1) // blue
        {
            cloud_color_r = 0;
            cloud_color_g = 0;
            cloud_color_b = 255;
        }
        if (model_number == 2) // pink
        {
            cloud_color_r = 255;
            cloud_color_g = 0;
            cloud_color_b = 255;
        }
        if (model_number == 3) // yellow
        {
            cloud_color_r = 255;
            cloud_color_g = 255;
            cloud_color_b = 128;
        }
        if (model_number == 4) // cyan
        {
            cloud_color_r = 0;
            cloud_color_g = 255;
            cloud_color_b = 255;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

        if (showingCorresopences)
        {
            // for transdorming shown models
            float modelVis_x = 0.25 - model_number * 0.25;
            float modelVis_y = 0.2;

            if (model_number >= 3)
            {
                modelVis_y = -0.15;
                modelVis_x += 0.5;
            }

            // Transforming all shown models, so that they don't end up in the scene or in each other
            pcl::transformPointCloud(*cloud_models.at(model_number), *off_scene_model, Eigen::Vector3f(modelVis_x, modelVis_y, 0), Eigen::Quaternionf(0, 0, 0, 1));
            pcl::transformPointCloud(*cloud_models_keypoints.at(model_number), *off_scene_model_keypoints, Eigen::Vector3f(modelVis_x, modelVis_y, 0), Eigen::Quaternionf(0, 0, 0, 1));

            // string id + color handler + adding Point Cloud
            std::stringstream ss_model;
            ss_model << "model_" << model_number;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler(off_scene_model, cloud_color_r, cloud_color_g, cloud_color_b);
            viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, ss_model.str());
        }

        for (int i = 0; i < cloud_models_allTransformations.at(model_number).size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*cloud_models.at(model_number), *transformed_model, cloud_models_allTransformations.at(model_number).at(i));

            // string id + color handler + adding Point Cloud
            std::stringstream ss_transformed_cloud;
            ss_transformed_cloud << "model_" << model_number << "_instance_" << i;
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_model_color_handler(transformed_model, cloud_color_r, cloud_color_g, cloud_color_b);
            viewer.addPointCloud(transformed_model, transformed_model_color_handler, ss_transformed_cloud.str());

            // show CoordinateSystem
            // converting Eigen::Matrix4f to Eigen::Affine3f
            Eigen::Affine3f transformationVisualization;
            transformationVisualization = cloud_models_allTransformations.at(model_number).at(i);

            // string id + adding Coordinatesystem
            std::stringstream ss_coordinatesystem;
            ss_coordinatesystem << "model_" << model_number << "object_pose_" << i;
            viewer.addCoordinateSystem(0.1, transformationVisualization, ss_coordinatesystem.str(), 0);

            if (showingCorresopences)
            {
                for (int j = 0; j < cloud_models_clustered_corrs.at(model_number).at(i).size(); j++)
                {
                    std::stringstream ss_line;
                    ss_line << "model_" << model_number << "correspondence_line" << i << "_" << j;

                    pcl::PointXYZ &model_point = off_scene_model_keypoints->at(cloud_models_clustered_corrs.at(model_number)[i][j].index_query);
                    pcl::PointXYZ &scene_point = scene_keypoints->at(cloud_models_clustered_corrs.at(model_number)[i][j].index_match);

                    //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
                    viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str()); // lines in green
                }
            }
        }
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    // ----------------------- Generating the clouds of the found instances -----------------------

    // schreibe alle hypothesen von allen models in einen instanzen-vector

    pcl::console::print_highlight("Transforming all instances...\n");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances; // ein vector mit allen instanzen
    // std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> allTransformations; // ein vector mit allen Transformationen
    std::vector<Eigen::Matrix4f> allTransformations; // ein vector mit allen Transformationen

    for (int model_number = 0; model_number < cloud_models.size(); model_number++)
    {
        for (int i = 0; i < cloud_models_allTransformations.at(model_number).size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*cloud_models.at(model_number), *transformed_model, cloud_models_allTransformations.at(model_number).at(i));
            instances.push_back(transformed_model);
            allTransformations.push_back(cloud_models_allTransformations.at(model_number).at(i)); // übernehmen aller Transformationen in einen eizelnen vector
        }
    }

    /*
        // schreibe alle hypothesen von allen models in einen instanzen-vector bestehend aus instanzen vectoren
        // so siend die einzelnen vectoren pro model und es ist gegliedert aufgebaut

        pcl::console::print_highlight("Transforming all instances...\n");

        std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>> instances;

        for (int model_number = 0; model_number < cloud_models.size(); model_number++)
        {
            std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances_single_model;
            for (int i = 0; i < cloud_models_allTransformations.at(model_number).size(); i++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::transformPointCloud(*cloud_models.at(model_number), *transformed_model, cloud_models_allTransformations.at(model_number).at(i));
                instances_single_model.push_back(transformed_model);
            }
            instances.push_back(instances_single_model);
        }
    */
    // ----------------------- ICP -----------------------

    pcl::console::print_highlight("ICP...\n");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> registered_instances;
    std::vector<Eigen::Matrix4f> transformation_icp;
    std::vector<double> icp_scores;

    for (int i = 0; i < instances.size(); i++)
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaximumIterations(icpMaxIterations);
        icp.setMaxCorrespondenceDistance(icpCorrespondenceDistance); // 2.5 * resolution // resolution = 0.0095
        icp.setInputTarget(scene);
        icp.setInputSource(instances[i]);
        pcl::PointCloud<pcl::PointXYZ>::Ptr registered(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*registered);
        registered_instances.push_back(registered);

        // Printing results of ICP
        std::cout << "Instance " << i << " ";
        if (icp.hasConverged())
        {
            std::cout << "Aligned!"
                      << " --- ICP Fitness Score: " << icp.getFitnessScore() << std::endl;
            icp_scores.push_back(icp.getFitnessScore());
        }
        else
        {
            std::cout << "Not Aligned!" << std::endl;
            icp_scores.push_back(100.0);
        }
        transformation_icp.push_back(icp.getFinalTransformation());
    }

    // ================================ Hypothesis Verification ================================

    if (poseHypothesisVerification)
    {
        // ----------------------- Global Hypothesis Verification -----------------------

        std::cout << "----------------------" << std::endl;
        pcl::console::print_highlight("Hypothesis Verification using Global Hypotheses Verification...\n");

        std::vector<bool> hypotheses_mask; // Mask Vector to identify positive hypotheses

        pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> GoHv;

        GoHv.setSceneCloud(scene);                  // Scene Cloud
        GoHv.addModels(registered_instances, true); // Models to verify
        GoHv.setResolution(hvResolution);
        GoHv.setResolutionOccupancyGrid(hvOccupancyGridResolution);
        GoHv.setInlierThreshold(hvInlierTH);
        GoHv.setOcclusionThreshold(hvOcclusionTH);
        GoHv.setRegularizer(hvRegulizer);
        GoHv.setRadiusClutter(hvClutterRadius);
        GoHv.setClutterRegularizer(hvClutterRegularizer);
        GoHv.setDetectClutter(hvDetectClutter);
        GoHv.setRadiusNormals(hvNormalRadius);

        GoHv.verify();
        GoHv.getMask(hypotheses_mask); // i-element TRUE if hvModels[i] verifies hypotheses

        for (std::size_t i = 0; i < hypotheses_mask.size(); i++)
        {
            if (hypotheses_mask[i])
            {
                std::cout << "Instance " << i << " is GOOD! <---" << std::endl;
            }
            else
            {
                std::cout << "Instance " << i << " is bad!" << std::endl;
            }
        }

        std::cout << "----------------------" << std::endl;

        // std::cout << "hypotheses_mask.size: " << hypotheses_mask.size() << std::endl;
        // std::cout << "instances.size: " << instances.size() << std::endl;

        // ----------------------- Visualizing after Hypothesis Verification -----------------------

        pcl::visualization::PCLVisualizer viewer_hv("Hypotheses Verification");
        viewer_hv.addCoordinateSystem(0.1);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_handler(scene, 255, 0, 0); // red
        viewer_hv.addPointCloud(scene, scene_color_handler, "scene_cloud");

        for (int i = 0; i < instances.size(); i++)
        {
            std::stringstream ss_instance;
            ss_instance << "instance_" << i;

            // All instances in red
            // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], 255, 0, 0); // red
            // viewer_hv.addPointCloud(instances[i], instance_color_handler, ss_instance.str());

            // show CoordinateSystem
            // converting Eigen::Matrix4f to Eigen::Affine3f
            Eigen::Affine3f transformationVisualization_icp;
            transformationVisualization_icp = transformation_icp[i] * allTransformations[i];
            std::stringstream ss_coordinatesystem;
            ss_coordinatesystem << "object_pose_" << i;
            viewer_hv.addCoordinateSystem(0.1, transformationVisualization_icp, ss_coordinatesystem.str(), 0);

            // if verification is good change color to green
            ss_instance << "_registered" << std::endl;
            if (hypotheses_mask[i])
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> registered_instance_color_handler(registered_instances[i], 0, 255, 0); // green
                viewer_hv.addPointCloud(registered_instances[i], registered_instance_color_handler, ss_instance.str());
            }
            else
            {
                // change color to cyan
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> registered_instance_color_handler(registered_instances[i], 0, 255, 255); // cyan - türkis
                viewer_hv.addPointCloud(registered_instances[i], registered_instance_color_handler, ss_instance.str());
            }
        }

        while (!viewer_hv.wasStopped())
        {
            viewer_hv.spinOnce();
        }
    }
    else
    {
        // ----------------------- ICP Fitness Scores -----------------------

        std::cout << "----------------------" << std::endl;
        pcl::console::print_highlight("Hypothesis Verification using lowest ICP Fitness Score...\n");

        // test printing icp scores
        for (int i = 0; i < icp_scores.size(); i++)
        {
            std::cout << "Score " << i << ": " << icp_scores.at(i) << std::endl;
        }

        // getting lowest icp score
        int index_lowestICPScore;
        double min_ICPScore = 100.0;
        for (int i = 0; i < icp_scores.size(); i++)
        {
            if (icp_scores.at(i) < min_ICPScore)
            {
                min_ICPScore = icp_scores.at(i);
                index_lowestICPScore = i;
            }
        }

        std::cout << "\n-------------------------------\n" << std::endl;
        pcl::console::print_highlight("Result of Hypothesis Verification using lowest ICP Fitness Score...\n");
        std::cout << "------" << std::endl;
        std::cout << "lowest ICP: " << min_ICPScore << " at " << index_lowestICPScore << std::endl;

        // ----------------------- Visualizing after Hypothesis Verification -----------------------

        pcl::visualization::PCLVisualizer viewer_hv("Hypotheses Verification");
        viewer_hv.addCoordinateSystem(0.1);
        // viewer_hv.initCameraParameters();

        // show scene
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_handler(scene, 255, 0, 0); // red
        viewer_hv.addPointCloud(scene, scene_color_handler, "scene_cloud");

        // show CoordinateSystem
        // converting Eigen::Matrix4f to Eigen::Affine3f
        Eigen::Affine3f transformationVisualization_icp;
        transformationVisualization_icp = transformation_icp[index_lowestICPScore] * allTransformations[index_lowestICPScore];
        std::stringstream ss_coordinatesystem;
        ss_coordinatesystem << "object_pose_" << index_lowestICPScore;
        viewer_hv.addCoordinateSystem(0.1, transformationVisualization_icp, ss_coordinatesystem.str(), 0);

        // show fitted model
        std::stringstream ss_instance;
        ss_instance << "instance_" << index_lowestICPScore << "_registered";
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> registered_instance_color_handler(registered_instances[index_lowestICPScore], 0, 255, 0); // green
        viewer_hv.addPointCloud(registered_instances[index_lowestICPScore], registered_instance_color_handler, ss_instance.str());

        // printing final Transformation
        std::cout << "\nFinal Transformation:" << std::endl;
        printf("\n");
        Eigen::Matrix4f finalTransformation = transformation_icp[index_lowestICPScore] * allTransformations[index_lowestICPScore];
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", finalTransformation(0, 0), finalTransformation(0, 1), finalTransformation(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", finalTransformation(1, 0), finalTransformation(1, 1), finalTransformation(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", finalTransformation(2, 0), finalTransformation(2, 1), finalTransformation(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", finalTransformation(0, 3), finalTransformation(1, 3), finalTransformation(2, 3));
        pcl::console::print_info("\n");

        while (!viewer_hv.wasStopped())
        {
            viewer_hv.spinOnce();
        }
    }

    return 0;
}