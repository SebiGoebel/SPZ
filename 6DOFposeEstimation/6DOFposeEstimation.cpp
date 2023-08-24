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

// #define filename_pcd_scene "../data/pointcloud_1_down_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_down.pcd"
//#define filename_pcd_scene "../data/pointcloud_1_up.pcd"
//#define filename_pcd_scene "../data/pointcloud_1_up_turned.pcd"
//#define filename_pcd_scene "../data/pointcloud_2.pcd"
   #define filename_pcd_scene "../data/pointcloud_3.pcd"
//#define filename_pcd_scene "../data/pointcloud_6.pcd"


//#define filename_pcd_model "../data/teil_default.pcd"
//#define filename_pcd_model "../data/teil_leafSize5.pcd"
#define filename_pcd_model "../data/aufgenommen/teil_aufgenommen_up.pcd"
//#define filename_pcd_model "../data/aufgenommen/teil_aufgenommen_down.pcd"

// test
// #define filename_pcd_scene "../data/milk_cartoon_all_small_clorox.pcd"
// #define filename_pcd_model "../data/milk.pcd"

// ___CENTERING MODEL:___

#define centeringModel true

// ___RESIZING MODEL:___

#define resizingModel false // [true/false]
                            // true --> resizes model (!!! only works with CAD Model !!!)
                            // false --> should be used when aufgenommene Models are used

// Object-Abmessungen
#define objectLenght 0.1238 // m
#define objectWidth 0.055   // m
#define objectHight 0.016   // m

#define resizingWithCalculatedRatio true // [true/false]
                                         // true  --> resizing factor for the object will be callculated based on Object-Abmessungen
                                         // false --> resizing factor is [scalingFactorForResizingObject]

#define scalingFactorForResizingObject 0.001 // resizing factor for manual resizing

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

// ___VOXEL GRID:___
#define withVoxelGrid false      // mit VoxelGrid funktioniert das garnicht !!!
#define voxelGridLeafSize 0.005f // --> 5 mm

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

    // resolution
    float resolution_1 = static_cast<float>(computeCloudResolution(model));
    std::cout << "Model resolution:       " << resolution_1 << std::endl;

    float resolution_1_scene = static_cast<float>(computeCloudResolution(scene));
    std::cout << "Scene resolution:       " << resolution_1_scene << std::endl;

    // visualizing input clouds
    comparisonViewer("Input Clouds", "Model", model, "Scene", scene);

    // =========================================================== Preparing Model ===========================================================

    // ---------------- centering the model ----------------

    if (centeringModel)
    {
        pcl::console::print_highlight("Centering Model...\n");

        Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
        for (const auto &p : *(model))
            sum_of_pos += p.getVector3fMap().cast<double>();

        Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
        transform_centering.topRightCorner<3, 1>() = -sum_of_pos / model->size();

        pcl::transformPointCloud(*model, *model, transform_centering);
        // turning 180° um y
        // pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 1, 0, 0));

        // turning 90° um z
        // pcl::transformPointCloud(*model, *model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071068, 0, 0, 0.7071068));

        // visualizing centered model
        littleViewer("centered model", model);
    }

    // -------------- moving model down --------------

    // for (auto &point : *(model))
    //{
    //     point.z -= 0.005;
    // }

    // ---------------- resizing the model ----------------

    if (resizingModel)
    {
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
    }

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

    // ----------------------- Downsampling with VoxelGrid -----------------------

    if (withVoxelGrid)
    {
        pcl::console::print_highlight("VoxelGrid...\n");

        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(voxelGridLeafSize, voxelGridLeafSize, voxelGridLeafSize); // Voxelgröße von 1 cm (0.01 Meter)
        voxel_grid.setInputCloud(model);
        voxel_grid.filter(*model);
        voxel_grid.setInputCloud(scene);
        voxel_grid.filter(*scene);

        // resolution
        resolution = static_cast<float>(computeCloudResolution(model));
        std::cout << "Model resolution:       " << resolution << std::endl;

        resolution_scene = static_cast<float>(computeCloudResolution(scene));
        std::cout << "Scene resolution:       " << resolution_scene << std::endl;

        // visualizing downsampled clouds
        comparisonViewer("Downsampled with VoxelGrid", "Model", model, "Scene", scene);
    }

    // ----------------------- Estimating Normals -----------------------

    pcl::console::print_highlight("Estimating Normals...\n");

    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch(pointNeighborsNormalEstimation); // 10
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

    pcl::console::print_highlight("Computing Model-Scene Correspondences...\n");

    pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(model_descriptors);

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_good_kp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_good_kp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*model_keypoints, model_good_keypoints_indices, *model_good_kp);
    pcl::copyPointCloud(*scene_keypoints, scene_good_keypoints_indices, *scene_good_kp);

    std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

    // ----------------------- Clustering -----------------------

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> allTransformations;
    std::vector<pcl::Correspondences> clustered_corrs;

    if (usingHough)
    {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
        pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

        pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
        rf_est.setFindHoles(true);
        rf_est.setRadiusSearch(referenceFrameRadius);

        rf_est.setInputCloud(model_keypoints);
        rf_est.setInputNormals(model_normals);
        rf_est.setSearchSurface(model);
        rf_est.compute(*model_rf);

        rf_est.setInputCloud(scene_keypoints);
        rf_est.setInputNormals(scene_normals);
        rf_est.setSearchSurface(scene);
        rf_est.compute(*scene_rf);

        //  Clustering
        pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
        clusterer.setHoughBinSize(clusteringSize);
        clusterer.setHoughThreshold(clusteringTH);
        clusterer.setUseInterpolation(true);
        clusterer.setUseDistanceWeight(false);

        clusterer.setInputCloud(model_keypoints);
        clusterer.setInputRf(model_rf);
        clusterer.setSceneCloud(scene_keypoints);
        clusterer.setSceneRf(scene_rf);
        clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // clusterer.cluster (clustered_corrs);
        clusterer.recognize(allTransformations, clustered_corrs);
    }
    else
    {
        pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
        gc_clusterer.setGCSize(clusteringSize);
        gc_clusterer.setGCThreshold(clusteringTH);

        gc_clusterer.setInputCloud(model_keypoints);
        gc_clusterer.setSceneCloud(scene_keypoints);
        gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

        // gc_clusterer.cluster (clustered_corrs);
        gc_clusterer.recognize(allTransformations, clustered_corrs);
    }

    // ----------------------- Stop if no instances where found -----------------------

    if (allTransformations.size() <= 0)
    {
        std::cout << " !!!!!!!!!! No instances where found !!!!!!!!!!" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Model instances found: " << allTransformations.size() << std::endl;
    }

    // ----------------------- Output after Clustering (printing allTransformations) -----------------------

    pcl::console::print_highlight("Outputting all instances...\n");

    for (int i = 0; i < allTransformations.size(); i++)
    {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

        // Print the rotation matrix and translation vector
        Eigen::Matrix3f rotation = allTransformations[i].block<3, 3>(0, 0);
        Eigen::Vector3f translation = allTransformations[i].block<3, 1>(0, 3);

        printf("\n");
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        printf("\n");
        printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
    }

    // filtering model with most correspondence
    int highestChecker = 0;
    int indexChecker = -1;
    for (int i = 0; i < allTransformations.size(); i++)
    {
        if (clustered_corrs[i].size() > highestChecker)
        {
            highestChecker = clustered_corrs[i].size();
            indexChecker = i;
        }
    }
    std::cout << "highestChecker = " << highestChecker << std::endl;
    std::cout << "indexChecker = " << indexChecker << std::endl;

    // ----------------------- Visualisierung von allen instances -----------------------

    pcl::console::print_highlight("Visualising Results of all found Instances...\n");

    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addCoordinateSystem(0.1);
    viewer.addPointCloud(scene, "scene_cloud");

    pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

    if (showingCorresopences)
    {
        //  We are translating the model so that it doesn't end in the middle of the scene representation
        pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 0, 0, 1));
        pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 0, 0, 1));

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> off_scene_model_color_handler(off_scene_model, 255, 255, 128); // yellow
        viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
    }

    for (int i = 0; i < allTransformations.size(); i++)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*model, *rotated_model, allTransformations[i]);

        std::stringstream ss_cloud;
        ss_cloud << "instance" << i;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_model_color_handler(rotated_model, 255, 0, 0); // red
        viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

        // show CoordinateSystem
        // converting Eigen::Matrix4f to Eigen::Affine3f
        Eigen::Affine3f transformationVisualization;
        transformationVisualization = allTransformations[i];

        std::stringstream ss_coordinatesystem;
        ss_coordinatesystem << "object_pose_" << i;

        viewer.addCoordinateSystem(0.1, transformationVisualization, ss_coordinatesystem.str(), 0);

        if (showingCorresopences)
        {
            for (int j = 0; j < clustered_corrs[i].size(); j++)
            {
                std::stringstream ss_line;
                ss_line << "correspondence_line" << i << "_" << j;

                pcl::PointXYZ &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
                pcl::PointXYZ &scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

                //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
                viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(model_point, scene_point, 0, 255, 0, ss_line.str());
            }
        }
    }

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    // ----------------------- Generating the clouds of the found instances -----------------------

    pcl::console::print_highlight("Transforming all instances...\n");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> instances;

    for (int i = 0; i < allTransformations.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*model, *transformed_model, allTransformations[i]);
        instances.push_back(transformed_model);
    }

    // ----------------------- ICP -----------------------

    pcl::console::print_highlight("ICP...\n");

    std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> registered_instances;
    std::vector<Eigen::Matrix4f> transformation_icp;
    std::vector<double> icp_scores;

    for (int i = 0; i < allTransformations.size(); i++)
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

    // ----------------------- Hypothesis Verification -----------------------

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

        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_hv(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr off_scene_model_keypoints_hv(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::PointCloud<pcl::PointXYZ>::Ptr off_model_good_kp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*model, *off_scene_model_hv, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 0, 0, 1));
        pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints_hv, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 0, 0, 1));
        pcl::transformPointCloud(*model_good_kp, *off_model_good_kp, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 0, 0, 1));

        for (int i = 0; i < instances.size(); i++)
        {
            std::stringstream ss_instance;
            ss_instance << "instance_" << i;

            // All instances in red
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> instance_color_handler(instances[i], 255, 0, 0); // red
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
        std::cout << "lowest ICP: " << min_ICPScore << " at " << index_lowestICPScore << std::endl;

        // ----------------------- Visualizing after Hypothesis Verification -----------------------

        pcl::visualization::PCLVisualizer viewer_hv("Hypotheses Verification");
        viewer_hv.addCoordinateSystem(0.1);

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
        ss_instance << "instance_" << index_lowestICPScore;
        ss_instance << "_registered" << std::endl;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> registered_instance_color_handler(registered_instances[index_lowestICPScore], 0, 255, 0); // green
        viewer_hv.addPointCloud(registered_instances[index_lowestICPScore], registered_instance_color_handler, ss_instance.str());

        while (!viewer_hv.wasStopped())
        {
            viewer_hv.spinOnce();
        }
    }

    return 0;
}