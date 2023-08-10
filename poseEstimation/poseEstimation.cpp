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

// ICP
#include <pcl/registration/icp.h>

// ___INPUT:___

// #define filename_pcd_scene "../data/pointcloud_1_down_turned.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_down.pcd"
// #define filename_pcd_scene "../data/pointcloud_1_up.pcd"
#define filename_pcd_scene "../data/pointcloud_1_up.pcd"
// #define filename_pcd_scene "../data/pointcloud_2.pcd"
//  #define filename_pcd_scene "../data/pointcloud_3.pcd"
//  #define filename_pcd_scene "../data/pointcloud_6.pcd"
#define filename_pcd_model "../data/teil_aufgenommen_up.pcd"
// #define filename_pcd_model "../data/teil_leafSize5.pcd"

#define resizingModel false

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
#define voxelLeafSize 0.02f // Voxelgröße in m (Tutorial: 1 cm --> 0.01)
// 0.005f
// 0.0025f --> beste
// 0.02f --> beste, schnellste

//  ___NORMAL-ESTIMATION:___
#define normalNeigborRadius 0.05f // tutorial: 0.03 --> Use all neighbors in a sphere of radius 3cm
// 0.01f
// 0.005f --> beste
// 0.05f --> beste, schnellste

// ___HASH-SEARCH:___
#define hashDistanceStep 0.05f // the step value between each bin of the hash map for the distance values

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene_copy(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model_copy(new pcl::PointCloud<pcl::PointXYZ>);

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
    // turning 180° um y
    // pcl::transformPointCloud(*cloud_model, *cloud_model, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0, 1, 0, 0));

    // visualizing centered model
    littleViewer("centered model", cloud_model);

    // ---------------- resizing the model ----------------
    if (resizingModel)
    {

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
    }

    // Übernehmen der zentrierten und verkleinerten punktwolke
    pcl::copyPointCloud(*cloud_model, *cloud_model_copy);

    // Übernehmen der scene ohne veränderungen
    pcl::copyPointCloud(*cloud_scene, *cloud_scene_copy);

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
    pass.setFilterLimits(-0.15, 0.165);
    // pass.setNegative (true);
    pass.filter(*cloud_scene);

    // Y:
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.09, 0.1);
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

    // =========================================================== separate Preparation DONE ===========================================================

    // ----------------------- Downsampling Model and Scene with VoxelGrid -----------------------

    pcl::console::print_highlight("Downsampling with VoxelGrid...\n");

    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
    voxel_grid.setInputCloud(cloud_model);
    voxel_grid.filter(*cloud_model);
    voxel_grid.setInputCloud(cloud_scene);
    voxel_grid.filter(*cloud_scene);

    // visualizing the downsampled model and scene
    comparisonViewer("downsampled Clouds", "Model", cloud_model, "Scene", cloud_scene);

    // ----------------------- Normal Estimation Scene -----------------------

    pcl::console::print_highlight("Normal Estimation Scene...\n");

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    // Estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud_scene);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals(new pcl::search::KdTree<pcl::PointXYZ>());
    normal_estimation.setSearchMethod(tree_normals);
    normal_estimation.setRadiusSearch(normalNeigborRadius);

    // Compute the normals features
    normal_estimation.compute(*normals);

    // combining normal-feature cloud with point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*cloud_scene, *normals, *scene_cloud_with_normals);

    // -------------- checking normals and visualizing them --------------

    // normals->size () should have the same size as the input cloud->size ()
    if (cloud_scene->size() == normals->size())
    {
        std::cout << "normals_size correct !!!" << std::endl;
        std::cout << "cloud_scene_size = normals_size" << std::endl;
    }
    else
    {
        std::cout << "normals_size NOT correct !!!" << std::endl;
        std::cout << "cloud_scene_size != normals_size sollten aber gleich sein" << std::endl;
        std::cout << "Normals_Size: " << normals->size() << std::endl;
        std::cout << "cloud_scene_size:   " << cloud_scene->size() << std::endl;
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
    std::cout << "Anz. der Normalen: " << normals->size() << std::endl;

    // visualize cloud with normals
    littleViewerPointNormal("normal estimation scene", scene_cloud_with_normals);

    /*
        // visualize normals
        pcl::visualization::PCLVisualizer viewer_test("normal estimation");
        viewer_test.setBackgroundColor(0.0, 0.0, 0.5);
        viewer_test.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_scene, normals);
        // viewer.addCoordinateSystem (0.1);
        viewer_test.initCameraParameters();

        while (!viewer_test.wasStopped())
        {
            viewer_test.spinOnce();
        }
    */

    // ----------------------- Normal Estimation Model (with the same parameters) -----------------------

    pcl::console::print_highlight("Normal Estimation Model...\n");

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr normals_model(new pcl::PointCloud<pcl::Normal>);

    // Estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_model;
    normal_estimation_model.setInputCloud(cloud_model);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_normals_model(new pcl::search::KdTree<pcl::PointXYZ>());
    normal_estimation_model.setSearchMethod(tree_normals_model);
    normal_estimation_model.setRadiusSearch(normalNeigborRadius);

    // Compute the normals features
    normal_estimation_model.compute(*normals_model);

    // combining normal-feature cloud with point cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr model_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    concatenateFields(*cloud_model, *normals_model, *model_cloud_with_normals);

    // -------------- checking normals and visualizing them --------------

    // normals->size () should have the same size as the input cloud->size ()
    if (cloud_model->size() == normals_model->size())
    {
        std::cout << "normals_size correct !!!" << std::endl;
        std::cout << "cloud_model_size = normals_size" << std::endl;
    }
    else
    {
        std::cout << "normals_size NOT correct !!!" << std::endl;
        std::cout << "cloud_model_size != normals_size sollten aber gleich sein" << std::endl;
        std::cout << "Normals_Size: " << normals_model->size() << std::endl;
        std::cout << "cloud_model_size:   " << cloud_model->size() << std::endl;
    }

    // check if normals are finite (not NaN or infinite)
    int notFiniteNormalsCounter_model = 0;
    for (int i = 0; i < normals_model->size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>((*normals_model)[i]))
        {
            // PCL_WARN("normals[%d] is not finite\n", i);
            notFiniteNormalsCounter_model++;
        }
    }

    std::cout << "Anz. der nicht finiten Normalen: " << notFiniteNormalsCounter_model << std::endl;
    std::cout << "Anz. der Normalen: " << normals_model->size() << std::endl;

    // visualize cloud with normals
    littleViewerPointNormal("normal estimation model", model_cloud_with_normals);

    /*
        // visualize normals
        pcl::visualization::PCLVisualizer viewer_test("normal estimation");
        viewer_test.setBackgroundColor(0.0, 0.0, 0.5);
        viewer_test.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_model, normals_model);
        // viewer.addCoordinateSystem (0.1);
        viewer_test.initCameraParameters();

        while (!viewer_test.wasStopped())
        {
            viewer_test.spinOnce();
        }
    */

    // =========================================================== Downsampling and Normal Estimation DONE - starting with PPF ===========================================================

    // -------------- Extracting Point Pair Features --------------

    pcl::console::print_highlight("Extracting PPFs from model ...\n");

    pcl::PointCloud<pcl::PPFSignature>::Ptr cloud_model_ppf(new pcl::PointCloud<pcl::PPFSignature>());
    pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
    ppf_estimator.setInputCloud(model_cloud_with_normals);
    ppf_estimator.setInputNormals(model_cloud_with_normals);
    ppf_estimator.compute(*cloud_model_ppf);

    pcl::console::print_highlight("Creating Hashmap_search ...\n");

    // pcl::PPFHashMapSearch::Ptr hashmap_search(new pcl::PPFHashMapSearch(12.0f / 180.0f * float(M_PI), 0.05f));
    // pcl::PPFHashMapSearch::Ptr hashmap_search(new pcl::PPFHashMapSearch(12.0f / 180.0f * float(M_PI), 0.01f)); // default values
    pcl::PPFHashMapSearch::Ptr hashmap_search(new pcl::PPFHashMapSearch(12.0f / 180.0f * float(M_PI), hashDistanceStep));
    hashmap_search->setInputFeatureCloud(cloud_model_ppf);

    // -------------- Registering model to scene --------------

    pcl::console::print_highlight("Registering models to scene ...\n");

    pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
    ppf_registration.setSceneReferencePointSamplingRate(10);                       // sampling rate for the scene reference point
    ppf_registration.setPositionClusteringThreshold(0.02f);                        // distance threshold below which two poses are considered close enough to be in the same cluster (for the clustering phase of the algorithm)
    ppf_registration.setRotationClusteringThreshold(15.0f / 180.0f * float(M_PI)); // rotation difference threshold below which two poses are considered to be in the same cluster (for the clustering phase of the algorithm)
    ppf_registration.setSearchMethod(hashmap_search);
    ppf_registration.setInputSource(model_cloud_with_normals);
    ppf_registration.setInputTarget(scene_cloud_with_normals);

    // aligning
    pcl::PointCloud<pcl::PointNormal> cloud_output;
    ppf_registration.align(cloud_output);
    /*
        // converting PointNormal-Cloud -> PointXYZ-Cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        for (const auto &point : cloud_output.points)
        {
            cloud_output_xyz->points.emplace_back(point.x, point.y, point.z);
        }
    */
    Eigen::Matrix4f transformation = ppf_registration.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");

    // converting from Matrix4f -> Affine3f
    Eigen::Affine3f final_transformation(transformation);

    // transforming Model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_model, *cloud_transformed, final_transformation);

    // transforming Model Copy for Visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_copy(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_model_copy, *cloud_transformed_copy, final_transformation);

    // Visualisation
    pcl::visualization::PCLVisualizer viewer_result("PPF Object Recognition - Results");
    viewer_result.setBackgroundColor(0, 0, 0);
    viewer_result.addCoordinateSystem(0.1);
    viewer_result.initCameraParameters();
    viewer_result.addPointCloud(cloud_scene_copy, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene_copy, 0.0, 255.0, 0.0), "scene");
    viewer_result.addPointCloud(cloud_transformed_copy, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_transformed_copy, 0.0, 0.0, 255.0), "model_transformed");

    pcl::console::print_highlight("All models have been registered!\n");

    while (!viewer_result.wasStopped())
    {
        viewer_result.spinOnce();
    }

    //////////////////////////////////

    // Visualiser
    pcl::visualization::PCLVisualizer viewer_resultVergleicher("PPF Object Recognition - Results");
    viewer_resultVergleicher.addCoordinateSystem(0.1);
    viewer_resultVergleicher.initCameraParameters();

    int v1(0);
    viewer_resultVergleicher.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer_resultVergleicher.setBackgroundColor(0, 0, 0, v1);
    viewer_resultVergleicher.addText("Original PointClouds", 10, 10, "v1 text", v1);
    viewer_resultVergleicher.addPointCloud(cloud_scene_copy, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene_copy, 0.0, 255.0, 0.0), "scene_vergleich");
    viewer_resultVergleicher.addPointCloud(cloud_transformed_copy, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_transformed_copy, 255.0, 0.0, 0.0), "model_transformed_vergleich");

    int v2(0);
    viewer_resultVergleicher.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer_resultVergleicher.setBackgroundColor(0, 0, 0, v2);
    viewer_resultVergleicher.addText("prepared PointClouds", 10, 10, "v2 text", v2);
    viewer_resultVergleicher.addPointCloud(cloud_scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene, 0.0, 255.0, 0.0), "scene_prepared_vergleich");
    viewer_resultVergleicher.addPointCloud(cloud_transformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_transformed, 255.0, 0.0, 0.0), "model_transformed_prepared_vergleich");

    while (!viewer_resultVergleicher.wasStopped())
    {
        viewer_resultVergleicher.spinOnce();
    }

    // ==================================================== ICP after Matching ====================================================

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_transformed);
    icp.setInputTarget(cloud_scene);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(2.5f * voxelLeafSize);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(150);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon(1);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.hasConverged())
    {
        std::cout << "Aligned!" << std::endl;
    }
    else
    {
        std::cout << "Not Aligned!" << std::endl;
    }

    Eigen::Matrix4f transformation_icp = icp.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation_icp(0, 0), transformation_icp(0, 1), transformation_icp(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation_icp(1, 0), transformation_icp(1, 1), transformation_icp(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation_icp(2, 0), transformation_icp(2, 1), transformation_icp(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation_icp(0, 3), transformation_icp(1, 3), transformation_icp(2, 3));
    pcl::console::print_info("\n");

    // converting from Matrix4f -> Affine3f
    Eigen::Affine3f final_transformation_icp(transformation_icp);

    // transforming Model
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_icp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_transformed, *cloud_transformed_icp, final_transformation_icp);

    // transforming Model Copy for Visualization
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_copy_icp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud_transformed_copy, *cloud_transformed_copy_icp, final_transformation_icp);

    //////////////////////////////////

    // Visualiser
    pcl::visualization::PCLVisualizer viewer_resultVergleicher_icp("PPF Object Recognition - Results - After ICP");
    viewer_resultVergleicher_icp.addCoordinateSystem(0.1);
    viewer_resultVergleicher_icp.initCameraParameters();
    viewer_resultVergleicher_icp.registerKeyboardCallback(keyboardEventOccurred, (void *)&viewer_resultVergleicher_icp);

    int v1_icp(0);
    viewer_resultVergleicher_icp.createViewPort(0.0, 0.0, 0.5, 1.0, v1_icp);
    viewer_resultVergleicher_icp.setBackgroundColor(0, 0, 0, v1_icp);
    viewer_resultVergleicher_icp.addText("Original PointClouds", 10, 10, "v1_icp text", v1_icp);
    viewer_resultVergleicher_icp.addPointCloud(cloud_scene_copy, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene_copy, 0.0, 255.0, 0.0), "scene_vergleich_icp");
    viewer_resultVergleicher_icp.addPointCloud(cloud_transformed_copy_icp, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_transformed_copy_icp, 255.0, 0.0, 0.0), "model_transformed_vergleich_icp");

    int v2_icp(0);
    viewer_resultVergleicher_icp.createViewPort(0.5, 0.0, 1.0, 1.0, v2_icp);
    viewer_resultVergleicher_icp.setBackgroundColor(0, 0, 0, v2_icp);
    viewer_resultVergleicher_icp.addText("prepared PointClouds", 10, 10, "v2_icp text", v2_icp);
    viewer_resultVergleicher_icp.addPointCloud(cloud_scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_scene, 0.0, 255.0, 0.0), "scene_prepared_vergleich_icp");
    viewer_resultVergleicher_icp.addPointCloud(cloud_transformed_icp, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_transformed_icp, 255.0, 0.0, 0.0), "model_transformed_prepared_vergleich_icp");

    while (!viewer_resultVergleicher_icp.wasStopped())
    {
        viewer_resultVergleicher_icp.spinOnce();
        
        if (spaceKeyPressed)
        {
            if (!coordinateSystemsAreShown)
            {
                // show CoordinateSystem
                // converting Eigen::Matrix4f to Eigen::Affine3f
                Eigen::Affine3f transformationVisualization_icp;
                transformationVisualization_icp = transformation_icp;

                Eigen::Affine3f transformationVisualisation;
                transformationVisualisation = transformation;

                Eigen::Affine3f combinedTransformation = transformationVisualization_icp * transformationVisualisation;

                // addCoordinateSystem(double scale, Eigen::Affine3f& tranform, std::string& id, int viewport)
                viewer_resultVergleicher_icp.addCoordinateSystem(0.1, combinedTransformation, "v1_icp object_pose", v1_icp);
                viewer_resultVergleicher_icp.addCoordinateSystem(0.1, "v1_icp camera_pose", v1_icp);

                viewer_resultVergleicher_icp.addCoordinateSystem(0.1, combinedTransformation, "v2_icp object_pose", v2_icp);
                viewer_resultVergleicher_icp.addCoordinateSystem(0.1, "v2_icp camera_pose", v2_icp);

                // added Text
                // addText3D(std::string& text, Point& position, double scale, double r, double g, double b, std::string& id, int viewport)
                pcl::PointXYZ origin_camera;
                origin_camera.x = 0.0 - 0.01;
                origin_camera.y = 0.0 + 0.02;
                origin_camera.z = 0.0;
                // double orientation_camera[3];
                // orientation_camera[0] = 0.0;
                // orientation_camera[1] = 0.0;
                // orientation_camera[2] = 0.0;
                viewer_resultVergleicher_icp.addText3D("Camera", origin_camera, 0.01, 1.0, 1.0, 1.0, "v1_icp camera_text", v1_icp);
                viewer_resultVergleicher_icp.addText3D("Camera", origin_camera, 0.01, 1.0, 1.0, 1.0, "v2_icp camera_text", v2_icp);

                pcl::PointXYZ origin_object;
                origin_object.x = combinedTransformation(0, 3) - 0.01;
                origin_object.y = combinedTransformation(1, 3) + 0.02;
                origin_object.z = combinedTransformation(2, 3);
                viewer_resultVergleicher_icp.addText3D("Object", origin_object, 0.01, 1.0, 1.0, 1.0, "v1_icp object_text", v1_icp);
                viewer_resultVergleicher_icp.addText3D("Object", origin_object, 0.01, 1.0, 1.0, 1.0, "v2_icp object_text", v2_icp);

                coordinateSystemsAreShown = true;
                std::cout << "=> added Coordinatesystems" << std::endl;
            }
            else
            {
                viewer_resultVergleicher_icp.removeText3D("v1_icp camera_text", v1_icp);
                viewer_resultVergleicher_icp.removeText3D("v1_icp object_text", v1_icp);

                viewer_resultVergleicher_icp.removeText3D("v2_icp camera_text", v2_icp);
                viewer_resultVergleicher_icp.removeText3D("v2_icp object_text", v2_icp);

                viewer_resultVergleicher_icp.removeAllCoordinateSystems();
                coordinateSystemsAreShown = false;
                std::cout << "=> removed Coordinatesystems" << std::endl;
            }

            spaceKeyPressed = false;
        }
        
    }

    return 0;
}