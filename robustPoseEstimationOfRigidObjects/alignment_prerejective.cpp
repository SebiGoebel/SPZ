#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>

// Plane Segmentation for Removing
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// Extracting Indices
#include <pcl/filters/extract_indices.h>

// pass through filter
#include <pcl/filters/passthrough.h>

// ICP
#include <pcl/registration/icp.h>

// Object-Abmessungen
#define objectLenght 0.1238 // m
#define objectWidth 0.055   // m
#define objectHight 0.016   // m

#define resizingModel true

#define resizingWithCalculatedRatio true // [true/false]
                                         // true  --> resizing factor for the object will be callculated based on Object-Abmessungen
                                         // false --> resizing factor is [scalingFactorForResizingObject]

#define scalingFactorForResizingObject 0.001 // resizing factor for manual resizing

// ___VOXELGRID:___
#define voxelLeafSize 0.005f // Voxelgröße in m (Tutorial: 5 mm --> 0.005)
// 0.02

//  ___NORMAL-ESTIMATION:___
#define normalNeigborRadius 0.005f // Tutorial: 0.005 --> Use all neighbors in a sphere of radius 5mm
// 0.05

// ___FPFH-ESTIMATION:___
#define searchRadiusFPFH 0.025f // Tutorial: 0.025
// 0.05

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// ========================================= computing Diameters =========================================

double computeCloudDiameter(const PointCloudT::ConstPtr &cloud, float leafSize)
{
  PointCloudT::Ptr cloud_downsampled(new PointCloudT());
  pcl::VoxelGrid<PointNT> vg;
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

// ========================================= Viwers =========================================

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

void littleViewerPointNormal(const char *windowTitle, const PointCloudT::ConstPtr &cloud)
{
  // Terminal message
  pcl::console::print_highlight("Visualizing ");
  std::cout << windowTitle << "..." << std::endl;

  // Visualiser
  pcl::visualization::PCLVisualizer viewer(windowTitle);
  viewer.setBackgroundColor(0.0, 0.0, 0.0);
  viewer.addCoordinateSystem(0.1);
  viewer.initCameraParameters();
  viewer.addPointCloud<PointNT>(cloud);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
}

void comparisonViewer(const char *windowTitle, std::string text1, const PointCloudT::ConstPtr &cloud1, std::string text2, const PointCloudT::ConstPtr &cloud2)
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
  viewer.addPointCloud<PointNT>(cloud1, "cloud1", v1);

  int v2(0);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer.addText(text2, 10, 10, "v2 text", v2);
  viewer.addPointCloud<PointNT>(cloud2, "cloud2", v2);

  viewer.addCoordinateSystem(0.1);

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

// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char **argv)
{
  // ================ loading Pointclouds ================

  // Point clouds
  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  PointCloudT::Ptr scene_before_downsampling(new PointCloudT);
  PointCloudT::Ptr scene(new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);

  PointCloudT::Ptr scene_copy(new PointCloudT);
  PointCloudT::Ptr object_copy(new PointCloudT);

  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT>(argv[1], *object) < 0 ||
      pcl::io::loadPCDFile<PointNT>(argv[2], *scene_before_downsampling) < 0)
  {
    pcl::console::print_error("Error loading object/scene file!\n");
    return (1);
  }

  // visualizing input clouds
  comparisonViewer("Input Clouds", "Model", object, "Scene", scene_before_downsampling);

  // =========================================================== Preparing Model ===========================================================

  // ---------------- centering the model ----------------

  pcl::console::print_highlight("Centering Object...\n");

  Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
  for (const auto &p : *(object))
    sum_of_pos += p.getVector3fMap().cast<double>();

  Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
  transform_centering.topRightCorner<3, 1>() = -sum_of_pos / object->size();

  pcl::transformPointCloud(*object, *object, transform_centering);
  // turning 90° um y
  // pcl::transformPointCloud(*object, *object, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));

  // visualizing centered model
  littleViewerPointNormal("centered model", object);

  // ---------------- resizing the model ----------------

  if (resizingModel)
  {

    if (resizingWithCalculatedRatio)
    {
      pcl::console::print_highlight("Calculating Scalingfactor...\n");

      // computing the cloud diameter from the object
      double diameter_beforeResizing = computeCloudDiameter(object, 1.0);
      std::cout << "Object cloud diameter: " << diameter_beforeResizing << std::endl;

      // computing the diameter from the real object using abmessungen
      double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
      std::cout << "Object abmessungen diameter: " << diameterAbmessung << std::endl;

      // calculating the scaling ratio
      double scalingRatio = diameterAbmessung / diameter_beforeResizing;
      pcl::console::print_highlight("Scalingfactor: %f\n", scalingRatio);

      // actual resizing
      pcl::console::print_highlight("Resizing Object...\n");

      for (auto &point : *(object))
      {
        point.x *= scalingRatio;
        point.y *= scalingRatio;
        point.z *= scalingRatio;
      }

      // computing the diameter for checking
      double diameterCheck_cloud = computeCloudDiameter(object, 0.005);
      std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
      std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }
    else
    {
      pcl::console::print_highlight("Manual Scalingfactor: %f \n", scalingFactorForResizingObject);

      // resizing with manually set factor
      pcl::console::print_highlight("Resizing Object...\n");

      for (auto &point : *(object))
      {
        point.x *= scalingFactorForResizingObject;
        point.y *= scalingFactorForResizingObject;
        point.z *= scalingFactorForResizingObject;
      }

      // computing the diameter for checking
      double diameterCheck_cloud = computeCloudDiameter(object, 0.005);
      std::cout << "Diameter check from cloud: " << diameterCheck_cloud << std::endl;
      double diameterAbmessung = computeObjectDiameter(objectLenght, objectWidth, objectHight);
      std::cout << "Expected diameter: " << (diameterAbmessung - 0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
    }

    // visualizing resized model
    littleViewerPointNormal("resized model", object);
  }

  // Übernehmen der zentrierten und verkleinerten punktwolke
  pcl::copyPointCloud(*object, *object_copy);

  // Übernehmen der scene ohne veränderungen
  pcl::copyPointCloud(*scene_before_downsampling, *scene_copy);

  // =========================================================== Preparing Scene ===========================================================

  // visualizing resized model
  littleViewerPointNormal("Input cloud", scene_before_downsampling);

  // ---------------- removing plane ----------------

  pcl::SACSegmentation<PointNT> seg;
  pcl::ExtractIndices<PointNT> extract_plane;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.0025);
  extract_plane.setNegative(true);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
  const auto nr_points = scene_before_downsampling->size();

  while (scene_before_downsampling->size() > 0.3 * nr_points)
  {
    seg.setInputCloud(scene_before_downsampling);
    seg.segment(*inliers_plane, *coefficients_plane);
    PCL_INFO("inliers_plane: %zu\n", static_cast<std::size_t>(inliers_plane->indices.size()));
    if (inliers_plane->indices.size() < 50000)
      break;

    extract_plane.setInputCloud(scene_before_downsampling);
    extract_plane.setIndices(inliers_plane);
    extract_plane.filter(*scene_before_downsampling);
  }

  // visualizing cloud after plane removing
  littleViewerPointNormal("plane removed", scene_before_downsampling);

  // ---------------- removing walls ----------------

  pcl::PassThrough<PointNT> pass;
  pass.setInputCloud(scene_before_downsampling);

  // X:
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.15, 0.165);
  // pass.setNegative (true);
  pass.filter(*scene_before_downsampling);

  // Y:
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.09, 0.1);
  // pass.setNegative (true);
  pass.filter(*scene_before_downsampling);

  // visualizing cloud after wall removing
  littleViewerPointNormal("walls removed", scene_before_downsampling);

  // ---------------- Visualizing ----------------

  // visualizing the prepared model and scene
  comparisonViewer("prepared Clouds", "Model", object, "Scene", scene_before_downsampling);

  // =========================================================== separate Preparation DONE ===========================================================

  // ----------------------- Downsampling Object and Scene with VoxelGrid -----------------------

  pcl::console::print_highlight("Downsampling with VoxelGrid...\n");

  pcl::VoxelGrid<PointNT> voxel_grid;
  voxel_grid.setLeafSize(voxelLeafSize, voxelLeafSize, voxelLeafSize);
  voxel_grid.setInputCloud(object);
  voxel_grid.filter(*object);
  voxel_grid.setInputCloud(scene_before_downsampling);
  voxel_grid.filter(*scene);

  // visualizing the downsampled model and scene
  comparisonViewer("downsampled Clouds", "Model", object, "Scene", scene);

  // ================ Estimating normals ================

  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(normalNeigborRadius);
  nest.setInputCloud(scene);
  nest.setSearchSurface(scene_before_downsampling);
  nest.compute(*scene);

  // visualizing the downsampled model and scene
  comparisonViewer("normal estimation", "Model", object, "Scene", scene);

  // =========================================================== Downsampling and Normal Estimation DONE - starting with Features ===========================================================

  // -------------- Extracting FPFHs --------------

  pcl::console::print_highlight("Estimating features...\n");

  FeatureEstimationT fest;
  fest.setRadiusSearch(searchRadiusFPFH);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);

  // -------------- Perform alignment --------------

  pcl::console::print_highlight("Starting alignment...\n");

  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(50000);                        // Number of RANSAC iterations
  align.setNumberOfSamples(3);                              // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5);                     // Number of nearest features to use (when 5, 5 best matches and one is choosen ramdomly --> increases robustness against outlier matches)
  align.setSimilarityThreshold(0.80f);                      // 0.95       // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * voxelLeafSize); // Inlier threshold
  align.setInlierFraction(0.25f);                           // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align(*object_aligned);
  }

  if (align.hasConverged())
  {
    // Print results
    printf("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation();
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
    pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
    pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.initCameraParameters();
    visu.registerKeyboardCallback(keyboardEventOccurred, (void *)&visu);

    while (!visu.wasStopped())
    {
      visu.spinOnce();

      if (spaceKeyPressed)
      {
        if (!coordinateSystemsAreShown)
        {
          // show CoordinateSystem
          // converting Eigen::Matrix4f to Eigen::Affine3f
          Eigen::Affine3f transformationVisualization;
          transformationVisualization = transformation;

          // addCoordinateSystem(double scale, Eigen::Affine3f& tranform, std::string& id, int viewport)
          visu.addCoordinateSystem(0.1, transformationVisualization, "object_pose", 0);
          visu.addCoordinateSystem(0.1, "camera_pose", 0);

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
          visu.addText3D("Camera", origin_camera, 0.01, 1.0, 1.0, 1.0, "camera_text", 0);

          pcl::PointXYZ origin_object;
          origin_object.x = transformation(0, 3) - 0.01;
          origin_object.y = transformation(1, 3) + 0.02;
          origin_object.z = transformation(2, 3);
          visu.addText3D("Object", origin_object, 0.01, 1.0, 1.0, 1.0, "object_text", 0);

          coordinateSystemsAreShown = true;
          std::cout << "=> added Coordinatesystems" << std::endl;
        }
        else
        {
          visu.removeText3D("camera_text", 0);
          visu.removeText3D("object_text", 0);
          visu.removeAllCoordinateSystems();
          coordinateSystemsAreShown = false;
          std::cout << "=> removed Coordinatesystems" << std::endl;
        }

        spaceKeyPressed = false;
      }
    }
  }
  else
  {
    pcl::console::print_error("Alignment failed!\n");
    return (1);
  }

  // ==================================================== ICP after Matching ====================================================

  pcl::IterativeClosestPoint<PointNT, PointNT> icp;
  icp.setInputSource(object_aligned);
  icp.setInputTarget(scene);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(2.5f * voxelLeafSize);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(150);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(1);

  pcl::PointCloud<PointNT> Final;
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
  PointCloudT::Ptr object_transformed_icp(new PointCloudT);
  pcl::transformPointCloud(*object_aligned, *object_transformed_icp, final_transformation_icp);

  // transforming Model Copy for Visualization
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_copy_icp(new pcl::PointCloud<pcl::PointXYZ>());
  // pcl::transformPointCloud(*cloud_transformed_copy, *cloud_transformed_copy_icp, final_transformation_icp);

  // Show alignment after icp
  pcl::visualization::PCLVisualizer visu_icp("Alignment ICP");
  visu_icp.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene_icp");
  visu_icp.addPointCloud(object_transformed_icp, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned_icp");
  visu_icp.initCameraParameters();
  visu_icp.registerKeyboardCallback(keyboardEventOccurred, (void *)&visu_icp);

  while (!visu_icp.wasStopped())
  {
    visu_icp.spinOnce();
  }

  return (0);
}
