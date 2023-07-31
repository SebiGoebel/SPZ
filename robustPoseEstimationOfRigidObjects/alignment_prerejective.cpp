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

// Object-Abmessungen
#define objectLenght 0.1238 // m
#define objectWidth 0.055   // m
#define objectHight 0.016   // m

#define resizingWithCalculatedRatio true // [true/false]
                                         // true  --> resizing factor for the object will be callculated based on Object-Abmessungen
                                         // false --> resizing factor is [scalingFactorForResizingObject]

#define scalingFactorForResizingObject 0.001 // resizing factor for manual resizing

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

double computeCloudDiameter(const PointCloudT::ConstPtr &cloud, float leafSize)
{
  PointCloudT::Ptr cloud_downsampled(new PointCloudT());
  // pcl::VoxelGrid<pcl::PointXYZ> vg;
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
  // Point clouds
  PointCloudT::Ptr object(new PointCloudT);
  PointCloudT::Ptr object_aligned(new PointCloudT);
  PointCloudT::Ptr scene_before_downsampling(new PointCloudT);
  PointCloudT::Ptr scene(new PointCloudT);
  FeatureCloudT::Ptr object_features(new FeatureCloudT);
  FeatureCloudT::Ptr scene_features(new FeatureCloudT);

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

  // ================ visualizing input clouds ================

  pcl::console::print_highlight("Visualizing input clouds...\n");

  // visualize converted cloud
  pcl::visualization::PCLVisualizer viewer_input("PCL Viewer");
  viewer_input.initCameraParameters();
  viewer_input.setBackgroundColor(0.0, 0.0, 0.0);

  int v1(0);
  viewer_input.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer_input.setBackgroundColor(0, 0, 0, v1);
  viewer_input.addText("Object", 10, 10, "v1 text", v1);
  viewer_input.addPointCloud<PointNT>(object, "object_cloud", v1);

  int v2(0);
  viewer_input.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer_input.setBackgroundColor(0.3, 0.3, 0.3, v2);
  viewer_input.addText("Scene", 10, 10, "v2 text", v2);
  viewer_input.addPointCloud<PointNT>(scene_before_downsampling, "scene_cloud", v2);

  viewer_input.addCoordinateSystem(0.1);

  while (!viewer_input.wasStopped())
  {
    viewer_input.spinOnce();
  }

  // ================ centering the object ================

  pcl::console::print_highlight("Centering Object...\n");

  Eigen::Vector3d sum_of_pos = Eigen::Vector3d::Zero();
  for (const auto &p : *(object))
    sum_of_pos += p.getVector3fMap().cast<double>();

  Eigen::Matrix4d transform_centering = Eigen::Matrix4d::Identity();
  transform_centering.topRightCorner<3, 1>() = -sum_of_pos / object->size();

  pcl::transformPointCloud(*object, *object, transform_centering);
  // turning 90° um y
  // pcl::transformPointCloud(*object, *object, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(0.7071, 0, -0.7071, 0));

  // ================ resizing object ================

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
    std::cout << "Expected diameter: " << (diameterAbmessung-0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
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
    std::cout << "Expected diameter: " << (diameterAbmessung-0.003) << std::endl; // -0.003 wegen Abrundung in der Kurve die die Diagonale verringert
  }

  // ================ Downsample object and scene ================

  // Downsample
  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f; // tutorial: 0.005f
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  grid.setInputCloud(scene_before_downsampling);
  grid.filter(*scene);

  // visualize downsampled cloud
  pcl::visualization::PCLVisualizer viewer_downsampled("filtered Cloud with VoxelGrid");
  viewer_downsampled.setBackgroundColor(0.0, 0.0, 0.0);
  viewer_downsampled.addPointCloud<PointNT>(object);
  viewer_downsampled.addCoordinateSystem(0.1);
  viewer_downsampled.initCameraParameters();

  while (!viewer_downsampled.wasStopped())
  {
    viewer_downsampled.spinOnce();
  }

  // ================ Estimating normals ================

  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.005);
  nest.setInputCloud(scene);
  nest.setSearchSurface(scene_before_downsampling);
  nest.compute(*scene);

  // ================ Estimating features ================

  // Estimate features
  pcl::console::print_highlight("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch(0.025);
  fest.setInputCloud(object);
  fest.setInputNormals(object);
  fest.compute(*object_features);
  fest.setInputCloud(scene);
  fest.setInputNormals(scene);
  fest.compute(*scene_features);

  // Perform alignment
  pcl::console::print_highlight("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> align;
  align.setInputSource(object);
  align.setSourceFeatures(object_features);
  align.setInputTarget(scene);
  align.setTargetFeatures(scene_features);
  align.setMaximumIterations(50000);               // Number of RANSAC iterations
  align.setNumberOfSamples(3);                     // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5);            // Number of nearest features to use (when 5, 5 best matches and one is choosen ramdomly --> increases robustness against outlier matches)
  align.setSimilarityThreshold(0.95f);             // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
  align.setInlierFraction(0.25f);                  // Required inlier fraction for accepting a pose hypothesis
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

  return (0);
}
