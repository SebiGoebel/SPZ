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

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT, PointNT, FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

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

  // Downsample
  pcl::console::print_highlight("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize(leaf, leaf, leaf);
  grid.setInputCloud(object);
  grid.filter(*object);
  grid.setInputCloud(scene_before_downsampling);
  grid.filter(*scene);

  // Estimate normals for scene
  pcl::console::print_highlight("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT, PointNT> nest;
  nest.setRadiusSearch(0.005);
  nest.setInputCloud(scene);
  nest.setSearchSurface(scene_before_downsampling);
  nest.compute(*scene);

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
