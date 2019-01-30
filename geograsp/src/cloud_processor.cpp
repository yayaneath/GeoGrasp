#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/passthrough.h>

#include <pcl/features/normal_3d_omp.h>

#include <boost/chrono/thread_clock.hpp>

#include <geograsp/GeoGrasp.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));
int frames = 0;
double accDuration = 0.0;


// callback signature
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & inputCloudMsg) {
  boost::chrono::thread_clock::time_point start = boost::chrono::thread_clock::now();
  frames++;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloudMsg, *cloud);

  // Remove NaN values and make it dense
  std::vector<int> nanIndices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

  /*std::cout << "### ORGINAL CLOUD WIDTH:" << cloud->width << "\n";
  std::cout << "### ORGINAL CLOUD HEIGHT:" << cloud->height << "\n";
  std::cout << "### ORGINAL CLOUD POINTS:" << cloud->points.size() << "\n";
  std::cout << "### ORGINAL CLOUD DENSITY:" << cloud->is_dense << "\n";
  std::cout << "### ORGINAL CLOUD ORGANIZATION:" << cloud->isOrganized() << "\n";
  std::cout << "### ORGINAL CLOUD ORIGIN:" << cloud->sensor_origin_[0] << "\n";
  std::cout << "### ORGINAL CLOUD ORIGIN:" << cloud->sensor_origin_[1] << "\n";
  std::cout << "### ORGINAL CLOUD ORIGIN:" << cloud->sensor_origin_[2] << "\n";
  std::cout << "### ORGINAL CLOUD ORIGIN:" << cloud->sensor_origin_[3] << "\n";
  std::cout << "### ORGINAL CLOUD ORIENTATION:" << cloud->sensor_orientation_.x() << "\n";
  std::cout << "### ORGINAL CLOUD ORIENTATION:" << cloud->sensor_orientation_.y() << "\n";
  std::cout << "### ORGINAL CLOUD ORIENTATION:" << cloud->sensor_orientation_.z() << "\n";
  std::cout << "### ORGINAL CLOUD ORIENTATION:" << cloud->sensor_orientation_.w() << "\n";

  float minX = std::numeric_limits<float>::max(), maxX = -std::numeric_limits<float>::max();
  float minY = std::numeric_limits<float>::max(), maxY = -std::numeric_limits<float>::max();
  float minZ = std::numeric_limits<float>::max(), maxZ = -std::numeric_limits<float>::max();

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    if (cloud->points[i].x < minX) minX = cloud->points[i].x;
    if (cloud->points[i].x > maxX) maxX = cloud->points[i].x;

    if (cloud->points[i].y < minY) minY = cloud->points[i].y;
    if (cloud->points[i].y > maxY) maxY = cloud->points[i].y;

    if (cloud->points[i].z < minZ) minZ = cloud->points[i].z;
    if (cloud->points[i].z > maxZ) maxZ = cloud->points[i].z;
  }

  std::cout << "### ORGINAL CLOUD X LIMITS:" << minX << ", " << maxX << "\n";
  std::cout << "### ORGINAL CLOUD Y LIMITS:" << minY << ", " << maxY << "\n";
  std::cout << "### ORGINAL CLOUD Z LIMITS:" << minZ << ", " << maxZ << "\n"; */

  // Save to file
  pcl::io::savePCDFileBinary("1-cloud.pcd", *cloud);

  // Remove background points
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  ptFilter.setFilterLimits(0.0, 1.3);
  ptFilter.filter(*cloud);

  pcl::io::savePCDFileBinary("2-cloud-z-filtered.pcd", *cloud);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> sacSegmentator;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>());

  sacSegmentator.setModelType(pcl::SACMODEL_PLANE);
  sacSegmentator.setMethodType(pcl::SAC_RANSAC);
  sacSegmentator.setMaxIterations(50);
  sacSegmentator.setDistanceThreshold(0.01);
  sacSegmentator.setInputCloud(cloud);
  sacSegmentator.segment(*inliers, *coefficients);

  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
  indExtractor.setInputCloud(cloud);
  indExtractor.setIndices(inliers);
  indExtractor.setNegative(false);

  // Get the points associated with the planar surface
  indExtractor.filter(*cloudPlane);

  pcl::io::savePCDFileBinary("3-cloud-plane.pcd", *cloudPlane);

  // Remove the planar inliers, extract the rest
  indExtractor.setNegative(true);
  indExtractor.filter(*cloud);

  pcl::io::savePCDFileBinary("3-cloud-objects.pcd", *cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
  ecExtractor.setClusterTolerance(0.01);
  ecExtractor.setMinClusterSize(750);
  //ecExtractor.setMaxClusterSize(25000);
  ecExtractor.setSearchMethod(tree);
  ecExtractor.setInputCloud(cloud);
  ecExtractor.extract(clusterIndices);

  if (clusterIndices.empty()) {
    // Visualize the result
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane, 0, 255, 0);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Main cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, "Plane");

    viewer->spinOnce();
  }
  else {
    std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
    int objectNumber = 0;

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (std::vector<int>::const_iterator pit = it->indices.begin(); 
          pit != it->indices.end(); ++pit)
        objectCloud->points.push_back(cloud->points[*pit]);

      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      GeoGrasp geoGraspPoints;
      geoGraspPoints.setBackgroundCloud(cloudPlane);
      geoGraspPoints.setObjectCloud(objectCloud);

      boost::chrono::thread_clock::time_point objStart = boost::chrono::thread_clock::now();
      geoGraspPoints.compute();
      boost::chrono::thread_clock::time_point objStop = boost::chrono::thread_clock::now();

      std::cout << "Elapsed time: " 
        << boost::chrono::duration_cast<boost::chrono::milliseconds>(objStop-objStart).count() << "ms\n";

      GraspConfiguration bestGrasp = geoGraspPoints.getBestGrasp();

      // Visualize the result
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(objectCloud);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane, 
        0, 155, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planeRGB(cloudPlane);

      std::string objectLabel = "";
      std::ostringstream converter;

      converter << objectNumber;
      objectLabel += converter.str();
      objectLabel += "-";

      pcl::io::savePCDFileBinary("4-" + objectLabel + "object.pcd", *objectCloud);

      viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, rgb, objectLabel + "Object");
      viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeRGB, objectLabel + "Plane");

      /////////////////////////////////////////////////////////////////////////////////////////////////////////

      /*viewer->addSphere(geoGraspPoints.getObjectCentroidPoint(), 0.01, 255, 69, 0,
        objectLabel + "Object centroid");
      viewer->addLine(geoGraspPoints.getObjectAxisCoeff(), objectLabel + "Object axis");

      pcl::PointCloud<pcl::PointNormal>::Ptr graspPlaneCloud(new pcl::PointCloud<pcl::PointNormal>());
      *graspPlaneCloud = geoGraspPoints.getGraspPlaneCloud();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> graspCloudColorHandler(
        graspPlaneCloud, 0, 255, 0);
      viewer->addPointCloud<pcl::PointNormal>(graspPlaneCloud, graspCloudColorHandler, 
        objectLabel + "Grasp plane cloud");*/

      /////////////////////////////////////////////////////////////////////////////////////////////////////////

      /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr firstPointRadiusCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      *firstPointRadiusCloud = geoGraspPoints.getFirstPointRadiusCloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr secondPointRadiusCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      *secondPointRadiusCloud = geoGraspPoints.getSecondPointRadiusCloud();

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> 
        graspPointFirstRadiusCloudColorHandler(firstPointRadiusCloud, 255, 100, 100);
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> 
        graspPointSecondRadiusCloudColorHandler(secondPointRadiusCloud, 255, 100, 100);

      viewer->addPointCloud<pcl::PointXYZRGB>(firstPointRadiusCloud, graspPointFirstRadiusCloudColorHandler, 
                                          objectLabel + "First point radius cloud");
      viewer->addPointCloud<pcl::PointXYZRGB>(secondPointRadiusCloud, graspPointSecondRadiusCloudColorHandler, 
                                          objectLabel + "Second point radius cloud");*/

      /////////////////////////////////////////////////////////////////////////////////////////////////////////

      /*pcl::PointCloud<pcl::PointNormal>::Ptr firstPointRadiusNormalCloud(new pcl::PointCloud<pcl::PointNormal>());
      *firstPointRadiusNormalCloud = geoGraspPoints.getFirstPointRadiusNormalCloud();
      pcl::PointCloud<pcl::PointNormal>::Ptr firstNormalCloudVoxel(new pcl::PointCloud<pcl::PointNormal>());
      *firstNormalCloudVoxel = geoGraspPoints.getFirstPointRadiusVoxelNormalCloud();

      pcl::PointCloud<pcl::PointNormal>::Ptr secondPointRadiusNormalCloud(new pcl::PointCloud<pcl::PointNormal>());
      *secondPointRadiusNormalCloud = geoGraspPoints.getSecondPointRadiusNormalCloud();
      pcl::PointCloud<pcl::PointNormal>::Ptr secondNormalCloudVoxel(new pcl::PointCloud<pcl::PointNormal>());
      *secondNormalCloudVoxel = geoGraspPoints.getSecondPointRadiusVoxelNormalCloud();

      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> 
        firstPointNormalColorHandler(firstPointRadiusNormalCloud, "curvature");
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal>
        secondPointNormalColorHandler(secondPointRadiusNormalCloud, "curvature");
  
      viewer->addPointCloud<pcl::PointNormal>(firstPointRadiusNormalCloud, firstPointNormalColorHandler, 
                                            objectLabel + "First point normals cloud");
      viewer->addPointCloudNormals<pcl::PointNormal>(firstNormalCloudVoxel, 1, 0.05,
                                                    objectLabel + "First point normals direction");

      viewer->addPointCloud<pcl::PointNormal>(secondPointRadiusNormalCloud, secondPointNormalColorHandler, 
                                            objectLabel + "Second point normals cloud");
      viewer->addPointCloudNormals<pcl::PointNormal>(secondNormalCloudVoxel, 1, 0.05, 
                                                    objectLabel + "Second point normals direction");*/

      /////////////////////////////////////////////////////////////////////////////////////////////////////////
  
      /*viewer->addSphere(geoGraspPoints.getFirstGraspPoint(), 0.01, 0, 0, 255, 
        objectLabel + "First initial grasping point");
      viewer->addSphere(geoGraspPoints.getSecondGraspPoint(), 0.01, 255, 0, 0, 
        objectLabel + "Second initial grasping point");*/

      /////////////////////////////////////////////////////////////////////////////////////////////////////////

      viewer->addSphere(bestGrasp.firstPoint, 0.01, 0, 0, 255,
        objectLabel + "First best grasp point");
      viewer->addSphere(bestGrasp.secondPoint, 0.01, 255, 0, 0,
        objectLabel + "Second best grasp point");
      /*viewer->addLine(bestGrasp.firstPoint, bestGrasp.secondPoint, 0, 255, 0, 
        objectLabel + "Grasping line");*/

      objectNumber++;
    }

    //viewer->spin();
    //viewer->spinOnce();
    while (!viewer->wasStopped())
      viewer->spinOnce(100);
  }
  
  boost::chrono::thread_clock::time_point stop = boost::chrono::thread_clock::now();

  accDuration = accDuration + boost::chrono::duration_cast<boost::chrono::milliseconds>(stop-start).count();

  std::cout << "++ Frames: " << frames << "\n";
  std::cout << "++ In: " << accDuration << "ms\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_processor");

  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.1);

  ros::NodeHandle n("~");
  std::string cloudTopic;
  
  n.getParam("topic", cloudTopic);

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, cloudCallback);

  ros::spin();

  return 0;
}
