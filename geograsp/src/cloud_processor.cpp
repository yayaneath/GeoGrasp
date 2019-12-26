#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geograsp/GeoGrasp.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

// callback signature
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & inputCloudMsg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloudMsg, *cloud);

  // Remove NaN values and make it dense
  std::vector<int> nanIndices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);

  // Remove background points
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;
  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("z");
  ptFilter.setFilterLimits(0.0, 1.5);
  ptFilter.filter(*cloud);

  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("y");
  ptFilter.setFilterLimits(-0.55, 0.40);
  ptFilter.filter(*cloud);

  ptFilter.setInputCloud(cloud);
  ptFilter.setFilterFieldName("x");
  ptFilter.setFilterLimits(-0.50, 0.50);
  ptFilter.filter(*cloud);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> sacSegmentator;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>());

  sacSegmentator.setModelType(pcl::SACMODEL_PLANE);
  sacSegmentator.setMethodType(pcl::SAC_RANSAC);
  sacSegmentator.setMaxIterations(50);
  sacSegmentator.setDistanceThreshold(0.02);
  sacSegmentator.setInputCloud(cloud);
  sacSegmentator.segment(*inliers, *coefficients);

  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
  indExtractor.setInputCloud(cloud);
  indExtractor.setIndices(inliers);
  indExtractor.setNegative(false);

  // Get the points associated with the planar surface
  indExtractor.filter(*cloudPlane);

  // Remove the planar inliers, extract the rest
  indExtractor.setNegative(true);
  indExtractor.filter(*cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
  ecExtractor.setClusterTolerance(0.01);
  ecExtractor.setMinClusterSize(750);
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

    // Every cluster found is considered an object
    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());

      for (std::vector<int>::const_iterator pit = it->indices.begin(); 
          pit != it->indices.end(); ++pit)
        objectCloud->points.push_back(cloud->points[*pit]);

      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      // Create and initialise GeoGrasp
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPlaneXYZ(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloudXYZ(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*cloudPlane, *cloudPlaneXYZ);
      pcl::copyPointCloud(*objectCloud, *objectCloudXYZ);

      GeoGrasp geoGraspPoints;
      geoGraspPoints.setBackgroundCloud(cloudPlaneXYZ);
      geoGraspPoints.setObjectCloud(objectCloudXYZ);
      geoGraspPoints.setGripTipSize(25); // 25mm grip
      geoGraspPoints.setGrasps(1); // Keep track only of the best

      // Calculate grasping points
      geoGraspPoints.compute();

      // Extract best pair of points
      GraspContacts bestGrasp = geoGraspPoints.getBestGrasp();
      GraspPose bestPose = geoGraspPoints.getBestGraspPose();

      // Visualize the result
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(objectCloud);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planeRGB(cloudPlane);

      std::string objectLabel = "";
      std::ostringstream converter;

      converter << objectNumber;
      objectLabel += converter.str();
      objectLabel += "-";

      viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, rgb, objectLabel + "Object");
      viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeRGB, objectLabel + "Plane");

      viewer->addSphere(bestGrasp.firstPoint, 0.01, 0, 0, 255, objectLabel + "First best grasp point");
      viewer->addSphere(bestGrasp.secondPoint, 0.01, 255, 0, 0, objectLabel + "Second best grasp point");

      pcl::ModelCoefficients axeX;
      axeX.values.resize (6);    // We need 6 values
      axeX.values[0] = bestPose.midPointPose.translation()[0];
      axeX.values[1] = bestPose.midPointPose.translation()[1];
      axeX.values[2] = bestPose.midPointPose.translation()[2];
      axeX.values[3] = bestPose.midPointPose.linear()(0, 0);
      axeX.values[4] = bestPose.midPointPose.linear()(1, 0);
      axeX.values[5] = bestPose.midPointPose.linear()(2, 0);

      viewer->addLine(axeX, objectLabel + "Pose axeX");

      pcl::ModelCoefficients axeY;
      axeY.values.resize (6);    // We need 6 values
      axeY.values[0] = bestPose.midPointPose.translation()[0];
      axeY.values[1] = bestPose.midPointPose.translation()[1];
      axeY.values[2] = bestPose.midPointPose.translation()[2];
      axeY.values[3] = bestPose.midPointPose.linear()(0, 1);
      axeY.values[4] = bestPose.midPointPose.linear()(1, 1);
      axeY.values[5] = bestPose.midPointPose.linear()(2, 1);

      viewer->addLine(axeY, objectLabel + "Pose axeY");

      pcl::ModelCoefficients axeZ;
      axeZ.values.resize (6);    // We need 6 values
      axeZ.values[0] = bestPose.midPointPose.translation()[0];
      axeZ.values[1] = bestPose.midPointPose.translation()[1];
      axeZ.values[2] = bestPose.midPointPose.translation()[2];
      axeZ.values[3] = bestPose.midPointPose.linear()(0, 2);
      axeZ.values[4] = bestPose.midPointPose.linear()(1, 2);
      axeZ.values[5] = bestPose.midPointPose.linear()(2, 2);

      viewer->addLine(axeZ, objectLabel + "Pose axeZ");
      
      objectNumber++;
    }

    while (!viewer->wasStopped())
      viewer->spinOnce(100);
  }
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
