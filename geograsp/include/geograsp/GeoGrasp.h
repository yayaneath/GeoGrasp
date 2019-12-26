#ifndef GEOGRASP_H_
#define GEOGRASP_H_

#include <iostream>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>

struct GraspContacts {
  pcl::PointXYZ firstPoint;
  pcl::PointXYZ secondPoint;
};

struct GraspPose {
  Eigen::Vector3f firstPoint;
  Eigen::Vector3f secondPoint;
  Eigen::Affine3f midPointPose;
};

class GeoGrasp {
  public:
    GeoGrasp();
    ~GeoGrasp();

    void setBackgroundCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void setBackgroundPlaneCoeff(const pcl::ModelCoefficients & coefficients);
    void setObjectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void setGrasps(const int & grasps);
    void setGripTipSize(const int & size);

    GraspContacts getBestGrasp() const;
    std::vector<GraspContacts> getGrasps() const;

    GraspPose getBestGraspPose() const;

    float getBestRanking() const;
    std::vector<float> getRankings() const;

    pcl::ModelCoefficients getObjectAxisCoeff() const;
    pcl::ModelCoefficients getBackgroundPlaneCoeff() const;

    void compute();

  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr backgroundCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr objectNormalCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr graspPlaneCloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr firstPointRadiusCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr firstPointRadiusNormalCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr firstNormalCloudVoxel;

    pcl::PointCloud<pcl::PointXYZ>::Ptr secondPointRadiusCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr secondPointRadiusNormalCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr secondNormalCloudVoxel;

    pcl::ModelCoefficients::Ptr backgroundPlaneCoeff;
    pcl::ModelCoefficients::Ptr objectAxisCoeff;
    pcl::ModelCoefficients::Ptr graspPlaneCoeff;

    pcl::PointXYZ objectCentroidPoint;
    pcl::PointNormal firstGraspPoint;
    pcl::PointNormal secondGraspPoint;

    std::vector<GraspContacts> graspPoints;
    std::vector<float> rankings;
    std::vector<float> pointsDistance;

    int numberBestGrasps;
    int gripTipSize;

    // Distance threshold for the grasp plane cloud
    static const float kGraspPlaneApprox;
    // Radius used to compute point cloud normals
    static const float kCloudNormalRadius;

    // Auxiliary functions

    void computeCloudPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
      pcl::ModelCoefficients::Ptr backPlaneCoeff);

    void filterOutliersFromCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
      const int & meanNeighbours, const float & distanceThreshold,
      pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud);

    void computeCloudNormals(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
      const float & searchRadius,
      pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals);

    void computeCloudGeometry(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
      pcl::ModelCoefficients::Ptr objAxisCoeff, pcl::PointXYZ & objCenterMass);

    template<typename T, typename U>
    void extractInliersCloud(const T & inputCloud,
      const pcl::PointIndices::Ptr & inputCloudInliers, T outputCloud);

    void findInitialPointsInSideView();

    void findInitialPointsInTopView();

    void buildGraspingPlane(const pcl::PointXYZ & planePoint,
      const Eigen::Vector3f & planeNormalVector, const float & distanceThreshold,
      const pcl::PointCloud<pcl::PointNormal>::Ptr & inputCloud,
      pcl::ModelCoefficients::Ptr graspPlaneCoeff,
      pcl::PointCloud<pcl::PointNormal>::Ptr graspPlaneCloud);

    void getClosestPointsByRadius(const pcl::PointNormal & point,
      const float & radius,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
      const pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
      pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud);

    template<typename T, typename U>
    void voxelizeCloud(const T & inputCloud, const float & leafSize,
      T outputCloud);

    void getBestGraspingPoints(
      const pcl::PointNormal & firstInitialPoint,
      const pcl::PointNormal & secondInitialPoint,
      const pcl::ModelCoefficients::Ptr & graspPlaneCoeff,
      const pcl::PointXYZ & centroidPoint,
      const pcl::PointCloud<pcl::PointNormal>::Ptr & firstNormalCloud,
      const pcl::PointCloud<pcl::PointNormal>::Ptr & secondNormalCloud,
      const int & numGrasps, std::vector<GraspContacts> & bestGrasps,
      std::vector<float> & bestRanks);
};

#endif
