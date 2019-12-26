#include "geograsp/GeoGrasp.h"

const float GeoGrasp::kGraspPlaneApprox = 0.007;
const float GeoGrasp::kCloudNormalRadius = 0.03;

GeoGrasp::GeoGrasp() :
    backgroundCloud(new pcl::PointCloud<pcl::PointXYZ>),
    objectCloud(new pcl::PointCloud<pcl::PointXYZ>),
    objectNormalCloud(new pcl::PointCloud<pcl::PointNormal>),
    graspPlaneCloud(new pcl::PointCloud<pcl::PointNormal>),
    firstPointRadiusCloud(new pcl::PointCloud<pcl::PointXYZ>),
    firstPointRadiusNormalCloud(new pcl::PointCloud<pcl::PointNormal>),
    firstNormalCloudVoxel(new pcl::PointCloud<pcl::PointNormal>),
    secondPointRadiusCloud(new pcl::PointCloud<pcl::PointXYZ>),
    secondPointRadiusNormalCloud(new pcl::PointCloud<pcl::PointNormal>),
    secondNormalCloudVoxel(new pcl::PointCloud<pcl::PointNormal>),
    backgroundPlaneCoeff(new pcl::ModelCoefficients),
    objectAxisCoeff(new pcl::ModelCoefficients),
    graspPlaneCoeff(new pcl::ModelCoefficients) { 
  numberBestGrasps = 1;
  gripTipSize = 30;
}

GeoGrasp::~GeoGrasp() {
  graspPoints.clear();
  rankings.clear();
  pointsDistance.clear();
}

void GeoGrasp::setBackgroundCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  backgroundCloud = cloud;
}

void GeoGrasp::setBackgroundPlaneCoeff(
    const pcl::ModelCoefficients & coefficients) {
  *backgroundPlaneCoeff = coefficients;
}

void GeoGrasp::setObjectCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  objectCloud = cloud;
}

void GeoGrasp::setGrasps(const int & grasps) {
  numberBestGrasps = grasps;
}

void GeoGrasp::setGripTipSize(const int & size) {
  gripTipSize = size;
}

GraspContacts GeoGrasp::getBestGrasp() const {
  GraspContacts grasp;

  if (this->graspPoints.empty())
    std::cout << "No grasp configurations were found during the computation\n";
  else {
    grasp.firstPoint = this->graspPoints[0].firstPoint;
    grasp.secondPoint = this->graspPoints[0].secondPoint;
  }

  return grasp;
}

std::vector<GraspContacts> GeoGrasp::getGrasps() const {
  return this->graspPoints;
}

GraspPose GeoGrasp::getBestGraspPose() const {
  GraspPose grasp;

  if (this->graspPoints.empty())
    std::cout << "No grasp configurations were found during the computation\n";
  else {
    grasp.firstPoint[0] = this->graspPoints[0].firstPoint.x;
    grasp.firstPoint[1] = this->graspPoints[0].firstPoint.y;
    grasp.firstPoint[2] = this->graspPoints[0].firstPoint.z;

    grasp.secondPoint[0] = this->graspPoints[0].secondPoint.x;
    grasp.secondPoint[1] = this->graspPoints[0].secondPoint.y;
    grasp.secondPoint[2]= this->graspPoints[0].secondPoint.z;

    /*Eigen::Vector3f midPoint((grasp.firstPoint[0] + grasp.secondPoint[0]) / 2.0,
                             (grasp.firstPoint[1] + grasp.secondPoint[1]) / 2.0,
                             (grasp.firstPoint[2] + grasp.secondPoint[2]) / 2.0);*/
    Eigen::Vector3f midPoint = (grasp.firstPoint + grasp.secondPoint) / 2.0;

    Eigen::Vector3f objAxisVector(this->objectAxisCoeff->values[3],
                                  this->objectAxisCoeff->values[4],
                                  this->objectAxisCoeff->values[5]);
    Eigen::Vector3f axeX, axeY, axeZ;
  
    axeZ = grasp.firstPoint - grasp.secondPoint;
    axeX = axeZ.cross(objAxisVector);
    axeY = axeZ.cross(axeX);
   
    axeX.normalize();
    axeY.normalize();
    axeZ.normalize();    

    Eigen::Matrix3f midPointRotation;
    midPointRotation << axeX[0], axeY[0], axeZ[0], 
                        axeX[1], axeY[1], axeZ[1],
                        axeX[2], axeY[2], axeZ[2];
    grasp.midPointPose.translation() = midPoint;
    grasp.midPointPose.linear() = midPointRotation;
  }

  return grasp;
}

float GeoGrasp::getBestRanking() const {
  float ranking;

  if (this->graspPoints.empty())
    std::cout << "No grasp configurations were found during the computation\n";
  else
    ranking = this->rankings[0];

  return ranking;
}

std::vector<float> GeoGrasp::getRankings() const {
  return this->rankings;
}

pcl::ModelCoefficients GeoGrasp::getObjectAxisCoeff() const {
  return *objectAxisCoeff;
}

pcl::ModelCoefficients GeoGrasp::getBackgroundPlaneCoeff() const {
  return *backgroundPlaneCoeff;
}

void GeoGrasp::compute() {
  // Some ratios dependant of the gripper
  const float voxelRadius = this->gripTipSize / 2000.0;
  float graspRadius = 2.0 * this->gripTipSize / 1000.0;

  std::cout << "Grasp radius: " << graspRadius << "\n"
            << "Voxel radius: " << voxelRadius << "\n";

  std::cout << "Loaded " << this->objectCloud->width * this->objectCloud->height 
            << " data points from object cloud\n";

  // Background plane
  if (backgroundPlaneCoeff->values.size() == 0)
    computeCloudPlane(this->backgroundCloud, this->backgroundPlaneCoeff);

  // Cloud plane filtering
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  float outliersThreshold = 1.0;
  int meanNeighbours = 50;

  filterOutliersFromCloud(this->objectCloud, meanNeighbours, outliersThreshold,
    cloud);

  // Object normals
  // This could be optimised passing a voxelized cloud instead
  computeCloudNormals(cloud, this->kCloudNormalRadius, this->objectNormalCloud);

  // Aproximate the object's main axis and centroid
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>);
  voxelizeCloud<pcl::PointCloud<pcl::PointXYZ>::Ptr, 
                pcl::VoxelGrid<pcl::PointXYZ> >(cloud, voxelRadius, voxelCloud);

  computeCloudGeometry(voxelCloud, this->objectAxisCoeff, this->objectCentroidPoint);
  
  // Find camera orientation wrt background plane
  Eigen::Vector3f backNormalVector(this->backgroundPlaneCoeff->values[0],
                                   this->backgroundPlaneCoeff->values[1],
                                   this->backgroundPlaneCoeff->values[2]);
  Eigen::Vector3f worldZVector(0, 0, 1);

  float backWorldZAngleCos = std::abs((backNormalVector.dot(worldZVector)) / 
    (backNormalVector.norm() * worldZVector.norm()));

  // Compute initial points accordingly
  if (backWorldZAngleCos > 0.9) {
    std::cout << "Camera in top view\n";
    findInitialPointsInTopView();
  }
  else {
    std::cout << "Camera in side view\n";
    findInitialPointsInSideView();
  }

  // A grasp plane must have been built, otherwise stop!
  if (graspPlaneCloud->points.size() == 0) {
    std::cout << "ERROR: GRASP PLANE IS EMPTY. Interrumpting.\n";

    return;
  }

  float objWidth = pcl::geometry::distance(this->firstGraspPoint.getVector3fMap(),
    this->secondGraspPoint.getVector3fMap());

  if (graspRadius * 2.0 >= objWidth)
    graspRadius = objWidth * 0.7 / 2.0;

  // Grasping areas
  getClosestPointsByRadius(this->firstGraspPoint, graspRadius, cloud, 
    this->objectNormalCloud, this->firstPointRadiusCloud, 
    this->firstPointRadiusNormalCloud);

  voxelizeCloud<pcl::PointCloud<pcl::PointNormal>::Ptr, 
                pcl::VoxelGrid<pcl::PointNormal> >(firstPointRadiusNormalCloud, 
                  voxelRadius, this->firstNormalCloudVoxel);
  
  getClosestPointsByRadius(this->secondGraspPoint, graspRadius, cloud,
    this->objectNormalCloud, this->secondPointRadiusCloud,
    this->secondPointRadiusNormalCloud);

  voxelizeCloud<pcl::PointCloud<pcl::PointNormal>::Ptr, 
                pcl::VoxelGrid<pcl::PointNormal> >(secondPointRadiusNormalCloud, 
                  voxelRadius, this->secondNormalCloudVoxel);

  // Best ranking
  getBestGraspingPoints(this->firstGraspPoint, this->secondGraspPoint, 
    this->graspPlaneCoeff, this->objectCentroidPoint, this->firstNormalCloudVoxel,
    this->secondNormalCloudVoxel, this->numberBestGrasps, this->graspPoints,
    this->rankings);

  for (size_t i = 0; i < this->rankings.size(); ++i)
    std::cout << "Grasp Configuration Rank: " << this->rankings[i] << "\n";

  std::cout << "===============================\n";
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - AUXILIARY FUNCTIONS - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void GeoGrasp::computeCloudPlane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
    pcl::ModelCoefficients::Ptr backPlaneCoeff) {
  pcl::PointIndices::Ptr cloudPlaneInliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(inputCloud);
  seg.segment(*cloudPlaneInliers, *backPlaneCoeff);
}

void GeoGrasp::filterOutliersFromCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
    const int & meanNeighbours, const float & distanceThreshold,
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorFilter;

  sorFilter.setInputCloud(inputCloud);
  sorFilter.setMeanK(meanNeighbours);
  sorFilter.setStddevMulThresh(distanceThreshold);
  sorFilter.filter(*outputCloud);
}

void GeoGrasp::computeCloudNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
    const float & searchRadius,
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormals) {

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> ne;

  ne.setInputCloud(inputCloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(searchRadius);
  ne.compute(*cloudNormals);

  // NormalEstimation only fills with normals the output
  for (size_t i = 0; i < cloudNormals->points.size(); ++i) {
    cloudNormals->points[i].x = inputCloud->points[i].x;
    cloudNormals->points[i].y = inputCloud->points[i].y;
    cloudNormals->points[i].z = inputCloud->points[i].z;
  }
}

void GeoGrasp::computeCloudGeometry(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
    pcl::ModelCoefficients::Ptr objAxisCoeff, pcl::PointXYZ & objCenterMass) {
  Eigen::Vector3f massCenter, majorVector, middleVector, minorVector;
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> featureExtractor;
  featureExtractor.setInputCloud(inputCloud);
  featureExtractor.compute();

  featureExtractor.getEigenVectors(majorVector, middleVector, minorVector);
  featureExtractor.getMassCenter(massCenter);

  objAxisCoeff->values.resize(6);
  objAxisCoeff->values[0] = massCenter[0];
  objAxisCoeff->values[1] = massCenter[1];
  objAxisCoeff->values[2] = massCenter[2];
  objAxisCoeff->values[3] = majorVector[0];
  objAxisCoeff->values[4] = majorVector[1];
  objAxisCoeff->values[5] = majorVector[2];

  objCenterMass.x = massCenter[0];
  objCenterMass.y = massCenter[1];
  objCenterMass.z = massCenter[2];
}

template<typename T, typename U>
void GeoGrasp::extractInliersCloud(const T & inputCloud,
    const pcl::PointIndices::Ptr & inputCloudInliers, T outputCloud) {
  U extractor;

  extractor.setInputCloud(inputCloud);
  extractor.setIndices(inputCloudInliers);
  extractor.filter(*outputCloud);
}

void GeoGrasp::findInitialPointsInSideView() {
  // Background plane normal and object main axis angle
  Eigen::Vector3f objAxisVector(this->objectAxisCoeff->values[3],
                                this->objectAxisCoeff->values[4],
                                this->objectAxisCoeff->values[5]);
  Eigen::Vector3f backNormalVector(this->backgroundPlaneCoeff->values[0],
                                   this->backgroundPlaneCoeff->values[1],
                                   this->backgroundPlaneCoeff->values[2]);
  Eigen::Vector3f worldZVector(0, 0, 1);

  float objGraspNormalAngleCos = std::abs((objAxisVector.dot(backNormalVector)) / 
    (objAxisVector.norm() * backNormalVector.norm()));
  float objWorldZAngleCos = std::abs((objAxisVector.dot(worldZVector)) / 
    (objAxisVector.norm() * worldZVector.norm()));

  std::cout << "Object angle cosine to background normal: " << objGraspNormalAngleCos << "\n";
  std::cout << "Object angle cosine to world Z: " << objWorldZAngleCos << "\n";

  // Grasping plane and initial points
  float planesAngleThreshold = 0.55, worldZAngleThreshold = 0.45;
  bool oppositeAxisVector = false;

  // Object's axis is reversed for consistency
  // Standing object but vector pointing down
  if (objGraspNormalAngleCos > (1.0 - planesAngleThreshold) && 
    this->objectAxisCoeff->values[4] > 0.0)
        oppositeAxisVector = true;
  else if (objGraspNormalAngleCos < planesAngleThreshold) { // Laying objects
    // Laying in the X direction but vector point left
    if (objWorldZAngleCos <= 0.1 && this->objectAxisCoeff->values[3] > 0.0)
        oppositeAxisVector = true;
    // Laying and pointing forward-right but vector points backwards-left
    else if (objWorldZAngleCos > 0.1 && this->objectAxisCoeff->values[3] < 0.0 &&
      this->objectAxisCoeff->values[4] > 0.0 && this->objectAxisCoeff->values[5] < 0.0)
        oppositeAxisVector = true;
    // Laying and pointing forward-left but vector points backwards-right
    else if (objWorldZAngleCos > 0.1 && this->objectAxisCoeff->values[3] > 0.0 &&
      this->objectAxisCoeff->values[4] > 0.0 && this->objectAxisCoeff->values[5] < 0.0)
        oppositeAxisVector = true;
  }

  if (oppositeAxisVector) {
      this->objectAxisCoeff->values[3] *= -1.0;
      this->objectAxisCoeff->values[4] *= -1.0;
      this->objectAxisCoeff->values[5] *= -1.0;

      objAxisVector = -objAxisVector;
  }

  // Grasping plane and initial points
  size_t firstPointIndex = -1, secondPointIndex = -1;

  buildGraspingPlane(this->objectCentroidPoint, objAxisVector, 
    this->kGraspPlaneApprox, this->objectNormalCloud, this->graspPlaneCoeff, 
    this->graspPlaneCloud);

  // Laying object in the X axis
  if (objGraspNormalAngleCos < planesAngleThreshold 
    && objWorldZAngleCos < worldZAngleThreshold) {
    std::cout << "It is a laying object in the X axis\n";

    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < this->graspPlaneCloud->points.size(); ++i) {
      if (this->graspPlaneCloud->points[i].z < minZ) {
        minZ = this->graspPlaneCloud->points[i].z;
        firstPointIndex = i;
      }

      if (this->graspPlaneCloud->points[i].z > maxZ) {
        maxZ = this->graspPlaneCloud->points[i].z;
        secondPointIndex = i;
      }
    }
  }
  else { // Standing objects and laying ones in the Z axis
    std::cout << "It is a laying object in the Z axis or it is standing\n";

    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < this->graspPlaneCloud->points.size(); ++i) {
      if (this->graspPlaneCloud->points[i].x < minX) {
        minX = this->graspPlaneCloud->points[i].x;
        firstPointIndex = i;
      }

      if (this->graspPlaneCloud->points[i].x > maxX) {
        maxX = this->graspPlaneCloud->points[i].x;
        secondPointIndex = i;
      }
    }
  }

  if (graspPlaneCloud->points.size() != 0) {
    this->firstGraspPoint = this->graspPlaneCloud->points[firstPointIndex];
    this->secondGraspPoint = this->graspPlaneCloud->points[secondPointIndex];
  }
}

void GeoGrasp::findInitialPointsInTopView() {
  // Background plane normal and object main axis angle
  Eigen::Vector3f objAxisVector(this->objectAxisCoeff->values[3],
                                this->objectAxisCoeff->values[4],
                                this->objectAxisCoeff->values[5]);
  Eigen::Vector3f worldXVector(1, 0, 0);
  Eigen::Vector3f worldZVector(0, 0, 1);

  float objWorldXAngleCos = std::abs((objAxisVector.dot(worldXVector)) / 
    (objAxisVector.norm() * worldXVector.norm()));
  float objWorldZAngleCos = std::abs((objAxisVector.dot(worldZVector)) / 
    (objAxisVector.norm() * worldZVector.norm()));

  std::cout << "Object angle cosine to world X: " << objWorldXAngleCos << "\n";
  std::cout << "Object angle cosine to world Z: " << objWorldZAngleCos << "\n";

  float worldXAngleThreshold = 0.5, worldZAngleThreshold = 0.25;
  bool reverseAxisVector = false;

  // Object's axis is reversed for consistency
  if (objWorldZAngleCos < worldZAngleThreshold) { // Laying objects
    // Object in the X axis but vector pointing right
    if (objWorldXAngleCos > worldXAngleThreshold && this->objectAxisCoeff->values[3] > 0)
      reverseAxisVector = true;
    // Object in the Y axis but vector pointing forward from the robot
    else if (objWorldXAngleCos < worldXAngleThreshold && this->objectAxisCoeff->values[4] < 0)
      reverseAxisVector = true;
  }
  // Standing object but vector pointing down with the Z axis
  else if (objWorldZAngleCos > worldZAngleThreshold && this->objectAxisCoeff->values[5] > 0)
    reverseAxisVector = true;

  if (reverseAxisVector) {
      this->objectAxisCoeff->values[3] *= -1.0;
      this->objectAxisCoeff->values[4] *= -1.0;
      this->objectAxisCoeff->values[5] *= -1.0;

      objAxisVector = -objAxisVector;
  }

  // Grasping plane and initial points
  size_t firstPointIndex = -1, secondPointIndex = -1;

  buildGraspingPlane(this->objectCentroidPoint, objAxisVector, 
    this->kGraspPlaneApprox, this->objectNormalCloud, this->graspPlaneCoeff, 
    this->graspPlaneCloud);

  // Object in the X axis
  if (objWorldXAngleCos > worldXAngleThreshold) {
    std::cout << "It is oriented with the X axis\n";

    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < this->graspPlaneCloud->points.size(); ++i) {
      if (this->graspPlaneCloud->points[i].y < minY) {
        minY = this->graspPlaneCloud->points[i].y;
        firstPointIndex = i;
      }

      if (this->graspPlaneCloud->points[i].y > maxY) {
        maxY = this->graspPlaneCloud->points[i].y;
        secondPointIndex = i;
      }
    }
  }
  else { // Object in the Y axis
    std::cout << "It is oriented with the Y axis\n";

    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < this->graspPlaneCloud->points.size(); ++i) {
      if (this->graspPlaneCloud->points[i].x < minX) {
        minX = this->graspPlaneCloud->points[i].x;
        firstPointIndex = i;
      }

      if (this->graspPlaneCloud->points[i].x > maxX) {
        maxX = this->graspPlaneCloud->points[i].x;
        secondPointIndex = i;
      }
    }
  }

  if (graspPlaneCloud->points.size() != 0) {
    this->firstGraspPoint = this->graspPlaneCloud->points[firstPointIndex];
    this->secondGraspPoint = this->graspPlaneCloud->points[secondPointIndex];
  }
}

void GeoGrasp::buildGraspingPlane(const pcl::PointXYZ & planePoint,
    const Eigen::Vector3f & planeNormalVector, const float & distanceThreshold,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & inputCloud,
    pcl::ModelCoefficients::Ptr graspPlaneCoeff,
    pcl::PointCloud<pcl::PointNormal>::Ptr graspPlaneCloud) {
  // Grasping plane model
  Eigen::Vector3f planePointVector(planePoint.x, planePoint.y, planePoint.z);
  Eigen::Hyperplane<float,3> graspHyperplane(planeNormalVector, planePointVector);

  graspPlaneCoeff->values.resize(4);
  graspPlaneCoeff->values[0] = graspHyperplane.coeffs()[0];
  graspPlaneCoeff->values[1] = graspHyperplane.coeffs()[1];
  graspPlaneCoeff->values[2] = graspHyperplane.coeffs()[2];
  graspPlaneCoeff->values[3] = graspHyperplane.coeffs()[3];

  // Grasping plane cloud
  Eigen::Vector4f graspPlaneVector(graspHyperplane.coeffs()[0],
                                   graspHyperplane.coeffs()[1],
                                   graspHyperplane.coeffs()[2],
                                   graspHyperplane.coeffs()[3]);
  pcl::SampleConsensusModelPlane<pcl::PointNormal>::Ptr planeSAC(new pcl::SampleConsensusModelPlane<pcl::PointNormal>(inputCloud));
  pcl::PointIndices::Ptr graspPlaneIndices(new pcl::PointIndices);

  // TODO: SOMETIMES IT DOESN'T FIND POINTS NEAR THE GRASP PLANE
  planeSAC->selectWithinDistance(graspPlaneVector, distanceThreshold, 
    graspPlaneIndices->indices);
  extractInliersCloud<pcl::PointCloud<pcl::PointNormal>::Ptr, 
                      pcl::ExtractIndices<pcl::PointNormal> >(inputCloud,
                        graspPlaneIndices, graspPlaneCloud);
}

void GeoGrasp::getClosestPointsByRadius(const pcl::PointNormal & point,
    const float & radius, const pcl::PointCloud<pcl::PointXYZ>::Ptr & inputCloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & inputNormalCloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsCloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr normalCloud) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr treeSearch(new pcl::search::KdTree<pcl::PointNormal>());
  pcl::PointIndices::Ptr pointsIndex(new pcl::PointIndices);
  std::vector<float> pointsSquaredDistance;

  treeSearch->setInputCloud(inputNormalCloud);

  if (treeSearch->radiusSearch(point, radius, pointsIndex->indices, 
      pointsSquaredDistance)) {
    extractInliersCloud<pcl::PointCloud<pcl::PointXYZ>::Ptr,
                        pcl::ExtractIndices<pcl::PointXYZ> >(inputCloud,
                          pointsIndex, pointsCloud);
    extractInliersCloud<pcl::PointCloud<pcl::PointNormal>::Ptr,
                        pcl::ExtractIndices<pcl::PointNormal> >(inputNormalCloud, 
                          pointsIndex, normalCloud);
  }
}

template<typename T, typename U>
void GeoGrasp::voxelizeCloud(const T & inputCloud, const float & leafSize,
    T outputCloud) {
  U voxelFilter;

  voxelFilter.setInputCloud(inputCloud);
  voxelFilter.setLeafSize(leafSize, leafSize, leafSize);
  voxelFilter.filter(*outputCloud);
}

// Tests every combination of points from both initial areas and ranks them in
// function of:
//    - distance to the object's centroid
//    - distance to the grasping plane
//    - curvature
//    - angle between the line formed by the grasping points and their normals
//
// In addition, these ranked points cannot be closer to the centroid than the
// initial points to avoid searching in the objects facing surface, and they
// must be distanced from each other at least as the initial points. 
// Furthermore, those new points cannot be closer to the other side initial
// points.
//
// @param firstInitialPoint -> Point in the first area which is the initial 
//                             grasp point in the area
// @param secondInitialPoint -> Point in the second area which is the initial
//                              grasp point in the area
// @param graspPlaneCoeff -> Coefficients of the grasping plane
// @param centroidPoint -> Object's centroid point
// @param firstNormalCloud -> Cloud of PointNormal in the first area
// @param secondNormalCloud -> Cloud of PointNormal in the second area
// @param numGrasps -> Since every combination is ranked, this parameter
//                     indicates the number of best combinations that will
//                     be kept in the returning vectors
// @param bestGrasps -> Vector to be filled with best grasps found
// @param bestRanks -> Vector to be filled with the best configurations' ranks

void GeoGrasp::getBestGraspingPoints(
    const pcl::PointNormal & firstInitialPoint,
    const pcl::PointNormal & secondInitialPoint,
    const pcl::ModelCoefficients::Ptr & graspPlaneCoeff,
    const pcl::PointXYZ & centroidPoint,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & firstNormalCloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr & secondNormalCloud,
    const int & numGrasps, std::vector<GraspContacts> & bestGrasps,
    std::vector<float> & bestRanks) {
  const float weightR1 = 1.5, weightR2 = 1.0; // BEST VALUES FOR W1 AND W2

  ////////////////////////////////////////////////////////////////////////////////
  // First we obtain the min and max values in order to normalize data

  Eigen::Vector3f graspNormalVector(graspPlaneCoeff->values[0],
                                    graspPlaneCoeff->values[1],
                                    graspPlaneCoeff->values[2]);
  Eigen::Hyperplane<float,3> graspHyperplane(graspNormalVector, 
    graspPlaneCoeff->values[3]);
  Eigen::Vector3f centroidVector(centroidPoint.x,
                                 centroidPoint.y,
                                 centroidPoint.z);
  float firstCentroidDistanceMin, firstCentroidDistanceMax;
  float firstGraspPlaneDistanceMin, firstGraspPlaneDistanceMax;
  float firstCurvatureMin, firstCurvatureMax;

  firstCurvatureMin = std::numeric_limits<float>::max();
  firstCentroidDistanceMin = firstGraspPlaneDistanceMin = firstCurvatureMin;
  firstCurvatureMax = std::numeric_limits<float>::min();
  firstCentroidDistanceMax = firstGraspPlaneDistanceMax = firstCurvatureMax; 

  for (size_t i = 0; i < firstNormalCloud->points.size(); ++i) {
    pcl::PointNormal thisPoint = firstNormalCloud->points[i];
    Eigen::Vector3f thisVector(thisPoint.x, thisPoint.y, thisPoint.z);
    float pointCentroidDistance = pcl::geometry::distance(thisVector, 
      centroidVector);
    float pointGraspPlaneDistance = graspHyperplane.absDistance(thisVector);
    float pointCurvature = thisPoint.curvature;

    if (pointCentroidDistance < firstCentroidDistanceMin)
      firstCentroidDistanceMin = pointCentroidDistance;
    if (pointCentroidDistance > firstCentroidDistanceMax)
      firstCentroidDistanceMax = pointCentroidDistance;

    if (pointGraspPlaneDistance < firstGraspPlaneDistanceMin)
      firstGraspPlaneDistanceMin = pointGraspPlaneDistance;
    if (pointGraspPlaneDistance > firstGraspPlaneDistanceMax)
      firstGraspPlaneDistanceMax = pointGraspPlaneDistance; 

    if (pointCurvature < firstCurvatureMin)
      firstCurvatureMin = pointCurvature;
    if (pointCurvature > firstCurvatureMax)
      firstCurvatureMax = pointCurvature; 
  }

  float secondCentroidDistanceMin, secondCentroidDistanceMax;
  float secondGraspPlaneDistanceMin, secondGraspPlaneDistanceMax;
  float secondCurvatureMin, secondCurvatureMax;

  secondCurvatureMin = std::numeric_limits<float>::max();
  secondCentroidDistanceMin = secondGraspPlaneDistanceMin = secondCurvatureMin;
  secondCurvatureMax = std::numeric_limits<float>::min();
  secondCentroidDistanceMax = secondGraspPlaneDistanceMax = secondCurvatureMax;

  for (size_t i = 0; i < secondNormalCloud->points.size(); ++i) {
    pcl::PointNormal thisPoint = secondNormalCloud->points[i];
    Eigen::Vector3f thisVector(thisPoint.x, thisPoint.y, thisPoint.z);
    float pointCentroidDistance = pcl::geometry::distance(thisVector, 
      centroidVector);
    float pointGraspPlaneDistance = graspHyperplane.absDistance(thisVector);
    float pointCurvature = thisPoint.curvature;

    if (pointCentroidDistance < secondCentroidDistanceMin)
      secondCentroidDistanceMin = pointCentroidDistance;
    if (pointCentroidDistance > secondCentroidDistanceMax)
      secondCentroidDistanceMax = pointCentroidDistance;

    if (pointGraspPlaneDistance < secondGraspPlaneDistanceMin)
      secondGraspPlaneDistanceMin = pointGraspPlaneDistance;
    if (pointGraspPlaneDistance > secondGraspPlaneDistanceMax)
      secondGraspPlaneDistanceMax = pointGraspPlaneDistance;  

    if (pointCurvature < secondCurvatureMin)
      secondCurvatureMin = pointCurvature;
    if (pointCurvature > secondCurvatureMax)
      secondCurvatureMax = pointCurvature;  
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Loop in search of the best points tuple

  Eigen::Vector3f firstInitialVector(firstInitialPoint.x,
                                     firstInitialPoint.y,
                                     firstInitialPoint.z);
  Eigen::Vector3f secondInitialVector(secondInitialPoint.x,
                                      secondInitialPoint.y,
                                      secondInitialPoint.z);
  float initialPointsDistance = pcl::geometry::distance(firstInitialVector, 
    secondInitialVector);
  float firstMinDistance = pcl::geometry::distance(firstInitialVector, 
    centroidVector);
  float secondMinDistance = pcl::geometry::distance(secondInitialVector, 
    centroidVector);
  float pointRank1 = 0.0, pointRank2 = 0.0, pointRank = 0.0;
  float epsilon = 1e-8; // Needed to avoid division by zero

  firstMinDistance = (firstMinDistance + epsilon - firstCentroidDistanceMin) /
                    (firstCentroidDistanceMax - firstCentroidDistanceMin);
  secondMinDistance = (secondMinDistance + epsilon - secondCentroidDistanceMin) /
                    (secondCentroidDistanceMax - secondCentroidDistanceMin);
  bestRanks.insert(bestRanks.begin(), std::numeric_limits<float>::min());

  for (size_t i = 0; i < firstNormalCloud->points.size(); ++i) {
    pcl::PointNormal firstPoint = firstNormalCloud->points[i];
    Eigen::Vector3f firstVector(firstPoint.x, firstPoint.y, firstPoint.z);
    float firstPointSecondInitialDistance = pcl::geometry::distance(firstVector, 
      secondInitialVector);
    float firstPointCentroidDistance = pcl::geometry::distance(firstVector, 
      centroidVector);
    firstPointCentroidDistance = (firstPointCentroidDistance + epsilon - 
      firstCentroidDistanceMin) / (firstCentroidDistanceMax - 
      firstCentroidDistanceMin);

    // Point is closer to the centroid than initial one: point is on the surface!
    if (firstPointCentroidDistance < firstMinDistance ||
        firstPointSecondInitialDistance < initialPointsDistance)
      continue;

    Eigen::Vector3f firstNormalVector(firstPoint.normal_x,
                                      firstPoint.normal_y,
                                      firstPoint.normal_z);
    float firstPointGraspPlaneDistance = graspHyperplane.absDistance(firstVector);
    float firstPointCurvature = firstPoint.curvature;

    // Normalize
    firstPointGraspPlaneDistance = (firstPointGraspPlaneDistance + epsilon - 
      firstGraspPlaneDistanceMin) / (firstGraspPlaneDistanceMax - 
      firstGraspPlaneDistanceMin);
    firstPointCurvature = (firstPointCurvature + epsilon - firstCurvatureMin) /
      (firstCurvatureMax - firstCurvatureMin);

    for (size_t j = 0; j < secondNormalCloud->points.size(); ++j) {
      pcl::PointNormal secondPoint = secondNormalCloud->points[j];
      Eigen::Vector3f secondVector(secondPoint.x, secondPoint.y, secondPoint.z);
      float graspPointsDistance = pcl::geometry::distance(firstVector, 
        secondVector);
      float secondPointFirstInitialDistance = pcl::geometry::distance(
        secondVector, firstInitialVector);
      float secondPointCentroidDistance = pcl::geometry::distance(secondVector, 
        centroidVector);
      secondPointCentroidDistance = (secondPointCentroidDistance + epsilon - 
        secondCentroidDistanceMin) / (secondCentroidDistanceMax - 
        secondCentroidDistanceMin);

      // Points are closer to the centroid than initials: points on the surface!
      if (secondPointCentroidDistance < secondMinDistance * 0.98 ||
          graspPointsDistance < initialPointsDistance * 0.98 ||
          secondPointFirstInitialDistance < initialPointsDistance * 0.98)
        continue;

      Eigen::Vector3f secondNormalVector(secondPoint.normal_x,
                                        secondPoint.normal_y,
                                        secondPoint.normal_z);
      float secondPointGraspPlaneDistance = graspHyperplane.absDistance(
        secondVector);
      float secondPointCurvature = secondPoint.curvature;

      Eigen::Vector3f graspDirectionVector = 
        Eigen::ParametrizedLine<float,3>::Through(firstVector, secondVector).direction();
      float graspPlaneAngleCos = epsilon + 
        std::abs((graspDirectionVector.dot(graspNormalVector)) /
          (graspDirectionVector.norm() * graspNormalVector.norm()));

      float firstGraspAngleCos = epsilon + 
        std::abs((graspDirectionVector.dot(firstNormalVector)) /
          (graspDirectionVector.norm() * firstNormalVector.norm()));
      float secondGraspAngleCos = epsilon + 
        std::abs((graspDirectionVector.dot(secondNormalVector)) /
          (graspDirectionVector.norm() * secondNormalVector.norm()));

      // Normalize
      secondPointGraspPlaneDistance = (secondPointGraspPlaneDistance + epsilon -
        secondGraspPlaneDistanceMin) / (secondGraspPlaneDistanceMax - 
        secondGraspPlaneDistanceMin);
      secondPointCurvature = (secondPointCurvature + epsilon -
        secondCurvatureMin) / (secondCurvatureMax - secondCurvatureMin);

      // Ranking function
      pointRank1 = 2.0 - firstPointGraspPlaneDistance - secondPointGraspPlaneDistance -
        (graspPlaneAngleCos - 0.20) * 10.0; // This [-8, 2], being cos=0.2 -> 0
        //1.0 - std::exp(graspPlaneAngleCos * 10.0) / 100.0;
      pointRank2 = 2.0 - firstPointCurvature - secondPointCurvature +
        firstGraspAngleCos + secondGraspAngleCos -
        std::abs(firstGraspAngleCos - secondGraspAngleCos);
      pointRank = weightR1 * pointRank1 + weightR2 * pointRank2;

      // Minimum quality criteria
      if (pointRank1 < 1.0 || pointRank2 < 1.0)
        continue;

      size_t rank = 0;
      std::vector<GraspContacts>::iterator graspsIt;

      for (rank = 0, graspsIt = bestGrasps.begin(); rank < bestRanks.size(); 
          ++rank, ++graspsIt) {
        if (pointRank > bestRanks[rank]) {
          GraspContacts newGrasp;
          newGrasp.firstPoint.x = firstPoint.x;
          newGrasp.firstPoint.y = firstPoint.y;
          newGrasp.firstPoint.z = firstPoint.z;
          newGrasp.secondPoint.x = secondPoint.x;
          newGrasp.secondPoint.y = secondPoint.y;
          newGrasp.secondPoint.z = secondPoint.z;

          bestRanks.insert(bestRanks.begin() + rank, pointRank);
          bestGrasps.insert(graspsIt, newGrasp);

          break;
        }
      }
    }
  }

  bestRanks.resize(numGrasps);
  bestGrasps.resize(numGrasps);

  ////////////////////////////////////////////////////////////////////////////////
  // No grasp configurations found, then add initial points

  if (std::numeric_limits<float>::min() == *bestRanks.begin()) {
    GraspContacts newGrasp;
    newGrasp.firstPoint.x = firstInitialPoint.x;
    newGrasp.firstPoint.y = firstInitialPoint.y;
    newGrasp.firstPoint.z = firstInitialPoint.z;

    newGrasp.secondPoint.x = secondInitialPoint.x;
    newGrasp.secondPoint.y = secondInitialPoint.y;
    newGrasp.secondPoint.z = secondInitialPoint.z;

    bestGrasps.insert(bestGrasps.begin(), newGrasp);
    bestGrasps.resize(1);
    bestRanks.resize(0);

    Eigen::Vector3f firstVector(firstInitialPoint.x, 
      firstInitialPoint.y, firstInitialPoint.z);
    Eigen::Vector3f firstNormalVector(firstInitialPoint.normal_x,
      firstInitialPoint.normal_y, firstInitialPoint.normal_z);

    Eigen::Vector3f secondVector(secondInitialPoint.x,
      secondInitialPoint.y, secondInitialPoint.z);
    Eigen::Vector3f secondNormalVector(secondInitialPoint.normal_x,
      secondInitialPoint.normal_y, secondInitialPoint.normal_z);

    float firstPointGraspPlaneDistance = graspHyperplane.absDistance(firstVector);
    float firstPointCurvature = firstInitialPoint.curvature;

    firstPointGraspPlaneDistance = (firstPointGraspPlaneDistance + epsilon - 
      firstGraspPlaneDistanceMin) / (firstGraspPlaneDistanceMax - 
      firstGraspPlaneDistanceMin);
    firstPointCurvature = (firstPointCurvature + epsilon - firstCurvatureMin) /
      (firstCurvatureMax - firstCurvatureMin);

    float secondPointGraspPlaneDistance = graspHyperplane.absDistance(secondVector);
    float secondPointCurvature = secondInitialPoint.curvature;

    secondPointGraspPlaneDistance = (secondPointGraspPlaneDistance + epsilon -
      secondGraspPlaneDistanceMin) / (secondGraspPlaneDistanceMax - 
      secondGraspPlaneDistanceMin);
    secondPointCurvature = (secondPointCurvature + epsilon - 
      secondCurvatureMin) / (secondCurvatureMax - secondCurvatureMin);

    Eigen::Vector3f graspDirectionVector = Eigen::ParametrizedLine<float,3>::Through(firstVector, secondVector).direction();
    float firstGraspAngleCos = epsilon + 
      std::abs((graspDirectionVector.dot(firstNormalVector)) /
      (graspDirectionVector.norm() * firstNormalVector.norm()));
    float secondGraspAngleCos = epsilon +
      std::abs((graspDirectionVector.dot(secondNormalVector)) /
      (graspDirectionVector.norm() * secondNormalVector.norm()));
    float graspPlaneAngleCos = epsilon +
      std::abs((graspDirectionVector.dot(graspNormalVector)) /
      (graspDirectionVector.norm() * graspNormalVector.norm()));

    // Ranking function
    pointRank1 = 2.0 - firstPointGraspPlaneDistance - secondPointGraspPlaneDistance -
      (graspPlaneAngleCos - 0.20) * 10.0; // This [-8, 2], being cos=0.2 -> 0
      //1.0 - std::exp(graspPlaneAngleCos * 10.0) / 100.0;
    pointRank2 = 2.0 - firstPointCurvature - secondPointCurvature +
      firstGraspAngleCos + secondGraspAngleCos -
      std::abs(firstGraspAngleCos - secondGraspAngleCos);
    pointRank = weightR1 * pointRank1 + weightR2 * pointRank2;

    bestRanks.insert(bestRanks.begin(), pointRank);
    bestRanks.resize(1);
  }
}
