# GeoGrasp - PACKAGE UNDER CONSTRUCTION!
Geometry-based method for computing grasping points on 3D point clouds. Find more details at: https://www.researchgate.net/publication/318874422_Using_Geometry_to_Detect_Grasping_Points_on_3D_Unknown_Point_Cloud

We are still working on improving the code efficiency and readability. Moreover, we will add tutorials on the use of GeoGrasp. Nevertheless, the current version of the package already works so a pair of grasping points can be computed for any given 3D point cloud.

# Requirements
The package has been tested on Ubuntu 16.04. GeoGrasp is wrapped in a ROS package with the following dependecies:

- ROS Kinetic
- PCL 1.7

The rest of the dependencies (ROS packages) can be found at the `package.xml` file inside the GeoGrasp folder. In order to compile it, just clone this repository inside the `source` directory of your catkin workspace and execute `catkin_make`.

# Examples of use

At `GeoGrasp/data` we have included two PCD files with two scenes. The `creeper-isolated.pcd` holds the 3D point cloud of a toy Creeper standing on a table. In contrast, `objects-example.pcd` contains a 3D point cloud in which there are multiple objets on a table. To test GeoGrasp, simply execute the test script `cloud_processor` included with the package:

```
rosrun geograsp cloud_processor _topic:="/cloud_pcd"
```

This will launch a ROS node that will subscribe to the topic `/cloud_pcd` in which point clouds will be published. The node reads these point clouds and processes them in order to compute the contact points using GeoGrasp. Then, execute the following:

```
rosrun pcl_ros pcd_to_pointcloud <file.pcd>
```

This will publish on a topic called `/cloud_pcd` the contains of the PCD file `<file.pcd>` (either of those included at `GeoGrasp/Data`).

# Citation
[1] Zapata-Impata, B. S., Mateo, C. M., Gil, P., & Pomares, J. (2017). Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud. In Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics (ICINCO) 2017 (Vol. 2, pp. 154–161). Best Paper Award. SCITEPRESS - Science and Technology Publications. https://doi.org/10.5220/0006470701540161

[2] Zapata-Impata, B. S., Gil, P., Pomares, J., & Torres, F. (2019). Fast Geometry-based Computation
of Grasping Points on 3D Point Clouds. International Journal of Advanced Robotic Systems. In
press.
