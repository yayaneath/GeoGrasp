# GeoGrasp
Geometry-based method for computing grasping points on 3D point clouds. Find more details at: https://www.researchgate.net/publication/331358070_Fast_Geometry-based_Computation_of_Grasping_Points_on_Three-dimensional_Point_Clouds

We are still working on improving the code efficiency and readability. Later, we will add tutorials on the use of GeoGrasp to work with robotic graspers.

# Requirements
The package has been tested on Ubuntu 16.04. GeoGrasp is wrapped in a ROS package with the following dependecies:

- ROS Kinetic
- PCL 1.7

The rest of the dependencies (ROS packages) can be found at the `package.xml` file inside the GeoGrasp folder. In order to compile it, just clone this repository inside the `source` directory of your catkin workspace and execute `catkin_make`.

# Examples of use

At `GeoGrasp/data` we have included two PCD files with two scenes. The `creeper-isolated.pcd` holds the 3D point cloud of a toy Creeper standing on a table. In contrast, `objects-example.pcd` contains a 3D point cloud in which there are multiple objets on a table. These clouds were captured with a Intel RealSense SR300 camera. To test GeoGrasp, simply execute the test script `cloud_processor` included in the repository:

```
rosrun geograsp cloud_processor _topic:="/cloud_pcd"
```

This launches a ROS node that subscribes to the topic `/cloud_pcd` in which point clouds will be published. The node reads these point clouds and processes them in order to compute the contact points using GeoGrasp. Next, execute the following node (`pcl_ros` ROS package is required for running this example):

```
rosrun pcl_ros pcd_to_pointcloud <file.pcd>
```

This publishes the contains of the PCD file `<file.pcd>` (either of those included at `GeoGrasp/Data`) on a topic called `/cloud_pcd`. See below an example of the computed points for the example PCD files:

<img src="/data/creeper-isolated.png" width="400"> <img src="/data/objects.png" width="445">

# TO-DOs

- Include example of use of the grasping points with robot

# Citation
[1] Zapata-Impata, B. S., Mateo, C. M., Gil, P., & Pomares, J. (2017). Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud. In Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics (ICINCO) 2017 (Vol. 2, pp. 154–161). Best Paper Award. SCITEPRESS - Science and Technology Publications. https://doi.org/10.5220/0006470701540161

[2] Zapata-Impata, B. S., Gil, P., Pomares, J., & Torres, F. (2019). Fast geometry-based computation of grasping points on three-dimensional point clouds. International Journal of Advanced Robotic Systems, 16(1), 172988141983184. https://doi.org/10.1177/1729881419831846
