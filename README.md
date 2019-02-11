# GeoGrasp - PACKAGE UNDER CONSTRUCTION!
Geometry-based method for computing grasping points on 3D point clouds. Find more details at: https://www.researchgate.net/publication/318874422_Using_Geometry_to_Detect_Grasping_Points_on_3D_Unknown_Point_Cloud

We are still working on improving the code efficiency and readability. Moreover, we will add more use cases with real robots. Nevertheless, in its current state, the package already works so a pair of grasping points can be computed for any given 3D point cloud.

# Requirements
The repo is a ROS package with the following dependecies:

- ROS Kinetic
- PCL 1.7

The rest of the dependencies (ROS packages) can be found at the `package.xml` file inside the GeoGrasp folder.

# Citation
Zapata-Impata, B. S., Mateo, C. M., Gil, P., & Pomares, J. (2017). Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud. In Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics (ICINCO) 2017 (Vol. 2, pp. 154–161). Best Paper Award. SCITEPRESS - Science and Technology Publications. https://doi.org/10.5220/0006470701540161

```
@inproceedings{Zapata-Impata2017,
	abstract = {In this paper, we focus on the task of computing a pair of points for grasping unknown objects, given a single point cloud scene with a partial view of them. The main goal is to estimate the best pair of 3D-located points so that a gripper can perform a stable grasp over the objects in the scene with no prior knowledge of their shape. We propose a geometrical approach to find those contact points by placing them near a perpendicular cutting plane to the object's main axis and through its centroid. During the experimentation we have found that this solution is fast enough and gives sufficiently stable grasps for being used on a real service robot.},
	author = {Zapata-Impata, Brayan S. and Mateo, Carlos M. and Gil, Pablo and Pomares, Jorge},
	booktitle = {Proceedings of the 14th International Conference on Informatics in Control, Automation and Robotics (ICINCO) 2017},
	doi = {10.5220/0006470701540161},
	isbn = {978-989-758-263-9},
	keywords = {3D Point Clouds,Grasping,Handle Grasping,Surface Detection,Unknown Object Manipulation.},
	number = {September},
	pages = {154--161},
	publisher = {SCITEPRESS - Science and Technology Publications},
	title = {{Using Geometry to Detect Grasping Points on 3D Unknown Point Cloud}},
	url = {http://www.scitepress.org/DigitalLibrary/Link.aspx?doi=10.5220/0006470701540161},
	volume = {2},
	year = {2017}
}

```
