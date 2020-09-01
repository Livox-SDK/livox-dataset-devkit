# Livox_dataset_v1.0 devkit

The development kit provides a version of [ROS](https://www.ros.org/)-based livox point cloud reading and 3D target bounding box display node.

# Dependencies
- ROS
- cmake
- python3.6+

# Installation
1. Install ROS according to [ros_installation](http://wiki.ros.org/melodic/Installation/Ubuntu)

2. Put the two provided nodes under the `catkin_ws` directory you created

```bash
$ mkdir catkin_ws
$ cp -r ./src ./catkin_ws
```
# Useage
1. Put the provided `pcd_getname.py` file in the root directory of `livox_dataset_v1.0`.
2. Get the file name list of the point cloud pcd file and the corresponding image file name list. Firstly create a new file 'namelist.txt' in the same directory of 'pcd_getname.py', and input the target folder name, for example 'data4_2020_05_11', then 
```bash
$ python pcd_getname.py
```
It will generate the files containing the names of pcd and images of 'data4_2020_05_11' in the corresponding folders.
3. Change `pcd_anno_read_node.cpp`  and  `pcd_single_read_node.cpp` root path according to your storage path of `livox_dataset_v1.0`.
4. Compile the projects under directory of `catkin_ws`
```bash
$ catkin_make
$ source devel/setup.bash
```

# Visualization

1. For Visualizing annotation results in one direction (lidar coordinate system):
```bash
$ roslaunch pcd_single_read show_results.launch
```

2. For Visualizing the annotation results in all directions (world coordinate system)
```bash
$ roslaunch pcd_anno_read show_results.launch
```
