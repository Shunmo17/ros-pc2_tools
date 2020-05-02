# pc2_tools

## Description

This is a ros package that summarizes pointcloud-related nodes.



## Contents

### scripts/pc2_to_grid_image.py

This node convert LRS pointcloud data to grid image, and save it as jpg. We assume this node is used in order to save LRS scan data as JPG when you make map with a SLAM package.

#### Subscribe

- lrs pc2 topic : sensor_msgs/PointCloud2

#### Output

- JPG files



### launch/pc2_to_grid_image.launch

This launch file launches `pc2_to_grid_image.py`.

#### Parameters

- lrs_pc2_topic : LRS pointcloud2 topic (It must be converted to PointCloud2 type beforehand.)
- save_path : save path of jpg files
- lrs_frame : frame_id of LRS
- grid_size : grid size of map [m]
- x_min, x_max, y_min, y_max : map size [m]



### src/pc2_transformer.cpp

This node transform PointCoud2 topic to the specified frame.

#### Subscribe

- input PointCloud2 topic : sensor_msgs/PointCloud2

#### Publish

- output PointCloud2 topic : sensor_msgs/PointCloud2



### launch/pc2_transformer.launch

This launch file launches `pc2_transformer.cpp`.

#### Parameters

- ns : If you use this node more than one, please use this namespace.
- input_pc2_topic : input PointCloud2 topic name
- output_pc2_topic : output PointCloud2 topic name
- new_frame_id : frame id of transformed PointCloud2 topic
- 

### src/pc2_to_image.cpp

This node generate Image from PointCoud2 topic .

#### Subscribe

- input PointCloud2 topic : sensor_msgs/PointCloud2

#### Publish

- output Image topic : sensor_msgs/Image



### launch/pc2_to_image.launch

This launch file launches `pc2_to_image.py`.

#### Parameters

- ns : If you use this node more than one, please use this namespace.
- input_pc2_topic : input PointCloud2 topic name
- output_image_topic : output Image topic name



## Usage

### pc2_to_grid_image

```
roslaunch pc2_tools pc2_to_grid_image.launch lrs_pc2_topic:="/lrs/scan/pc2" save_path:="/$(find pc2_tools)/saved_lrs_data" lrs_frame:=""/lrs_frame" grid_size:="0.05" x_min:="-1.0" x_max:="10.0" y_min:="-1.0" y_max:="10.0"
```



### pc2_transformer

```
roslaunch pc2_tools pc2_transformer.launch ns:="ns1" input_pc2_topic:="/lrs/scan" output_pc2_topic:="lrs/transformd_scan" new_frame_id:="/map"
```



