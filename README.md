# calibration_colmap
This package provides calibration of intrinsic parameters of a camera.

### Requirements

* COLMAP
* ROS kinetic
* NVIDIA GPU with CUDA installed

### Installation
Clone this repository into your catkin workspace.

### How to use
* Run a node publishing "/image_raw" and "/tf" (from "/world" to "/base_link")
* Launch colmap.launch
'roslaunch colmap colmap.launch'

### Parameters
|Parameter| Type| Description|
----------|-----|------------|
|'image'|*String*|The image topic. Default '/image_raw'|
|'parent_frame'|*String*|The parent frame of the /tf. Default '/world'|
|'child_frame'|*String*|The child frame of the /tf. Default '/base_link'|
|'dense'|*bool*|If dense is True, dense reconstruction is executed after sparse reconstruction. Default 'False'|
|'workspace'|*String*|COLMAP workspace. Default './tmp/colmap'|
|'image_num'|*uint*|Number of images for calibration. Default '50'|
|'distance'|*Double*|Interval of distance. Default '5' meters|
|'rotation'|*Double*|Interval of yaw angle. Default '5' degree|
|'yml_dir'|*String*|Path to output directory of yml file. Default './yml/colmap'|
|'yml_name'|*String*|yml file name. Default 'camera_param.yml'|
