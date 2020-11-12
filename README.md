# Assembly Camera Manager

Camera manager for furniture assembly project (Kinect Azure and Zivid) using ArUCO marker

### Features

- Extrinsic Calibration for Kinect Azure
- Extrinsic Calibration for Zivid
- tf manager for world map and cameras

### Prerequisites

- Azure Kinect driver & ROS Wrapper
- fiducials: https://github.com/SeungBack/fiducials

### TO Do

- Accurate calibration of triple camera network using ICP b/w markers


## Getting Started
### Kinect Azure

1. Launch camera node and manager
```
$ ROS_NAMESPACE=azure1 roslaunch azure_kinect_ros_driver driver.launch color_resolution:=2160P depth_mode:=NFOV_2X2BINNED fps:=5 tf_prefix:=azure1_
$ roslaunch assembly_camera_manager single_azure_manager.launch 
```
2. Get camera pose from single marker
```
$ rosservice call /azure1/get_camera_pose_single_marker \
    "{publish_worldmap: true, target_id: 0, n_frame: 10, \
      img_err_thresh: 0.01, obj_err_thresh: 0.01}" 
```
3. Get camera pose from multiple marker
```
$ rosservice call /azure1/get_camera_pose_multiple_marker \
    "{publish_worldmap: true, target_ids: [0, 2, 4, 6, 8], n_frame: 10, \
    img_err_thresh: 0.01, obj_err_thresh: 0.01}" 
```
4. Set camera pose from yaml
```
$ rosservice call /azure1/set_camera_pose "json_file: 'map_to_azure1_rgb_camera_link_20201102-183839'"
```


### Zivid 
1. Launch camera node and manager
```
$ roslaunch assembly_camera_manager zivid_manager.launch capture_time:=3 repeat:=true
```
2. Get camera pose from single marker
```
$ rosservice call /zivid_camera/get_camera_pose_single_marker \
    "{publish_worldmap: true, target_id: 4, n_frame: 1, \
      img_err_thresh: 0.02, obj_err_thresh: 0.01}" 
```
3. Get camera pose from multiple marker
```
$ rosservice call /zivid_camera/get_camera_pose_multiple_marker \
    "{publish_worldmap: true, target_ids: [0, 2, 4, 6, 8], n_frame: 10, \
    img_err_thresh: 0.01, obj_err_thresh: 0.01}" 
```
4. Set camera pose from yaml
```
$ rosservice call /zivid_camera/set_camera_pose "json_file: 'map_to_zivid_camera_rgb_camera_link_20201102-183839'"
```

### Notes

- build package with catkin_make -DCMAKE_BUILD_TYPE=Release for [better performance](https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/70)

### Authors

* **Seunghyeok Back** [seungback](https://github.com/SeungBack)

### License
This project is licensed under the MIT License

### Acknowledgments
This work was supported by Institute for Information & Communications Technology Promotion(IITP) grant funded by Korea goverment(MSIT) (No.2019-0-01335, Development of AI technology to generate and validate the task plan for assembling furniture in the real and virtual environment by understanding the unstructured multi-modal information from the assembly manual.






