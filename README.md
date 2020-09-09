# Assembly Camera Manager

Camera manager for furniture assembly project (Kinect Azure and Zivid) using ArUCO marker

### Features

- Receive the sensor value of Zivid and Kinect Azure 
- Intrinsic & Extrinsic Calibration for Kinect Azure
- Extrinsic Calibration for Zivid (ExtrinsicCalibrateZivid.srv)
- tf manager for map and cameras

## To Do
- service for extrinsic calibration of Kinect Azure

## Getting Started

### Prerequisites

- Azure Kinect driver & ROS Wrapper
- Intrinsic Calibration: Azure Kinect python wrapper: https://github.com/brendandburns/py-k4a
- fiducials: https://github.com/SeungBack/fiducials

### Zivid
```
$ roslaunch assembly_camera_manager zivid_manager.launch
$ rosservice call /zivid_camera/extrinsic_calibration
```

### Kinect Azure
#### Single Camera
```
$ ROS_NAMESPACE=azure1 roslaunch azure_kinect_ros_driver driver.launch color_resolution:=1536P depth_mode:=WFOV_UNBINNED fps:=5  tf_prefix:=azure1_

$ roslaunch assembly_camera_manager single_azure_manager.launch 

$ rosservice call /triple_azure/extrinsic_calibration "target_fiducial_ids: [0, 1, 3]"
```

#### triple camera
```
$ ROS_NAMESPACE=azure3 roslaunch azure_kinect_ros_driver driver.launch sensor_sn:=000880594512 wired_sync_mode:=2 subordinate_delay_off_master_usec:=500 fps:=5 color_resolution:=720P depth_mode:=NFOV_2X2BINNED tf_prefix:=azure3_ rgb_point_cloud:=true

$ ROS_NAMESPACE=azure2 roslaunch azure_kinect_ros_driver driver.launch sensor_sn:=000853594412 wired_sync_mode:=2 subordinate_delay_off_master_usec:=250 fps:=5 color_resolution:=720P depth_mode:=NFOV_2X2BINNED tf_prefix:=azure2_ rgb_point_cloud:=true

$ ROS_NAMESPACE=azure1 roslaunch azure_kinect_ros_driver driver.launch sensor_sn:=000696793812 wired_sync_mode:=1 color_resolution:=720P depth_mode:=NFOV_2X2BINNED fps:=5 tf_prefix:=azure1_ rgb_point_cloud:=true

# Calibrate Multi K4a Network
$ roslaunch assembly_camera_manager triple_azure_manager.launch
$ rosservice call /triple_azure/extrinsic_calibration "target_fiducial_ids: [0, 1, 23"

```

### Notes

- build package with catkin_make -DCMAKE_BUILD_TYPE=Release for better performance 
- https://github.com/microsoft/Azure_Kinect_ROS_Driver/issues/70


## Authors

* **Seunghyeok Back** [seungback](https://github.com/SeungBack)

## License

This project is licensed under the MIT License

## Acknowledgments

This work was supported by Institute for Information & Communications Technology Promotion(IITP) grant funded by Korea goverment(MSIT) (No.2019-0-01335, Development of AI technology to generate and validate the task plan for assembling furniture in the real and virtual environment by understanding the unstructured multi-modal information from the assembly manual.

## References

- [Aruco_Tracker](https://github.com/njanirudh/Aruco_Tracker)
- [Aruco calibration](https://github.com/abhishek098/camera_calibration)
- [Opencv example](https://www.learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/)
- [Open3d example](https://github.com/intel-isl/Open3D)




