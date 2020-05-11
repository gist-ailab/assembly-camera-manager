# assembly_camera_calibrator
camera calibration tool for furniture assembly project (Kinect Azure and Zivid) using ArUCO marker

## Dependencies

- Azure Kinect python wrapper: https://github.com/brendandburns/py-k4a

## Usage

1. print aruco marker board (imgs/aruco_marker_board.pdf)
2. capture 50 images of marker board 
```
python src/capture_frame_azure.py {file_name}
```
3. camera calibration
```
python src/calibrate_azure.py
```

## Current Features

- Intrinsic Calibartion for Azure 

## TODO

- Extrinsic Calibration between Azures
- Object Pose Estimation 
- Support for Zivid

## References

- [Aruco_Tracker](https://github.com/njanirudh/Aruco_Tracker)
- Aruco calibration: https://github.com/abhishek098/camera_calibration
- https://www.learnopencv.com/augmented-reality-using-aruco-markers-in-opencv-c-python/




