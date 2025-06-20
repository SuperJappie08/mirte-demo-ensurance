# Detection 3D Apples

## Overview

The **Apple Detection Node** is a ROS 2 service-based node that captures an RGB image from a RealSense or compatible camera and performs real-time apple detection using a YOLOv8 object detection model. The node supports both raw and compressed image topics and publishes the bounding box of the most likely apple in the frame. It can be triggered on-demand using a custom ROS 2 service.

## Features

- **One-shot Image Capture**: Subscribes once to the camera feed and processes a single frame upon receiving a service request.
- **YOLOv8 Integration**: Runs object detection using a pre-trained YOLO model, configurable via parameter.
- **Apple Filtering**: Applies red-color dominance heuristics to improve apple detection robustness.
- **Bounding Box Publisher**: Publishes the Region of Interest (ROI) of the detected apple.
- **Snapshot Logging**: Saves detection result images (with or without apple) to disk.
- **Supports Compressed or Raw Streams**: Works with both `/image_raw` and `/image_raw/compressed`.

## Topics and Services

### Service

- **`/get_point`** (fsm/srv/GetPoint): Triggers a one-shot image capture and detection.

### Publisher

- **`/apple_detection/bbox_coords`** (sensor_msgs/msg/RegionOfInterest): Publishes the coordinates of the best detected apple.

## Parameters

| Name             | Type    | Default                          | Description                                        |
|------------------|---------|----------------------------------|----------------------------------------------------|
| `model_path`     | string  | `"./weights/best.pt"`            | Path to the YOLO model file (e.g., `.pt`)         |
| `image_transport`| string  | `"raw"`                          | Can be set to `"compressed"` for efficiency       |

## Usage

### Building

Make sure your ROS 2 workspace is sourced, and build the package:

```bash
colcon build --packages-select detection_3d_apples
source install/setup.bash

# Run with compressed image transport (recommended):
ros2 run detection_3d_apples detection_node --ros-args -p image_transport:=compressed

- Snapshot Logging: Saves each processed image (with or without detected apple) to `~/apple_detections/` with a timestamp.


