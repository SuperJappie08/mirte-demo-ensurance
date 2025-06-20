# proj_pts_location

## Overview

`proj_pts_location` is a ROS 2 package that computes the **3D position of an apple** given a 2D bounding box from an RGB image, a depth image, and the camera parameters (intrinsics and extrinsics).

---

## Features

- Converts 2D bounding box detections into 3D points.
- Uses depth data and camera intrinsics to compute the position.
- Transforms the 3D point into the `map` frame using TF.
- Publishes a `geometry_msgs::Point` and a `visualization_msgs::Marker` to visualize in RViz.

---

## Inputs & Outputs

### Subscribed Topics

| Topic                         | Type                               | Description                          |
|------------------------------|------------------------------------|--------------------------------------|
| `/apple_detection/bbox_coords` | `sensor_msgs/msg/RegionOfInterest` | Bounding box around detected apple   |
| `/camera/depth/image_raw`    | `sensor_msgs/msg/Image`            | Aligned depth image                  |

### Published Topics

| Topic                    | Type                           | Description                           |
|--------------------------|--------------------------------|---------------------------------------|
| `/point_3D`              | `geometry_msgs/msg/Point`      | 3D location of the apple in `map` frame |
| `/visualization_marker` | `visualization_msgs/msg/Marker`| RViz marker for 3D point visualization |

---

## Camera Intrinsics Configuration

The camera intrinsics of MIRTE Master robot are saved in `camera_intrinsics.yaml`.

## Launching the node

To run the node separately, use the following command:

```bash
ros2 run proj_pts_location point_locator --ros-args -p image_transport:=compressedDepth
```

The argument specifies that the node needs the compressed version of the depth image.