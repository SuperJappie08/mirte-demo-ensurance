#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
from ultralytics import YOLO

import numpy as np
import os
import cv2
import time
from datetime import datetime


class AppleDetectionNode(Node):
    def __init__(self):
        super().__init__('apple_detection_node')

        # Bridge for converting ROS Image/CompressedImage ↔ OpenCV
        self.bridge = CvBridge()

        # Read “image_transport” parameter (defaults to “raw”)
        transport = self.declare_parameter("image_transport", "raw").get_parameter_value().string_value
        self.use_compressed = (transport == "compressed")
        self.get_logger().info(
            f"Using image_transport = '{transport}'. "
            f"Will subscribe to {'compressed' if self.use_compressed else 'raw'} topic when requested."
        )

        # We use the built-in “sensor_data” QoS so that it matches most camera publishers
        self.sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # Publisher for bounding boxes (RegionOfInterest)
        self.bbox_pub = self.create_publisher(
            RegionOfInterest,
            '/apple_detection/bbox_coords',
            10
        )

        # Directory for saving snapshots (with or without a detected box)
        self.out_dir = os.path.expanduser('~/apple_detections')
        os.makedirs(self.out_dir, exist_ok=True)

        # Load the YOLO model (path given via parameter “model_path”)
        model_path = self.declare_parameter(
            'model_path',
            os.path.join(os.path.dirname(__file__), 'weights', 'best.pt')
        ).get_parameter_value().string_value
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)

        # Placeholders for the one-shot image and a flag indicating reception
        self.latest_image_msg = None
        self.image_received = False

        # Create a single ReentrantCallbackGroup that will be used for both:
        #   • handling the service callback
        #   • handling the one-shot subscription callback
        self.cb_group = ReentrantCallbackGroup()

        # Create the GetPoint service inside that callback group
        self.create_service(
            Trigger,
            'get_point',
            self.handle_get_point,
            callback_group=self.cb_group
        )


    def _compressed_callback(self, msg: CompressedImage):
        """
        Called exactly once when the first CompressedImage arrives
        after we subscribe. It simply stores the message and sets the flag.
        """
        if not self.image_received:
            self.get_logger().info("CompressedImage callback triggered (one-shot).")
            self.latest_image_msg = msg
            self.image_received = True


    def _raw_callback(self, msg: Image):
        """
        Called exactly once when the first raw Image arrives
        after we subscribe. It stores the message and sets the flag.
        """
        if not self.image_received:
            self.get_logger().info("Raw Image callback triggered (one-shot).")
            self.latest_image_msg = msg
            self.image_received = True


    def handle_get_point(self, request, response):
        """
        Service handler (runs under the ReentrantCallbackGroup). When called:
          1. Reset image_received flag
          2. Dynamically create a one-shot subscription (under the same callback group)
          3. Spin in a short loop (time.sleep) while waiting for exactly one image to arrive
             (the ReentrantCallbackGroup + MultiThreadedExecutor allow that callback to run)
          4. Destroy the subscription
          5. If no image arrived within 5 seconds → respond success=False
          6. Otherwise decode the image, run YOLO, publish ROI (if found), save snapshot,
             and return success=True/False accordingly.
        """
        # Reset flags
        self.latest_image_msg = None
        self.image_received = False

        if self.use_compressed:
            topic = '/camera/color/image_raw/compressed'
            callback_fn = self._compressed_callback
            msg_type = CompressedImage
            self.get_logger().info("Service call received: subscribing to compressed topic.")
        else:
            topic = '/camera/color/image_raw'
            callback_fn = self._raw_callback
            msg_type = Image
            self.get_logger().info("Service call received: subscribing to raw topic: " + topic)

        # Create a one-shot subscription under the same ReentrantCallbackGroup
        sub = self.create_subscription(
            msg_type,
            topic,
            callback_fn,
            self.sensor_qos,
            callback_group=self.cb_group
        )
        self.get_logger().info(
            f"One-shot subscription to '{topic}' created. Waiting up to 60 seconds for a frame..."
        )

        # Wait up to 5 seconds for exactly one callback to fire
        timeout_sec = 600.0
        start_time = time.time()
        while rclpy.ok() and not self.image_received and (time.time() - start_time) < timeout_sec:
            # Sleep briefly—ReentrantCallbackGroup + MultiThreadedExecutor allow the subscription callback
            # to run on another thread while this service callback is “sleeping.”
            time.sleep(0.05)

        # Immediately destroy that one-shot subscription
        self.destroy_subscription(sub)
        self.get_logger().info("One-shot subscription destroyed.")

        # If no image arrived, respond success=False
        if not self.image_received or self.latest_image_msg is None:
            self.get_logger().warn("No image received within 5 seconds. Returning success=False.")
            response.success = False
            return response

        # Decode the received ROS message into an OpenCV BGR image
        try:
            if self.use_compressed:
                np_arr = np.frombuffer(self.latest_image_msg.data, np.uint8)
                cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                cv_img = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to decode incoming image: {e}")
            response.success = False
            return response

        # Run YOLO on that single frame
        img_copy = cv_img.copy()
        results = self.model(img_copy)[0]
        boxes = results.boxes.xyxy.cpu().numpy()
        confs = results.boxes.conf.cpu().numpy()
        clss = results.boxes.cls.cpu().numpy().astype(int)

        best = None
        for (x1, y1, x2, y2), conf, cls in zip(boxes, confs, clss):
            # Only accept class 0 (apple) with confidence > 0.5
            if cls != 0 or conf <= 0.5:
                continue

            x1i, y1i = max(0, int(x1)), max(0, int(y1))
            x2i = min(img_copy.shape[1] - 1, int(x2))
            y2i = min(img_copy.shape[0] - 1, int(y2))
            if x2i <= x1i or y2i <= y1i:
                continue

            roi = img_copy[y1i:y2i, x1i:x2i]
            if roi.size == 0:
                continue

            mb, mg, mr = cv2.mean(roi)[:3]
            # Simple red-dominance filter
            # if not (mr > mg * 1.2 and mr > mb * 1.2 and mr > 100):
            #     continue

            self.get_logger().info(f"Red={mr:.1f}, Green={mg:.1f}, Blue={mb:.1f}")
            if not (mr > mg * 1.05 and mr > mb * 1.05 and mr > 60):
                continue


            if best is None or conf > best[0]:
                best = (conf, x1i, y1i, x2i, y2i)

        if best:
            _, x1i, y1i, x2i, y2i = best
            width = x2i - x1i
            height = y2i - y1i

            roi_msg = RegionOfInterest()
            roi_msg.x_offset = x1i
            roi_msg.y_offset = y1i
            roi_msg.width = width
            roi_msg.height = height

            self.bbox_pub.publish(roi_msg)
            self.get_logger().info(
                f"Published bbox: x={x1i}, y={y1i}, w={width}, h={height}"
            )
            cv2.rectangle(img_copy, (x1i, y1i), (x2i, y2i), (0, 255, 0), 2)
            snapshot_label = "detected"
            response.success = True
        else:
            self.get_logger().info("No valid apple detected in that frame.")
            snapshot_label = "no_apple"
            response.success = False

        # Save a snapshot (labeled “detected” or “no_apple”)
        filename = os.path.join(
            self.out_dir,
            f"{snapshot_label}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        )
        cv2.imwrite(filename, img_copy)
        self.get_logger().info(f"Saved snapshot: {filename}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = AppleDetectionNode()

    # Use a MultiThreadedExecutor so the subscription callback (in cb_group) can run
    # concurrently with the service callback (also in the same ReentrantCallbackGroup).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
