import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
from sensor_msgs.msg import CompressedImage, BatteryState
from fsm.srv import SendState

from PyQt5 import QtWidgets, uic, QtGui
from PyQt5.QtCore import QTimer
import sys
import os
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSPresetProfiles

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class ROSNode(Node):
    def __init__(self, main_app):
        super().__init__('gui_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.lookup_base_link)

        self.main_app = main_app
        
        # Client for FSM states
        self.send_state_client = self.create_client(SendState, '/send_state')
        while not self.send_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /send_state not available, waiting...')
        
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.latest_image = None


        #--------- Subscriptions ----------

        self.battery_subscription = self.create_subscription(
            BatteryState,
            '/io/power/power_watcher',
            self.battery_callback,
            10
        )

        self.apples_sub = self.create_subscription(
            Int32,
            '/picked_apples_count',
            self.apples_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/fsm_debug',
            self.status_callback,
            10
        )


    # Process incoming battery messages and update GUI
    def battery_callback(self, msg):
        if msg.percentage is not None:
            percentage = msg.percentage * 100
            battery_message = f"Battery: {percentage:.2f}%"
            self.main_app.update_status_item("Battery", battery_message)
        else:
            self.main_app.update_status_item("Battery", "Battery: No information") 

    # Process incoming number of apple messages and update GUI
    def apples_callback(self, msg):
        if msg.data is not None:
            apple_text = f"Picked Apples: {msg.data}"
            self.main_app.update_status_item("Picked Apples", apple_text)
        else:
            self.main_app.update_status_item("Picked Apples", "Picked Apples: No information")

    # Process incoming status messages and update GUI
    def status_callback(self, msg):
        if msg.data is not None:
            if "State:" in msg.data:
                state_line = msg.data.split("State:")[-1].strip()
                status_text = f"Status: {state_line}"
                self.main_app.update_status_item("Status", status_text)
            else:
                self.main_app.update_status_item("Status", "Status: No information")
        else:
            self.main_app.update_status_item("Status", "Status: No information")           

    # Send command to start/stop operation
    def call_send_state(self, command):
        req = SendState.Request()
        req.command = command
        future = self.send_state_client.call_async(req)
        return future

    # Look up robot position
    def lookup_base_link(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(seconds=0))

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            print(f"[TF Position] x={x:.2f}, y={y:.2f}")


            self.main_app.update_robot_position(x, y)
        except Exception as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")


class MainApp:
    def __init__(self):
        rclpy.init()
        self.node = ROSNode(self)

        self.app = QtWidgets.QApplication(sys.argv)

        #------- Windows -------
        ui_path_start = os.path.join(get_package_share_directory('task_gui'), 'ui', 'start_window.ui')
        self.start_window = uic.loadUi(ui_path_start)

        ui_path_main = os.path.join(get_package_share_directory('task_gui'), 'ui', 'main_window.ui')
        self.main_window = uic.loadUi(ui_path_main)

        #------ Button operations -------
        self.start_window.startButton.clicked.connect(self.start_operation)
        self.main_window.stopButton.clicked.connect(self.stop_operation)
        self.main_window.captureButton.clicked.connect(self.capture_image_on_demand)

        #------- Images -------
        terra_crop_path = os.path.join(get_package_share_directory('task_gui'), 'ui/images', 'TerraCrop.png')
        pixmap1 = QtGui.QPixmap(terra_crop_path)
        self.start_window.terraCropLabel.setPixmap(pixmap1)
        self.start_window.terraCropLabel.setScaledContents(True)

        self.main_window.terraCropLabel_2.setPixmap(pixmap1)
        self.main_window.terraCropLabel_2.setScaledContents(True)

        tudelft_path = os.path.join(get_package_share_directory('task_gui'), 'ui/images', 'tu_delft.png')
        pixmap2 = QtGui.QPixmap(tudelft_path)
        self.start_window.tuDelftLabel.setPixmap(pixmap2)
        self.start_window.tuDelftLabel.setScaledContents(True)

        mdp_path = os.path.join(get_package_share_directory('task_gui'), 'ui/images', 'MDPInc.png')
        pixmap3 = QtGui.QPixmap(mdp_path)
        self.start_window.mdpLabel.setPixmap(pixmap3)
        self.start_window.mdpLabel.setScaledContents(True)

        map_path = os.path.join(get_package_share_directory('task_gui'), 'ui/images', 'map_new.png')

        self.map_pixmap = QtGui.QPixmap(map_path)
        self.main_window.mapLabel.setPixmap(self.map_pixmap)
        self.main_window.mapLabel.setScaledContents(True)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(10)

        self.update_status_item("Battery", "Battery: No information") 
        self.update_status_item("Picked Apples", "Picked Apples: No information")
        self.update_status_item("Status", "Status: No information")  

        self.start_window.show()
        self.app.exec_()

        rclpy.shutdown()

    def ui_path(self, filename):
        return os.path.join(os.path.dirname(__file__), 'ui', filename)


    # Start operation using the start button
    def start_operation(self):
        self.node.call_send_state("SCAN_TREE")
        self.start_window.hide()
        self.main_window.show()

    # Stop operation using the Stop button
    def stop_operation(self):
        self.node.call_send_state("REST")
        self.main_window.hide()
        self.start_window.show()


    # Create subscription to camera color image when the button is clicked
    def capture_image_on_demand(self):

        if hasattr(self, 'image_sub') and self.image_sub is not None:
            self.node.get_logger().warn("Image subscription already exists. Skipping new one.")
            return

        self.captured_image = None
        self.image_received = False

        self.image_sub = self.node.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self._one_shot_image_callback,
            QoSPresetProfiles.SENSOR_DATA.value
        )
        self.node.get_logger().info("One-shot image subscription created.")

        self.capture_timer = QTimer()
        self.capture_timer.setSingleShot(True)
        self.capture_timer.timeout.connect(self._handle_image_timeout)
        self.capture_timer.start(3000)


    # Handle received image and destroy subscription
    def _one_shot_image_callback(self, msg):
        if self.image_received:
            return

        self.image_received = True
        self.node.get_logger().info("Received one-shot image.")

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.node.destroy_subscription(self.image_sub)
        self.image_sub = None
        self.capture_timer.stop()

        if image is None:
            self.node.get_logger().warn("Failed to decode image.")
            return

        self.captured_image = image
        self.display_captured_image(image)

        self.capture_timer.stop()


    def _handle_image_timeout(self):
        if not self.image_received:
            self.node.get_logger().warn("Timed out waiting for image.")
            self.node.destroy_subscription(self.image_sub)
            self.image_sub = None


    # Display image on GUI
    def display_captured_image(self, image):
        print(f"Displaying image of size: {image.shape}")
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        pixmap = QtGui.QPixmap.fromImage(qt_image)
        self.main_window.imageLabel.setPixmap(pixmap)


    # Update battery / apple number / status
    def update_status_item(self, label, text):
        list_widget = self.main_window.statusListWidget
        items = [list_widget.item(i).text() for i in range(list_widget.count())]
        found = False
        for i, item_text in enumerate(items):
            if item_text.startswith(label + ":"):
                list_widget.item(i).setText(text)
                found = True
                break
        if not found:
            list_widget.addItem(text)


    # Convert world coordinates to pixel coordinates and draw robot position
    def update_robot_position(self, world_x, world_y):
        map_width_m, map_height_m = 1.86, 3.09
        image_width_px, image_height_px = 800, 1200

        scale_x = image_width_px / map_width_m
        scale_y = image_height_px / map_height_m

        pixel_x = (map_width_m - world_x - 0.2) * scale_x
        pixel_y = (map_height_m + world_y + (-0.55)) * scale_y

        print(f"[DRAWING AT] px: {pixel_x:.1f}, py: {pixel_y:.1f}")

        if self.map_pixmap:
            pixmap_copy = self.map_pixmap.copy()
            painter = QtGui.QPainter(pixmap_copy)
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 0), 4))
            painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 255, 0)))
            painter.drawEllipse(int(pixel_x) - 25, int(pixel_y) - 25, 50, 50)
            painter.end()
            self.main_window.mapLabel.setPixmap(pixmap_copy)
            self.main_window.mapLabel.repaint()
            print("Updated")


def main():
    MainApp()

if __name__ == '__main__':
    main()

