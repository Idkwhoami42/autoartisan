import rclpy
from rclpy.node import Node
from flask import Flask, jsonify, request
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import queue
from realsense2_camera_msgs.msg import RGBD
from .utils import process_frames


class FlaskNode(Node):
    def __init__(self):
        super().__init__("flask_node")
        self.get_logger().info("FlaskNode is starting...")
        self.app = Flask(__name__)
        self.setup_routes()
        self.server_thread = Thread(target=self.run_flask)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.images = queue.Queue()
        self.detection_images = queue.Queue()
        self.last_image_time = 0

        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            RGBD, '/camera/camera/rgbd', self.callback, 120
        )

        # self.color_sub = self.create_subscription(
        #     Image, "/camera/camera/color/image_rect_raw", self.color_callback, 10
        # )

        # self.detection_sub = self.create_subscription(
        #     Image, "/camera/camera/color/detection", self.detection_callback, 10
        # )


    def setup_routes(self):
        @self.app.route("/")
        def index():
            return jsonify({"message": "Welcome to the Flask ROS 2 Node!"})

        @self.app.route("/latest_image")
        def images():
            return jsonify([self.images.get(), self.detection_images.get()])

    def run_flask(self):
        self.app.run(host="0.0.0.0", port=5000)

    def color_callback(self, msg: Image):

        if msg.header.stamp.sec - self.last_image_time < 1:
            return

        self.last_image_time = msg.header.stamp.sec

        try:
            cv_image = self.ros_to_cv2(msg)

            # Encode image to JPEG
            _, buffer = cv2.imencode(".jpg", cv_image)

            self.images.put(
                f"data:image/jpeg;base64,{base64.b64encode(buffer).decode('utf-8')}"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

        return

    def detection_callback(self, msg: Image):

        try:
            cv_image = self.ros_to_cv2(msg)
            _, buffer = cv2.imencode(".jpg", cv_image)

            self.detection_images.put(
                f"data:image/jpeg;base64,{base64.b64encode(buffer).decode('utf-8')}"
            )
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

        return

    def ros_to_cv2(self, ros_image):
        if ros_image.encoding == "rgb8":
            cv_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                ros_image.height, ros_image.width, 3
            )
        elif ros_image.encoding == "bgr8":
            cv_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape(
                ros_image.height, ros_image.width, 3
            )
        else:
            raise ValueError(f"Unsupported encoding: {ros_image.encoding}")

        return cv_image

    def callback(self, msg: RGBD):
        
        color_frame, depth_frame = msg.rgb, msg.depth
        
        color_frame = self.bridge.imgmsg_to_cv2(color_frame, "passthrough")
        depth_frame = self.bridge.imgmsg_to_cv2(depth_frame, "passthrough")

        c =  color_frame
        d = process_frames(depth_frame.copy(), color_frame.copy())

        _, buffer1 = cv2.imencode(".jpg", c)
        _, buffer2 = cv2.imencode(".jpg", d)

        self.images.put(
            f"data:image/jpeg;base64,{base64.b64encode(buffer1).decode('utf-8')}"
        )
        self.detection_images.put(
            f"data:image/jpeg;base64,{base64.b64encode(buffer2).decode('utf-8')}"
        )

        return


def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskNode()

    try:
        rclpy.spin(flask_node)
    except KeyboardInterrupt:
        flask_node.get_logger().info("Shutting down FlaskNode...")
    finally:
        flask_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
