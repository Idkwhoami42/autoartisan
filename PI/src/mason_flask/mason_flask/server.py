# backend.py
import rclpy
from rclpy.node import Node
from flask import Flask, Response
import json
import base64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import threading
from realsense2_camera_msgs.msg import RGBD
from utils import process_frames
import time
import random


app = Flask(__name__)


class FlaskNode(Node):
    def __init__(self):
        super().__init__("image_streamer")

        # Initialize CV bridge and image storage
        self.bridge = CvBridge()

        self.image = None
        self.detection_image = None
        self.last_image_time = 0

        # Create subscription
        self.subscription = self.create_subscription(
            RGBD, "/camera/camera/rgbd", self.image_callback, 10  # QoS proile depth
        )

    def image_callback(self, msg):
        color_frame, depth_frame = msg.rgb, msg.depth

        color_frame = self.bridge.imgmsg_to_cv2(color_frame, "passthrough")
        depth_frame = self.bridge.imgmsg_to_cv2(depth_frame, "passthrough")

        c = color_frame
        d = process_frames(depth_frame.copy(), color_frame.copy())

        _, buffer1 = cv2.imencode(".jpg", c)
        _, buffer2 = cv2.imencode(".jpg", d)

        self.image = (
            f"data:image/jpeg;base64,{base64.b64encode(buffer1).decode('utf-8')}"
        )
        self.detection_image = (
            f"data:image/jpeg;base64,{base64.b64encode(buffer2).decode('utf-8')}"
        )


def ros_spin():
    rclpy.spin(node)


@app.route("/latest_image")
def stream():
    def generate():
        while True:
            if node.image and node.detection_image:
                yield f"data: {json.dumps({'image': node.image, 'detection': node.detection_image})}\n\n"
            time.sleep(0.1)

    return Response(generate(), mimetype="text/event-stream")

@app.route("/resovoir")
def resovoir():
    def stream():
        for i in range(100, -1, -1):
            yield  f"data: {i}\n\n"
            time.sleep(random.random())
    return Response(stream(), mimetype='text/event-stream')

if __name__ == "__main__":
    # Initialize ROS 2
    rclpy.init()

    # Create node
    node = FlaskNode()

    # Start ROS 2 spin in a separate thread
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    # Run Flask app
    app.run(host="0.0.0.0", port=5000, threaded=True)

    # Cleanup
    rclpy.shutdown()
