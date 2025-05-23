import threading
import rclpy
# from mason_test.srv import Float
import time
import subprocess
from std_srvs.srv import Empty
from rclpy.node import Node
from flask import Flask, Response, jsonify, request
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import base64
import queue
from realsense2_camera_msgs.msg import RGBD
from std_msgs.msg import Float32MultiArray, Int32
from .utils import process_frames
import json


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
        
        self.time_pos_pair = [time.time(), np.nan, np.nan]
        self.capacity = 0
        self.percentage = "Capacity: - "

        self.bridge = CvBridge()

        self.image_lock = threading.Lock()
        self.image = ""
        self.detection_image = ""
        
        self.subscription = self.create_subscription(
            RGBD, '/camera/camera/rgbd', self.callback, 3
        )
        
        # self.stop_client_x = self.create_client(Empty, 'odrive_stop_x', 10)
        # self.stop_client_y = self.create_client(Empty, 'odrive_stop_y', 10)
        # self.pos_subscription = self.create_subscription(
        #     Float32MultiArray, 'odrive_position', self.position_callback, 10
        # )
        # self.capacity_sub = self.create_subscription(
        #     Int32, 'capacity', self.capacity_callback, 10
        # )
        # self.percentage_sub = self.create_subscription(
        #     Int32, 'capacity_percentage', self.perc_callback, 10
        # )

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
            def stream():
                # while True:
                #     if not self.images.empty() and not self.detection_images.empty():
                #         img1 = self.images.get()
                #         img2 = self.detection_images.get()
                #         yield f"data: {json.dumps({
                #             "image": img1,
                #             "detection": img2
                #         })}\n\n"
                #         print("Sent image")
                #         time.sleep(0.1)
                #     else:
                #         time.sleep(2)
                while True:
                    with self.image_lock:
                        if self.image != "" and self.detection_image != "":
                            yield f"data: {json.dumps({
                                "image": self.image,
                                "detection": self.detection_image
                            })}\n\n"
                            self.image = ""
                            self.detection_image = ""
                            print("Sent image")
                    time.sleep(0.1)

            return Response(stream(), mimetype="text/event-stream")

        # @self.app.route("/resovoir")
        # def resovoir():
        #     return Response()
        
        # @self.app.route("/stop", methods=["POST"])
        # def stop_x_y():
        #     req = Empty.Request()
        #     while not self.stop_client_x.wait_for_service(timeout_sec=1.0) or self.stop_client_y.wait_for_service(timeout_sec=1.0):
        #         self.get_logger().info('Stopping service not available, waiting. . .')
        #     self.stop_client_x.call_async(req)
        #     self.stop_client_y.call_async(req)
        #     time.sleep(1)
        #     if (time.time() - self.time_pos_pair[0] < 1):
        #         self.get_logger().info(f'Stopped at ({self.time_pos_pair[1]}, {self.time_pos_pair[2]})')
        #     return jsonify({"message" : f'Stopped at ({self.time_pos_pair[1]}, {self.time_pos_pair[2]})'})
        
        # @self.app.route("/start", methods=["POST"])
        # def start():
        #     self.start_ros_node()
        #     return jsonify({"message" : "Mason started"})

    def run_flask(self):
        self.app.run(host="0.0.0.0", port=5000)
        
    # def position_callback(self, msg: Float32MultiArray):
    #     self.time_pos_pair = [time.time(), msg.data[0], msg.data[1]]
    
    # def capacity_callback(self, msg: Int32):
    #     self.capacity = msg.data
        
    # def perc_callback(self, msg: Int32):
    #     self.percentage = msg.data
        
    # def start_ros_node():
    #     subprocess.Popen(["ros2", "launch", "mason_control", "mason_control.launch.py"])

    def callback(self, msg: RGBD):

        if msg.header.stamp.sec - self.last_image_time < 1:
            return

        with self.image_lock:
            self.last_image_time = msg.header.stamp.sec
            
            color_frame, depth_frame = msg.rgb, msg.depth
            
            color_frame = self.bridge.imgmsg_to_cv2(color_frame, "passthrough")
            depth_frame = self.bridge.imgmsg_to_cv2(depth_frame, "passthrough")

            c =  color_frame
            d = process_frames(depth_frame.copy(), color_frame.copy())

            _, buffer1 = cv2.imencode(".jpg", c)
            _, buffer2 = cv2.imencode(".jpg", d)

            # self.images.put(
            #     f"data:image/jpeg;base64,{base64.b64encode(buffer1).decode('utf-8')}"
            # )
            # self.detection_images.put(
            #     f"data:image/jpeg;base64,{base64.b64encode(buffer2).decode('utf-8')}"
            # )
            self.image = f"data:image/jpeg;base64,{base64.b64encode(buffer1).decode('utf-8')}"
            self.detection_image = f"data:image/jpeg;base64,{base64.b64encode(buffer2).decode('utf-8')}"
            print("Setting image")



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
