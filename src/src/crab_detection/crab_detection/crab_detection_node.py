import os
from collections import Counter

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO

# ---------------- Settings ----------------
INPUT_TOPIC = '/oak/image_raw'        # OAK camera topic (change if  differs)
OUTPUT_TOPIC = '/crab_detections/image'   
MODEL_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'models', 'model-3.pt')
THRESHOLD = 0.6
PUBLISH_EVERY = 1
TARGET_CLASS_ID = 0      # class '0' only (European green crab)
# ------------------------------------------


def cv2_to_imgmsg(frame, header=None):
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(frame).tobytes()
    return msg


def imgmsg_to_cv2(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3).copy()
    if msg.encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class CrabDetectionNode(Node):

    def __init__(self):
        super().__init__('crab_detection_node')

        self.model = YOLO(os.path.normpath(MODEL_PATH))

        self.frame_counter = 0
        self.window_counts = []
        self.window_frames = {}
        self.last_header = None

        self.camera_sub = self.create_subscription(Image, INPUT_TOPIC, self.camera_callback, qos_profile_sensor_data)

        self.image_pub = self.create_publisher(Image, OUTPUT_TOPIC, 10)

        self.get_logger().info(f'Model: {os.path.normpath(MODEL_PATH)}')
        self.get_logger().info(f'Listening on {INPUT_TOPIC}, publishing on {OUTPUT_TOPIC}')

    def camera_callback(self, msg):

        frame = imgmsg_to_cv2(msg)

        results = self.model.predict(source=frame,conf=THRESHOLD,classes=[TARGET_CLASS_ID],verbose=False)
        result = results[0]

        crab_count = len(result.boxes)
        annotated = result.plot(labels=True, conf=True)

        self.window_counts.append(crab_count)
        self.window_frames[crab_count] = annotated
        self.last_header = msg.header
        self.frame_counter += 1

        if self.frame_counter < PUBLISH_EVERY:
            return

        stable_count = Counter(self.window_counts).most_common(1)[0][0]
        out_frame = self.window_frames[stable_count].copy()

        text = f'European Green Crabs: {stable_count}'
        pos = (10, 35)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(out_frame, text, pos, font, 1.0, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(out_frame, text, pos, font, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

        self.image_pub.publish(cv2_to_imgmsg(out_frame, header=self.last_header))

        self.frame_counter = 0
        self.window_counts.clear()
        self.window_frames.clear()


def main(args=None):
    rclpy.init(args=args)
    node = CrabDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()