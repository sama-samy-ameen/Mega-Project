import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rov_interfaces.msg import ObjectData
import cv2
import numpy as np


class RedSquareNode(Node):
    def __init__(self):
        super().__init__('red_square_node')
        self.bridge = CvBridge()
        self.frames = {}  

        self.pub = self.create_publisher(ObjectData, '/model_readings', 10)

        # Subscribe to the 5 cameras
        for i in range(1, 6):
            self.create_subscription(Image,f'/camera{i}/image_raw',lambda msg, cam=i: self.image_callback(msg, cam),10)

        self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg, cam):
        self.frames[cam] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, np.array([0, 60, 30]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 60, 30]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best = None
        best_area = 500  
        for c in contours:
            area = cv2.contourArea(c)
            x, y, w, h = cv2.boundingRect(c)
            is_square = 0.7 < w / h < 1.3
            if is_square and area > best_area:
                best_area = area
                confidence = int(100 * area / (w * h))  
                best = (x, y, w, h, confidence)
        return best

    def timer_callback(self):
        msg = ObjectData()  

        for cam in range(1, 6):
            frame = self.frames.pop(cam, None)
            if frame is None:
                continue

            result = self.detect(frame)
            if result is not None:
                x, y, w, h, conf = result
                msg.x = int(x + w / 2)  
                msg.y = int(y + h / 2)
                msg.width = int(w)
                msg.height = int(h)
                msg.confidence = conf
                msg.camera = cam
                break

        # If nothing was found camera stays 0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RedSquareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()