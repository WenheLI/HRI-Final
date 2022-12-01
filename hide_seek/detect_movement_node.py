import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError 
import numpy as np
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist  
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
import rclpy                                    # Python library for ROS 2

class DetectMovementNode(Node):
    def __init__(self):
        super().__init__('detect_movement_node')
        self.get_logger().info("DetectMovementNode has been started")
        self.depth_subscription = self.create_subscription(
            Image, "/stereo/depth", self.depth_callback, 1
        )
        self.depth_image = None
        self.bridge = CvBridge()

        self.detect_subscription = self.create_subscription(
            SpatialDetectionArray, "/color/yolov4_Spatial_detections", self.detect_callback, 5
        )

        self.image_sub = self.create_subscription(
            Image, "/color/image", self.image_callback, 5
        )

        self.movement_pub = self.create_publisher(
            Twist, "/cmd_vel", 1
        )

        self.talker_pub = self.create_publisher(
            String, "/talk", 1
        )
        
        self.tracking_items = {}

        self.move_cmd = Twist()

    def image_callback(self, msg):
        # self.get_logger().info("Image received")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.cv_image is not None:
                cv2.imshow("Image window", self.cv_image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def extract_image_patch(self, image, bbox, patch_shape=(20,20)):
        bbox = np.array(bbox)
        if patch_shape is not None:
            # correct aspect ratio to patch shape
            target_aspect = float(patch_shape[1]) / patch_shape[0]
            new_width = target_aspect * bbox[3]
            bbox[0] -= (new_width - bbox[2]) / 2
            bbox[2] = new_width

        # convert to top left, bottom right
        bbox[2:] += bbox[:2]
        bbox = bbox.astype(np.int)

        # clip at image boundaries
        bbox[:2] = np.maximum(0, bbox[:2])
        bbox[2:] = np.minimum(np.asarray(image.shape[:2][::-1]) - 1, bbox[2:])
        if np.any(bbox[:2] >= bbox[2:]):
            return None
        sx, sy, ex, ey = bbox
        image = image[sy:ey, sx:ex]
        image = cv2.resize(image, tuple(patch_shape[::-1]))
        return image

    def depth_callback(self, msg):
        # self.get_logger().info("Depth image received")
        return 
        try:
            self.depth_imge = self.depth_bridge.imgmsg_to_cv2(msg)
            self.depth_image = cv2.resize(self.depth_image, (640, 480))
            if self.depth_image is not None:
                cv2.imshow("Depth Image", self.depth_image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def detect_callback(self, msg):
        # self.get_logger().info("Detect image received")
        current_tracking = {
            'left': 0,
            'right': 0,
            'middle': 0
        }
        temp_holder = []
        if len(msg.detections) > 0:
            for detection in msg.detections:
                temp = detection.results[0]

                if temp.class_id == '0':
                    # self.get_logger().info("Person detected")
                    bbox = detection.bbox
                    position = detection.position

                    print(bbox.center, bbox.size_x, bbox.size_y)
                    bbox_center_x = bbox.center.x
                    bbox_center_y = bbox.center.y
                    bbox_min_x = bbox.center.x - bbox.size_x / 2
                    bbox_min_y = bbox.center.y - bbox.size_y / 2
                    bbox_max_x = bbox.center.x + bbox.size_x / 2
                    bbox_max_y = bbox.center.y + bbox.size_y / 2
                    print(position.x, position.y, position.z)
                    temp_holder.append([bbox_center_x, bbox_center_y, bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y, position.x, position.y, position.z])
        if (temp_holder.__len__() == 3):
            # get left tracking item and right
            temp_holder.sort(key=lambda x: x[0])
            left = temp_holder[0]
            middle = temp_holder[1]
            right = temp_holder[2]

            current_tracking['left'] = left
            current_tracking['right'] = right
            current_tracking['middle'] = middle

        if self.tracking_items.__len__() != 0:
            for key in self.tracking_items.keys():

                curr_left = current_tracking[key]
                prev_left = self.tracking_items[key]
                if abs((curr_left[-3] - prev_left[-3])) > 0.01:
                    # some action
                    self.get_logger().info(key + " person moved")
                if abs((curr_left[-2] - prev_left[-2])) > 0.01:
                    # some action
                    self.get_logger().info(key + " person moved")
                if abs((curr_left[-1] - prev_left[-1])) > 0.01:
                    # some action
                    self.get_logger().info(key + " person moved")
        
#
def main():
    rclpy.init()
    detect_movement_node = DetectMovementNode()
    rclpy.spin(detect_movement_node)
    detect_movement_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
