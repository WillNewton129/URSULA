#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# COCO labels for YOLOv4 tiny (80 classes)
COCO_LABELS = [
    'person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train',
    'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
    'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
    'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
    'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
    'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
    'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
    'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
    'sofa', 'pottedplant', 'bed', 'diningtable', 'toilet', 'tvmonitor',
    'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
    'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

GREEN = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
GREEN_BGR = (0, 255, 0)


class DetectionVisualiser(Node):

    def __init__(self):
        super().__init__('detection_visualiser')

        self.bridge = CvBridge()
        self.latest_detections = None

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            TrackDetection2DArray,
            '/color/yolov4_Spatial_tracklets',
            self.detection_callback,
            10
        )

        # Subscribe to camera image
        self.image_sub = self.create_subscription(
            Image,
            '/color/image',
            self.image_callback,
            10
        )

        # Publisher for 3D markers (RViz2)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/detection_markers',
            10
        )

        # Publisher for annotated image (rqt_image_view / Foxglove)
        self.image_pub = self.create_publisher(
            Image,
            '/detection/image',
            10
        )

        self.get_logger().info('Detection visualiser started')

    def detection_callback(self, msg):
        # Store latest detections for use in image callback
        self.latest_detections = msg

        # Publish 3D markers as before
        marker_array = MarkerArray()
        marker_id = 1000

        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.header = msg.header
        clear_marker.ns = 'detections'
        marker_array.markers.append(clear_marker)

        for detection in msg.detections:
            if not detection.results:
                continue

            best = max(detection.results, key=lambda r: r.hypothesis.score)
            score = best.hypothesis.score

            if score < 0.30:
                continue

            try:
                class_id = int(best.hypothesis.class_id)
                label = COCO_LABELS[class_id] if class_id < len(COCO_LABELS) else str(class_id)
            except (ValueError, IndexError):
                label = best.hypothesis.class_id

            pos = best.pose.pose.position

            sphere = Marker()
            sphere.header = msg.header
            sphere.ns = 'detections'
            sphere.id = marker_id
            marker_id += 1
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pos.x
            sphere.pose.position.y = pos.y
            sphere.pose.position.z = pos.z
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.1
            sphere.scale.y = 0.1
            sphere.scale.z = 0.1
            sphere.color = GREEN
            sphere.lifetime.sec = 1
            marker_array.markers.append(sphere)

            text = Marker()
            text.header = msg.header
            text.ns = 'detections'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos.x
            text.pose.position.y = pos.y
            text.pose.position.z = pos.z + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.1
            text.color = GREEN
            text.text = f'{label} {score:.0%}'
            text.lifetime.sec = 1
            marker_array.markers.append(text)

            self.get_logger().info(
                f'Detected: {label} ({score:.0%}) at x={pos.x:.2f} y={pos.y:.2f} z={pos.z:.2f}m'
            )

        self.marker_pub.publish(marker_array)

    def image_callback(self, msg):
        if self.latest_detections is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        h, w = frame.shape[:2]

        for detection in self.latest_detections.detections:
            if not detection.results:
                continue

            best = max(detection.results, key=lambda r: r.hypothesis.score)
            score = best.hypothesis.score

            if score < 0.3:
                continue

            try:
                class_id = int(best.hypothesis.class_id)
                label = COCO_LABELS[class_id] if class_id < len(COCO_LABELS) else str(class_id)
            except (ValueError, IndexError):
                label = best.hypothesis.class_id

            # Bounding box in pixel coordinates
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            bw = detection.bbox.size_x
            bh = detection.bbox.size_y

            x1 = int(cx - bw / 2)
            y1 = int(cy - bh / 2)
            x2 = int(cx + bw / 2)
            y2 = int(cy + bh / 2)

            # Clamp to image bounds
            x1 = max(0, min(x1, w))
            y1 = max(0, min(y1, h))
            x2 = max(0, min(x2, w))
            y2 = max(0, min(y2, h))

            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), GREEN_BGR, 2)

            # Draw label background and text
            label_text = f'{label} {score:.0%}'
            (text_w, text_h), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(frame, (x1, y1 - text_h - 8), (x1 + text_w, y1), GREEN_BGR, -1)
            cv2.putText(frame, label_text, (x1, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'publish error: {e}')

       # cv2.namedWindow("Object Tracking", cv2.WINDOW_NORMAL)
       # cv2.resizeWindow("Object Tracking", 1280, 800)
       # cv2.imshow("Object Tracking", frame)
       # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualiser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
