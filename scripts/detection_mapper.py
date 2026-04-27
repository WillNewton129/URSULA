#!/usr/bin/env python3
"""
detection_mapper.py

Subscribes to /oak/nn/detections (vision_msgs/Detection2DArray) and the
/tf tree (map→base_link), then publishes a MarkerArray so detected objects
appear as labelled cylinders on the SLAM map in Foxglove's 3D panel.

In Foxglove:
  - Add a Markers display to the 3D panel
  - Set topic to /ursula/detection_markers
  - Each detected object appears as a coloured cylinder at the robot's
    map position at time of detection, with a text label above it.

Note: markers are placed at the *robot's* map position, not a ray-cast
estimate of the object's position.  This is a reasonable approximation
for objects that are close (within 3 m) and the camera is forward-facing.
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException

# COCO 80-class labels (YOLOv4-tiny default)
COCO_CLASSES = [
    'person','bicycle','car','motorbike','aeroplane','bus','train','truck',
    'boat','traffic light','fire hydrant','stop sign','parking meter','bench',
    'bird','cat','dog','horse','sheep','cow','elephant','bear','zebra',
    'giraffe','backpack','umbrella','handbag','tie','suitcase','frisbee',
    'skis','snowboard','sports ball','kite','baseball bat','baseball glove',
    'skateboard','surfboard','tennis racket','bottle','wine glass','cup',
    'fork','knife','spoon','bowl','banana','apple','sandwich','orange',
    'broccoli','carrot','hot dog','pizza','donut','cake','chair','sofa',
    'pottedplant','bed','diningtable','toilet','tvmonitor','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush',
]


class DetectionMapper(Node):
    def __init__(self):
        super().__init__('detection_mapper')

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/ursula/detection_markers', 10
        )

        self.create_subscription(
            Detection2DArray,
            '/oak/nn/detections',
            self._detection_cb,
            10,
        )

        self.marker_id = 0
        self.get_logger().info('Detection mapper started — listening on /oak/nn/detections')

    def _detection_cb(self, msg: Detection2DArray):
        if not msg.detections:
            return

        # Look up robot position in map frame
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except TransformException:
            return

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        marker_array = MarkerArray()

        for det in msg.detections:
            if not det.results:
                continue

            best    = max(det.results, key=lambda r: r.hypothesis.score)
            score   = best.hypothesis.score
            if score < 0.5:
                continue

            try:
                class_id   = int(best.hypothesis.class_id)
                class_name = COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES) else str(class_id)
            except (ValueError, IndexError):
                class_name = best.hypothesis.class_id

            # Deterministic colour per class
            r = (class_id * 37  % 256) / 255.0
            g = (class_id * 97  % 256) / 255.0
            b = (class_id * 173 % 256) / 255.0

            now = self.get_clock().now().to_msg()

            # Cylinder marker
            cyl = Marker()
            cyl.header.frame_id = 'map'
            cyl.header.stamp    = now
            cyl.ns              = 'detections'
            cyl.id              = self.marker_id;  self.marker_id += 1
            cyl.type            = Marker.CYLINDER
            cyl.action          = Marker.ADD
            cyl.pose.position.x = robot_x
            cyl.pose.position.y = robot_y
            cyl.pose.position.z = 0.15
            cyl.pose.orientation.w = 1.0
            cyl.scale.x         = 0.25
            cyl.scale.y         = 0.25
            cyl.scale.z         = 0.30
            cyl.color.a         = 0.75
            cyl.color.r         = r
            cyl.color.g         = g
            cyl.color.b         = b
            cyl.lifetime.sec    = 0   # Persistent until node restarts

            # Text label
            txt = Marker()
            txt.header          = cyl.header
            txt.ns              = 'detection_labels'
            txt.id              = self.marker_id;  self.marker_id += 1
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = robot_x
            txt.pose.position.y = robot_y
            txt.pose.position.z = 0.55
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.18
            txt.color.a         = 1.0
            txt.color.r         = 1.0
            txt.color.g         = 1.0
            txt.color.b         = 1.0
            txt.text            = f'{class_name} {score:.0%}'
            txt.lifetime.sec    = 0

            marker_array.markers.extend([cyl, txt])
            self.get_logger().info(
                f'Detected {class_name} ({score:.0%}) at map ({robot_x:.2f}, {robot_y:.2f})'
            )

        if marker_array.markers:
            self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()