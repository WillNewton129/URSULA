#!/usr/bin/env python3
"""
mock_detector.py

Simulation stand-in for tracker_yolov4_spatial_node.

Publishes synthetic Detection2DArray messages on /oak/nn/detections so that
detection_mapper.py can be verified end-to-end in Gazebo without OAK-D hardware.

What it does:
  - Waits for the image topic to confirm the Gazebo camera plugin is running.
  - Publishes one randomised detection every (1/publish_hz) seconds.
  - Cycles through a small set of COCO classes so you can see different
    coloured markers appearing on the map in Foxglove.
  - Logs each publication so you can confirm it in the terminal.

Parameters (set from launch file):
  publish_hz    (float, default 0.2)  — detections per second (0.2 = every 5 s)
  image_topic   (str)                 — topic to watch to confirm camera is live
  use_sim_time  (bool)                — must match the rest of the sim stack

Topics published:
  /oak/nn/detections  (vision_msgs/Detection2DArray)

Topics subscribed:
  <image_topic>       (sensor_msgs/Image) — used only to confirm camera is live

To verify it is working:
  ros2 topic echo /oak/nn/detections
  ros2 topic echo /ursula/detection_markers   (after detection_mapper starts)
"""

import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
)


# Small representative subset of COCO classes for variety in sim
_SIM_CLASSES = [
    (0,  'person'),
    (39, 'bottle'),
    (56, 'chair'),
    (63, 'laptop'),
    (67, 'cell phone'),
    (73, 'book'),
]


class MockDetector(Node):
    def __init__(self):
        super().__init__('mock_detector')

        self.declare_parameter('publish_hz',  0.2)
        self.declare_parameter('image_topic', '/oak_d/rgb/image_raw')

        hz          = self.get_parameter('publish_hz').value
        image_topic = self.get_parameter('image_topic').value

        self._camera_live = False
        self._pub = self.create_publisher(
            Detection2DArray, '/oak/nn/detections', 10
        )

        # Watch image topic — only start publishing once camera is up
        self.create_subscription(Image, image_topic, self._image_cb, 1)

        period = 1.0 / max(hz, 0.01)
        self.create_timer(period, self._publish_detection)

        self.get_logger().info(
            f'Mock detector started. Publishing to /oak/nn/detections '
            f'at {hz:.2f} Hz once {image_topic} is live.'
        )

    def _image_cb(self, _msg: Image):
        if not self._camera_live:
            self._camera_live = True
            self.get_logger().info(
                'Gazebo camera confirmed live — mock detections now active.'
            )

    def _publish_detection(self):
        if not self._camera_live:
            return

        class_id, class_name = random.choice(_SIM_CLASSES)
        score = round(random.uniform(0.60, 0.97), 2)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(class_id)
        hyp.hypothesis.score    = score

        det = Detection2D()
        det.header.stamp    = self.get_clock().now().to_msg()
        det.header.frame_id = 'oak_d_rgb_camera_frame'
        det.results.append(hyp)

        # Bounding box centred in a 640x480 image — realistic but arbitrary
        det.bbox.center.position.x = random.uniform(100.0, 540.0)
        det.bbox.center.position.y = random.uniform(80.0,  400.0)
        det.bbox.size_x            = random.uniform(60.0,  200.0)
        det.bbox.size_y            = random.uniform(60.0,  200.0)

        arr = Detection2DArray()
        arr.header = det.header
        arr.detections.append(det)

        self._pub.publish(arr)
        self.get_logger().info(
            f'Mock detection: {class_name} (id={class_id}) '
            f'score={score:.0%}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
