#!/usr/bin/env python3
"""
ursula_manager.py

High-level robot management node.  Provides ROS 2 services that can be
called from Foxglove's Service Call panel without SSHing into the Jetson.

Services:
  /ursula/save_map     (std_srvs/Trigger)
      Saves the current slam_toolbox map to the map_save_dir directory.
      Creates both a timestamped copy and a 'ursula_map' (latest) copy.
      After saving, switch to localisation mode by restarting the launch
      with slam_mode:=localization.

  /ursula/get_status   (std_srvs/Trigger)
      Returns the current operating mode as the response message string.

Published topics:
  /ursula/status  (std_msgs/String, 1 Hz)
      Plain-text current state.  Add a Raw Messages panel in Foxglove.

Usage:
  Runs automatically when launched from jetson_hardware_launch.launch.py.
  To call from the command line:
    ros2 service call /ursula/save_map std_srvs/srv/Trigger {}
    ros2 service call /ursula/get_status std_srvs/srv/Trigger {}
  To call from Foxglove:
    Add a Service Call panel, select the service, click Call.
"""

import os
import subprocess
import datetime

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


class UrsulaManager(Node):
    def __init__(self):
        super().__init__('ursula_manager')

        self.declare_parameter('map_save_dir', '/home/uoljetson/maps')
        self.map_save_dir = self.get_parameter('map_save_dir').value
        os.makedirs(self.map_save_dir, exist_ok=True)

        self.mode = 'running'

        # ── Status publisher ──────────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/ursula/status', 10)
        self.create_timer(1.0, self._publish_status)

        # ── Services ─────────────────────────────────────────────────────
        self.create_service(Trigger, '/ursula/save_map',   self._save_map_cb)
        self.create_service(Trigger, '/ursula/get_status', self._get_status_cb)

        self.get_logger().info(
            f'URSULA Manager started. Map save dir: {self.map_save_dir}'
        )

    # ----------------------------------------------------------------
    # Timer
    # ----------------------------------------------------------------
    def _publish_status(self):
        msg = String()
        msg.data = self.mode
        self.status_pub.publish(msg)

    # ----------------------------------------------------------------
    # Service callbacks
    # ----------------------------------------------------------------
    def _save_map_cb(self, request, response):
        """Save the current map via slam_toolbox's SaveMap service."""
        timestamp    = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        ts_path      = os.path.join(self.map_save_dir, f'ursula_map_{timestamp}')
        latest_path  = os.path.join(self.map_save_dir, 'ursula_map')

        self.get_logger().info(f'Saving map to: {ts_path}')

        def call_save(path):
            cmd = [
                'ros2', 'service', 'call',
                '/slam_toolbox/save_map',
                'slam_toolbox/srv/SaveMap',
                f'{{name: {{data: "{path}"}}}}'
            ]
            try:
                result = subprocess.run(
                    cmd, capture_output=True, text=True, timeout=15
                )
                return result.returncode == 0, result.stderr
            except subprocess.TimeoutExpired:
                return False, 'Timeout waiting for slam_toolbox service'
            except Exception as e:
                return False, str(e)

        ok, err = call_save(ts_path)
        if not ok:
            msg = f'Map save FAILED: {err}'
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        # Also save as 'ursula_map' (latest — used by localization mode)
        call_save(latest_path)

        msg = (
            f'Map saved to {ts_path} '
            f'and {latest_path}. '
            f'To localise on this map, restart with slam_mode:=localization'
        )
        self.get_logger().info(msg)
        self.mode = f'map_saved_{timestamp}'
        response.success = True
        response.message = msg
        return response

    def _get_status_cb(self, request, response):
        response.success = True
        response.message = self.mode
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UrsulaManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()