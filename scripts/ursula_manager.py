#!/usr/bin/env python3
"""
ursula_manager.py

High-level robot management node.  Provides ROS 2 services that can be
called from Foxglove's Service Call panel without SSHing into the Jetson.

CHANGES FROM PREVIOUS VERSION:
  - map_save_dir default changed to ~/ros2_ws/maps (survives reboots, unlike /tmp)
  - After saving, automatically converts the .pgm map image to a PNG using
    PIL/Pillow so you can inspect the map visually (open maps/ursula_map.png)
  - PNG is also saved with a timestamp copy so you can compare map quality
    across mapping sessions
  - Added /ursula/map_preview_path topic that publishes the PNG path so
    Foxglove can display it if you add a Raw Messages panel

Services:
  /ursula/save_map     (std_srvs/Trigger)
  /ursula/get_status   (std_srvs/Trigger)

Published topics:
  /ursula/status            (std_msgs/String, 1 Hz)
  /ursula/map_preview_path  (std_msgs/String, latched — publishes path to PNG)
"""

import os
import subprocess
import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String


def _pgm_to_png(pgm_path: str, png_path: str) -> bool:
    """Convert a .pgm occupancy map to a viewable .png.

    Uses PIL/Pillow if available, falls back to imagemagick convert,
    falls back to a raw byte read for PGM P5 format.
    Returns True on success.
    """
    # Method 1: Pillow (most reliable)
    try:
        from PIL import Image
        img = Image.open(pgm_path)
        img.save(png_path)
        return True
    except ImportError:
        pass
    except Exception as e:
        pass  # Try next method

    # Method 2: imagemagick
    try:
        result = subprocess.run(
            ['convert', pgm_path, png_path],
            capture_output=True, timeout=10
        )
        if result.returncode == 0:
            return True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass

    # Method 3: Manual PGM P5 read (no deps)
    try:
        import struct
        with open(pgm_path, 'rb') as f:
            # Read PGM header
            magic = f.readline().strip()
            if magic != b'P5':
                return False
            # Skip comments
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            width, height = map(int, line.split())
            maxval = int(f.readline().strip())
            raw = f.read()

        # Write a minimal PNG
        import zlib
        import struct as st

        def png_chunk(tag, data):
            c = zlib.crc32(tag + data) & 0xffffffff
            return st.pack('>I', len(data)) + tag + data + st.pack('>I', c)

        # Invert slam_toolbox colouring: 205=unknown(grey), 254=free(white), 0=occupied(black)
        pixels = bytearray(raw)
        for i in range(len(pixels)):
            v = pixels[i]
            if v == 205:        # unknown -> mid grey
                pixels[i] = 128
            elif v >= 250:      # free -> white
                pixels[i] = 255
            else:               # occupied -> black
                pixels[i] = 0

        # Build PNG scanlines
        scanlines = b''
        row_bytes = width
        for y in range(height):
            row = pixels[y * row_bytes:(y + 1) * row_bytes]
            scanlines += b'\x00' + bytes(row)

        compressed = zlib.compress(scanlines)

        with open(png_path, 'wb') as f:
            f.write(b'\x89PNG\r\n\x1a\n')
            f.write(png_chunk(b'IHDR',
                st.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)))
            f.write(png_chunk(b'IDAT', compressed))
            f.write(png_chunk(b'IEND', b''))
        return True

    except Exception:
        return False


class UrsulaManager(Node):
    def __init__(self):
        super().__init__('ursula_manager')

        # Default to ~/ros2_ws/maps so maps survive reboots.
        # In sim_full_test_launch.launch.py this is overridden to /tmp/sim_maps
        # (keep /tmp for sim so we don't pollute the real robot maps directory).
        self.declare_parameter(
            'map_save_dir',
            os.path.expanduser('~/ros2_ws/maps')
        )
        self.map_save_dir = os.path.expanduser(
            self.get_parameter('map_save_dir').value
        )
        os.makedirs(self.map_save_dir, exist_ok=True)

        self.mode = 'running'
        self.last_png_path = ''

        # ── Status publisher (1 Hz) ────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/ursula/status', 10)
        self.create_timer(1.0, self._publish_status)

        # ── Map preview path publisher (latched so late subscribers get it) ──
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.preview_pub = self.create_publisher(
            String, '/ursula/map_preview_path', latched_qos
        )

        # ── Services ──────────────────────────────────────────────────
        self.create_service(Trigger, '/ursula/save_map',   self._save_map_cb)
        self.create_service(Trigger, '/ursula/get_status', self._get_status_cb)

        self.get_logger().info(
            f'URSULA Manager started.\n'
            f'  Map save dir : {self.map_save_dir}\n'
            f'  After saving, open the .png preview from that directory.\n'
            f'  To verify localization loaded correctly:\n'
            f'    ros2 param get /slam_toolbox map_file_name'
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
        """Save the current map via slam_toolbox's SaveMap service.

        Saves two copies:
          ursula_map_<timestamp>  — timestamped archive
          ursula_map              — 'latest' copy used by localization mode

        Also converts both .pgm files to .png for visual inspection.
        Open ~/ros2_ws/maps/ursula_map.png (or /tmp/sim_maps/ursula_map.png
        in simulation) in any image viewer to see the saved map.
        """
        timestamp   = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        ts_path     = os.path.join(self.map_save_dir, f'ursula_map_{timestamp}')
        latest_path = os.path.join(self.map_save_dir, 'ursula_map')

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

        # Also save the 'latest' copy used by localization mode
        call_save(latest_path)

        # ── PNG preview generation ────────────────────────────────────
        # slam_toolbox saves <path>.pgm and <path>.yaml
        # We convert the .pgm to a .png for easy visual inspection.
        png_path = latest_path + '.png'
        ts_png   = ts_path + '.png'
        pgm_path = latest_path + '.pgm'

        # Give slam_toolbox 1 second to flush the file to disk
        import time
        time.sleep(1.0)

        preview_msg = ''
        if os.path.exists(pgm_path):
            ok_png = _pgm_to_png(pgm_path, png_path)
            if ok_png:
                # Also save a timestamped PNG for comparison
                try:
                    import shutil
                    shutil.copy2(png_path, ts_png)
                except Exception:
                    pass

                self.last_png_path = png_path
                preview_msg = f'\nMap PNG preview: {png_path}'

                # Publish path so Foxglove can pick it up
                path_msg = String()
                path_msg.data = png_path
                self.preview_pub.publish(path_msg)

                self.get_logger().info(
                    f'Map PNG saved: {png_path}\n'
                    f'  Open with:  eog {png_path}\n'
                    f'              xdg-open {png_path}\n'
                    f'              display {png_path}  (imagemagick)'
                )
            else:
                preview_msg = '\nPNG conversion failed (install Pillow: pip3 install Pillow)'
        else:
            preview_msg = f'\nPGM not found at {pgm_path} (slam_toolbox may still be writing)'

        msg = (
            f'Map saved to:\n'
            f'  {ts_path}.yaml / .pgm\n'
            f'  {latest_path}.yaml / .pgm\n'
            f'To localise on this map, restart with slam_mode:=localization'
            + preview_msg
        )
        self.get_logger().info(msg)
        self.mode = f'map_saved_{timestamp}'
        response.success = True
        response.message = msg
        return response

    def _get_status_cb(self, request, response):
        response.success = True
        response.message = (
            f'mode={self.mode}  '
            f'map_save_dir={self.map_save_dir}  '
            f'last_png={self.last_png_path or "none"}'
        )
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