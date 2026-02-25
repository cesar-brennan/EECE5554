#!/usr/bin/env python3
"""
EECE5554 Lab 2 - RTK GNSS Driver Node (ROS2 Humble)
Reads GNGGA sentences from an RTK GNSS device,
parses them, converts lat/lon to UTM, and publishes a custom RTK GPS message.
"""

import rclpy
from rclpy.node import Node
import serial
import utm
import argparse

from gps_msg.msg import Customrtk


def parse_gngga(line):
    """
    Parse a GNGGA NMEA sentence and return a dictionary of values.
    Handles both $GNGGA and $GPGGA prefixes.
    """
    parts = line.strip().split(',')
    if len(parts) < 15:
        return None
    if 'GGA' not in parts[0]:
        return None

    try:
        fix_quality = int(parts[6])
    except (ValueError, IndexError):
        return None
    if fix_quality == 0:
        return None

    try:
        utc_raw = parts[1]
        utc = float(utc_raw) if utc_raw else 0.0
        h = int(utc_raw[0:2])
        m = int(utc_raw[2:4])
        s = float(utc_raw[4:])
        utc_secs = h * 3600.0 + m * 60.0 + s

        lat_raw = parts[2]
        lat_deg = int(lat_raw[0:2])
        lat_min = float(lat_raw[2:])
        latitude = lat_deg + lat_min / 60.0
        if parts[3] == 'S':
            latitude = -latitude

        lon_raw = parts[4]
        lon_deg = int(lon_raw[0:3])
        lon_min = float(lon_raw[3:])
        longitude = lon_deg + lon_min / 60.0
        if parts[5] == 'W':
            longitude = -longitude

        hdop = float(parts[8]) if parts[8] else 0.0
        altitude = float(parts[9]) if parts[9] else 0.0

        utm_e, utm_n, zone_num, zone_let = utm.from_latlon(latitude, longitude)

        return {
            'utc': utc,
            'utc_secs': utc_secs,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'hdop': hdop,
            'fix_quality': fix_quality,
            'utm_easting': utm_e,
            'utm_northing': utm_n,
            'zone': zone_num,
            'letter': zone_let,
        }
    except (ValueError, IndexError):
        return None


class RTKDriverNode(Node):
    """ROS2 node that reads from a serial RTK GNSS and publishes Customrtk."""

    def __init__(self, port):
        super().__init__('rtk_driver')
        self.publisher_ = self.create_publisher(Customrtk, 'gps', 10)

        try:
            self.serial_port = serial.Serial(port, baudrate=4800, timeout=2.0)
            self.get_logger().info(f'Opened serial port: {port} at 4800 baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise SystemExit(1)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not line:
            return
        if 'GGA' not in line:
            return

        data = parse_gngga(line)
        if data is None:
            return

        msg = Customrtk()
        secs = int(data['utc_secs'])
        nsecs = int((data['utc_secs'] - secs) * 1e9)
        msg.header.stamp.sec = secs
        msg.header.stamp.nanosec = nsecs
        msg.header.frame_id = 'GPS1_Frame'

        msg.latitude = data['latitude']
        msg.longitude = data['longitude']
        msg.altitude = data['altitude']
        msg.utm_easting = data['utm_easting']
        msg.utm_northing = data['utm_northing']
        msg.utc = data['utc']
        msg.zone = data['zone']
        msg.letter = data['letter']
        msg.fix_quality = data['fix_quality']
        msg.hdop = data['hdop']

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Fix={msg.fix_quality}, HDOP={msg.hdop:.1f}, '
            f'Lat={msg.latitude:.7f}, Lon={msg.longitude:.7f}, '
            f'E={msg.utm_easting:.3f}, N={msg.utm_northing:.3f}, '
            f'Alt={msg.altitude:.1f}'
        )

    def destroy_node(self):
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='RTK GPS Driver for ROS2')
    parser.add_argument('port', type=str, nargs='?', default='/dev/ttyUSB0',
                        help='Serial port path')
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = RTKDriverNode(parsed_args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()