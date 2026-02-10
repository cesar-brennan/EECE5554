#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
EECE5554 Lab 1 - GPS Driver Node (ROS2 Humble)
Reads GPGGA sentences from a serial GPS device (BU-353S4),
parses them, converts lat/lon to UTM, and publishes a custom GPS message.
"""

import rclpy
from rclpy.node import Node
import serial
import utm
import sys
import argparse

from gps_msg.msg import GpsMsg


def parse_gpgga(line):
    """
    Parse a GPGGA NMEA sentence and return a dictionary of values.

    Example GPGGA sentence:
    $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47

    Fields:
      0: $GPGGA           6: Fix quality      12: Geoid units
      1: UTC time          7: Num satellites   13: Age of DGPS
      2: Latitude          8: HDOP             14: Checksum
      3: N/S               9: Altitude
      4: Longitude        10: Alt units
      5: E/W              11: Geoid separation
    """
    parts = line.strip().split(',')

    if len(parts) < 15:
        return None
    if 'GPGGA' not in parts[0] and 'GNGGA' not in parts[0]:
        return None

    # Check for a valid fix
    try:
        fix_quality = int(parts[6])
    except (ValueError, IndexError):
        return None
    if fix_quality == 0:
        return None

    try:
        # ---- UTC Time ----
        utc_raw = parts[1]
        utc = float(utc_raw) if utc_raw else 0.0
        hours = int(utc_raw[0:2])
        minutes = int(utc_raw[2:4])
        seconds = float(utc_raw[4:])
        utc_in_secs = hours * 3600.0 + minutes * 60.0 + seconds

        # ---- Latitude (convert ddmm.mmmm to decimal degrees) ----
        lat_raw = parts[2]
        lat_dir = parts[3]
        lat_deg = int(lat_raw[0:2])
        lat_min = float(lat_raw[2:])
        latitude = lat_deg + lat_min / 60.0
        if lat_dir == 'S':
            latitude = -latitude

        # ---- Longitude (convert dddmm.mmmm to decimal degrees) ----
        lon_raw = parts[4]
        lon_dir = parts[5]
        lon_deg = int(lon_raw[0:3])
        lon_min = float(lon_raw[3:])
        longitude = lon_deg + lon_min / 60.0
        if lon_dir == 'W':
            longitude = -longitude

        # ---- HDOP ----
        hdop = float(parts[8]) if parts[8] else 0.0

        # ---- Altitude ----
        altitude = float(parts[9]) if parts[9] else 0.0

        # ---- Convert to UTM ----
        utm_result = utm.from_latlon(latitude, longitude)
        utm_easting = utm_result[0]
        utm_northing = utm_result[1]
        zone_number = utm_result[2]   # int
        zone_letter = utm_result[3]   # string

        return {
            'utc': utc,
            'utc_secs': utc_in_secs,
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'hdop': hdop,
            'utm_easting': utm_easting,
            'utm_northing': utm_northing,
            'zone': zone_number,
            'letter': zone_letter,
        }

    except (ValueError, IndexError):
        return None


class GPSDriverNode(Node):
    """ROS2 node that reads from a serial GPS and publishes GpsMsg."""

    def __init__(self, port):
        super().__init__('gps_driver')

        # Create publisher on topic "gps"
        self.publisher_ = self.create_publisher(GpsMsg, 'gps', 10)

        # Open serial port (BU-353S4 default baud rate is 4800)
        try:
            self.serial_port = serial.Serial(port, baudrate=4800, timeout=2.0)
            self.get_logger().info(f'Opened serial port: {port} at 4800 baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not open serial port {port}: {e}')
            raise SystemExit(1)

        # Read from serial at ~10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Read a line from serial, parse it, and publish if valid GPGGA."""
        try:
            line = self.serial_port.readline().decode('ascii', errors='replace').strip()
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return

        if not line:
            return

        # Only process GPGGA or GNGGA sentences
        if 'GPGGA' not in line and 'GNGGA' not in line:
            return

        data = parse_gpgga(line)
        if data is None:
            self.get_logger().warn(f'Could not parse GPGGA: {line}')
            return

        # Build the message
        msg = GpsMsg()

        # Header - use GPS UTC time, NOT system time
        secs = int(data['utc_secs'])
        nsecs = int((data['utc_secs'] - secs) * 1e9)
        msg.header.stamp.sec = secs
        msg.header.stamp.nanosec = nsecs
        msg.header.frame_id = 'GPS1_Frame'

        msg.latitude = data['latitude']
        msg.longitude = data['longitude']
        msg.altitude = data['altitude']
        msg.hdop = data['hdop']
        msg.utm_easting = data['utm_easting']
        msg.utm_northing = data['utm_northing']
        msg.utc = data['utc']
        msg.zone = data['zone']
        msg.letter = data['letter']

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}, '
            f'E={msg.utm_easting:.2f}, N={msg.utm_northing:.2f}, '
            f'Alt={msg.altitude:.1f}, HDOP={msg.hdop}'
        )

    def destroy_node(self):
        """Clean up serial port on shutdown."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed.')
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='GPS Driver for ROS2')
    parser.add_argument('port', type=str, nargs='?', default='/dev/ttyUSB0',
                        help='Serial port path (e.g., /dev/ttyUSB0 or /dev/pts/6)')

    # ROS2 may pass extra args, so parse only known args
    parsed_args, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = GPSDriverNode(parsed_args.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()