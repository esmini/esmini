import os
import re
import socket
import struct
import sys
import threading
import time
import unittest
import argparse

from test_common import ESMINI_PATH, set_timeout, run_scenario

SCENARIO_PATH = os.path.join(ESMINI_PATH, 'resources/xosc/cut-in.xosc')
COMMON_ARGS = '--headless --fixed_timestep 0.05 --quit_at_end '


def is_osi_supported():
    return os.path.isfile(os.path.join(ESMINI_PATH, 'bin', 'osireceiver')) or \
           os.path.isfile(os.path.join(ESMINI_PATH, 'bin', 'osireceiver.exe'))


class UdpPacketCollector:
    def __init__(self, ip='127.0.0.1', port=48198, timeout=5.0, max_packets=10):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.max_packets = max_packets
        self.packets = []
        self.sock = None
        self.thread = None
        self.started_event = threading.Event()
        self.stop_event = threading.Event()

    def _listen(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        self.sock.settimeout(0.2)
        self.started_event.set()

        start_time = time.time()
        while not self.stop_event.is_set() and (time.time() - start_time < self.timeout):
            if len(self.packets) >= self.max_packets:
                break
            try:
                data, _ = self.sock.recvfrom(65536)
                self.packets.append(data)
            except socket.timeout:
                continue
            except Exception:
                break

    def start(self):
        self.thread = threading.Thread(target=self._listen)
        self.thread.daemon = True
        self.thread.start()
        self.started_event.wait(timeout=2.0)

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


class TestSuite(unittest.TestCase):

    def test_osi_udp_custom_port(self):
        """Test sending OSI packets over a custom UDP port (127.0.0.1:5000)."""
        if not is_osi_supported():
            print('skipping test_osi_udp_custom_port for non-OSI builds ', end='', file=sys.stderr)
            return

        port = 5000
        collector = UdpPacketCollector(ip='127.0.0.1', port=port, max_packets=5)
        collector.start()

        try:
            log, _, _, _ = run_scenario(
                SCENARIO_PATH,
                COMMON_ARGS + f'--osi_receiver_ip 127.0.0.1 --osi_receiver_port {port}'
            )
        finally:
            collector.stop()

        self.assertGreater(len(collector.packets), 0, f"No packets received on custom port {port}")
        counter, datasize = struct.unpack('iI', collector.packets[0][:8])
        self.assertNotEqual(counter, 0, "Packet counter should not be 0")
        self.assertGreater(datasize, 0, "Packet datasize should be positive")

    def test_osi_udp_custom_data_size(self):
        """Test sending OSI packets with custom max UDP data size (1500 bytes)."""
        if not is_osi_supported():
            print('skipping test_osi_udp_custom_data_size for non-OSI builds ', end='', file=sys.stderr)
            return

        port = 5000
        max_size = 1500
        collector = UdpPacketCollector(ip='127.0.0.1', port=port, max_packets=10)
        collector.start()

        try:
            log, _, _, _ = run_scenario(
                SCENARIO_PATH,
                COMMON_ARGS + f'--osi_receiver_ip 127.0.0.1 --osi_receiver_port {port} --osi_receiver_udp_data_size {max_size}'
            )
        finally:
            collector.stop()

        self.assertGreater(len(collector.packets), 0, f"No packets received on port {port}")
        for idx, packet in enumerate(collector.packets):
            self.assertGreaterEqual(len(packet), 8, f"Packet {idx} is smaller than 8-byte header")
            counter, datasize = struct.unpack('iI', packet[:8])
            self.assertLessEqual(
                datasize,
                max_size,
                f"Packet {idx} datasize ({datasize}) exceeds configured max data size ({max_size})"
            )

    def test_osi_udp_default_values(self):
        """Test default UDP port (48198) and default max data size (8192 bytes)."""
        if not is_osi_supported():
            print('skipping test_osi_udp_default_values for non-OSI builds ', end='', file=sys.stderr)
            return

        default_port = 48198
        default_max_data_size = 8192
        collector = UdpPacketCollector(ip='127.0.0.1', port=default_port, max_packets=5)
        collector.start()

        try:
            log, _, _, _ = run_scenario(
                SCENARIO_PATH,
                COMMON_ARGS + '--osi_receiver_ip 127.0.0.1'
            )
        finally:
            collector.stop()

        self.assertGreater(len(collector.packets), 0, f"No packets received on default port {default_port}")
        for idx, packet in enumerate(collector.packets):
            self.assertGreaterEqual(len(packet), 8, f"Packet {idx} is smaller than 8-byte header")
            counter, datasize = struct.unpack('iI', packet[:8])
            self.assertLessEqual(
                datasize,
                default_max_data_size,
                f"Packet {idx} datasize ({datasize}) exceeds default max data size ({default_max_data_size})"
            )

    def test_osi_udp_invalid_port_warning(self):
        """Test warning messages when port is out of allowed range [1024, 65536]."""
        if not is_osi_supported():
            print('skipping test_osi_udp_invalid_port_warning for non-OSI builds ', end='', file=sys.stderr)
            return

        # Low port (< 1024)
        log_low, _, _, err_low = run_scenario(
            SCENARIO_PATH,
            COMMON_ARGS + '--osi_receiver_ip 127.0.0.1 --osi_receiver_port 500'
        )
        combined_low = (log_low or '') + (err_low or '')
        self.assertIsNotNone(
            re.search(r'Requested OSI UDP port 500 is outside allowed range \[1024, 65536\], using default port 48198', combined_low),
            f"Expected port warning not found in log for port 500. Output:\n{combined_low}"
        )

        # High port (> 65536)
        log_high, _, _, err_high = run_scenario(
            SCENARIO_PATH,
            COMMON_ARGS + '--osi_receiver_ip 127.0.0.1 --osi_receiver_port 70000'
        )
        combined_high = (log_high or '') + (err_high or '')
        self.assertIsNotNone(
            re.search(r'Requested OSI UDP port 70000 is outside allowed range \[1024, 65536\], using default port 48198', combined_high),
            f"Expected port warning not found in log for port 70000. Output:\n{combined_high}"
        )

    def test_osi_udp_invalid_data_size_warning(self):
        """Test warning messages when data size is out of allowed range [1464, 65499]."""
        if not is_osi_supported():
            print('skipping test_osi_udp_invalid_data_size_warning for non-OSI builds ', end='', file=sys.stderr)
            return

        # Size below minimum (< 1464)
        log_low, _, _, err_low = run_scenario(
            SCENARIO_PATH,
            COMMON_ARGS + '--osi_receiver_ip 127.0.0.1 --osi_receiver_udp_data_size 500'
        )
        combined_low = (log_low or '') + (err_low or '')
        self.assertIsNotNone(
            re.search(r'Requested OSI UDP data size 500 is below minimum allowed size 1464, using minimum instead', combined_low),
            f"Expected data size warning not found in log for size 500. Output:\n{combined_low}"
        )

        # Size above maximum (> 65499)
        log_high, _, _, err_high = run_scenario(
            SCENARIO_PATH,
            COMMON_ARGS + '--osi_receiver_ip 127.0.0.1 --osi_receiver_udp_data_size 100000'
        )
        combined_high = (log_high or '') + (err_high or '')
        self.assertIsNotNone(
            re.search(r'Requested OSI UDP data size 100000 exceeds maximum allowed size 65499, using maximum instead', combined_high),
            f"Expected data size warning not found in log for size 100000. Output:\n{combined_high}"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--timeout", type=int, default=40, help="timeout per testcase")
    parser.add_argument("testcase", nargs="?", help="run only this testcase")
    args = parser.parse_args()

    print("timeout:", args.timeout, file=sys.stderr)
    set_timeout(args.timeout)

    if args.testcase:
        unittest.main(argv=['ignored', '-v', 'TestSuite.' + args.testcase])
    else:
        unittest.main(argv=[''], verbosity=2)

