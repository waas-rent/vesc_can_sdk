#!/usr/bin/env python3
"""
VESC CAN SDK - Python Monitor

This script demonstrates how to use the VESC CAN SDK from Python to monitor
VESC motor controllers over CAN.

Copyright (c) 2024 VESC CAN SDK Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import sys
import time
import signal
import argparse
import threading
from typing import Dict, Any, Optional
import ctypes
from ctypes import cdll, c_bool, c_uint8, c_uint32, c_float, c_int32, c_char, c_int16, c_uint16, c_uint64

# Load the VESC CAN SDK library
try:
    vesc_lib = cdll.LoadLibrary("./libvesc_can_sdk.so")
except OSError:
    print("Error: Could not load libvesc_can_sdk.so")
    print("Make sure to build the SDK first with 'make'")
    sys.exit(1)

# Define C types for structures
class VescValues(ctypes.Structure):
    _fields_ = [
        ("temp_fet", c_float),
        ("temp_motor", c_float),
        ("current_motor", c_float),
        ("current_in", c_float),
        ("current_id", c_float),
        ("current_iq", c_float),
        ("duty_cycle", c_float),
        ("rpm", c_float),
        ("v_in", c_float),
        ("amp_hours", c_float),
        ("amp_hours_charged", c_float),
        ("watt_hours", c_float),
        ("watt_hours_charged", c_float),
        ("tachometer", c_int32),
        ("tachometer_abs", c_int32),
        ("fault_code", c_uint8),
        ("pid_pos", c_float),
        ("controller_id", c_uint8),
        ("temp_mos1", c_float),
        ("temp_mos2", c_float),
        ("temp_mos3", c_float),
        ("vd", c_float),
        ("vq", c_float),
        ("status", c_uint8)
    ]

class VescMotorRLResponse(ctypes.Structure):
    _fields_ = [
        ("resistance", c_float),
        ("inductance", c_float),
        ("ld_lq_diff", c_float),
        ("valid", c_bool)
    ]

class VescFwVersion(ctypes.Structure):
    _fields_ = [
        ("major", c_uint8),
        ("minor", c_uint8),
        ("hw_name", c_char * 32),
        ("uuid", c_uint8 * 12),
        ("pairing_done", c_bool),
        ("test_version", c_uint8),
        ("hw_type", c_uint8),
        ("cfg_num", c_uint8),
        ("valid", c_bool)
    ]

class VescAdcValues(ctypes.Structure):
    _fields_ = [
        ("adc1", c_float),
        ("adc2", c_float),
        ("adc3", c_float),
        ("v_in", c_float),
        ("valid", c_bool)
    ]

class VescPpmValues(ctypes.Structure):
    _fields_ = [
        ("ppm", c_float),
        ("pulse_len", c_float),
        ("valid", c_bool)
    ]

# Define function signatures
vesc_lib.vesc_can_init.argtypes = [ctypes.c_void_p]
vesc_lib.vesc_can_init.restype = c_bool

vesc_lib.vesc_set_response_callback.argtypes = [ctypes.c_void_p]

vesc_lib.vesc_process_can_frame.argtypes = [c_uint32, ctypes.POINTER(c_uint8), c_uint8]

vesc_lib.vesc_get_values.argtypes = [c_uint8]
vesc_lib.vesc_get_decoded_adc.argtypes = [c_uint8]
vesc_lib.vesc_get_decoded_ppm.argtypes = [c_uint8]
vesc_lib.vesc_get_fw_version.argtypes = [c_uint8]
vesc_lib.vesc_detect_motor_r_l.argtypes = [c_uint8]

vesc_lib.vesc_parse_get_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescValues)]
vesc_lib.vesc_parse_get_values.restype = c_bool

vesc_lib.vesc_parse_motor_rl_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescMotorRLResponse)]
vesc_lib.vesc_parse_motor_rl_response.restype = c_bool

vesc_lib.vesc_parse_fw_version.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescFwVersion)]
vesc_lib.vesc_parse_fw_version.restype = c_bool

vesc_lib.vesc_parse_adc_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescAdcValues)]
vesc_lib.vesc_parse_adc_values.restype = c_bool

vesc_lib.vesc_parse_ppm_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescPpmValues)]
vesc_lib.vesc_parse_ppm_values.restype = c_bool

class VescMonitor:
    def __init__(self, can_interface: str, vesc_id: int = 1):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.running = True
        self.latest_values = {}
        self.latest_fw_version = None
        self.latest_adc_values = None
        self.latest_ppm_values = None
        self.latest_motor_rl = None
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Initialize CAN interface
        self._init_can()
        
        # Initialize VESC SDK
        self._init_vesc_sdk()
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def _signal_handler(self, signum, frame):
        print(f"\nReceived signal {signum}, shutting down...")
        self.running = False
    
    def _init_can(self):
        """Initialize CAN interface using SocketCAN"""
        import socket
        import struct
        
        # Create CAN socket
        self.can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        
        # Get interface index
        self.can_socket.bind((self.can_interface,))
        
        # Set non-blocking mode
        self.can_socket.setblocking(False)
        
        print(f"CAN interface {self.can_interface} initialized")
    
    def _init_vesc_sdk(self):
        """Initialize VESC CAN SDK"""
        # Define CAN send function
        def can_send(id: int, data: bytes, length: int) -> bool:
            try:
                # Create CAN frame
                can_id = id | 0x80000000  # Extended frame
                frame = struct.pack("<I", can_id) + struct.pack("B", length) + data + b'\x00' * (8 - length)
                self.can_socket.send(frame)
                return True
            except Exception as e:
                print(f"CAN send error: {e}")
                return False
        
        # Convert Python function to C callback
        self.can_send_func = ctypes.CFUNCTYPE(c_bool, c_uint32, ctypes.POINTER(c_uint8), c_uint8)(can_send)
        
        # Initialize SDK
        if not vesc_lib.vesc_can_init(self.can_send_func):
            raise RuntimeError("Failed to initialize VESC CAN SDK")
        
        # Set response callback
        def response_callback(controller_id: int, command: int, data: bytes, length: int):
            self._handle_response(controller_id, command, data, length)
        
        self.response_callback = ctypes.CFUNCTYPE(None, c_uint8, c_uint8, ctypes.POINTER(c_uint8), c_uint8)(response_callback)
        vesc_lib.vesc_set_response_callback(self.response_callback)
        
        print("VESC CAN SDK initialized")
    
    def _handle_response(self, controller_id: int, command: int, data: bytes, length: int):
        """Handle VESC response"""
        data_array = (c_uint8 * length)(*data)
        
        if command == 27:  # COMM_GET_VALUES
            values = VescValues()
            if vesc_lib.vesc_parse_get_values(data_array, length, ctypes.byref(values)):
                self.latest_values = {
                    'temp_fet': values.temp_fet,
                    'temp_motor': values.temp_motor,
                    'current_motor': values.current_motor,
                    'current_in': values.current_in,
                    'duty_cycle': values.duty_cycle,
                    'rpm': values.rpm,
                    'v_in': values.v_in,
                    'amp_hours': values.amp_hours,
                    'amp_hours_charged': values.amp_hours_charged,
                    'watt_hours': values.watt_hours,
                    'watt_hours_charged': values.watt_hours_charged,
                    'tachometer': values.tachometer,
                    'tachometer_abs': values.tachometer_abs,
                    'fault_code': values.fault_code,
                    'pid_pos': values.pid_pos,
                    'controller_id': values.controller_id,
                    'temp_mos1': values.temp_mos1,
                    'temp_mos2': values.temp_mos2,
                    'temp_mos3': values.temp_mos3,
                    'vd': values.vd,
                    'vq': values.vq,
                    'status': values.status
                }
        
        elif command == 0:  # COMM_FW_VERSION
            version = VescFwVersion()
            if vesc_lib.vesc_parse_fw_version(data_array, length, ctypes.byref(version)):
                self.latest_fw_version = {
                    'major': version.major,
                    'minor': version.minor,
                    'hw_name': version.hw_name.decode('utf-8').rstrip('\x00'),
                    'hw_type': version.hw_type,
                    'cfg_num': version.cfg_num
                }
        
        elif command == 30:  # COMM_GET_DECODED_ADC
            adc = VescAdcValues()
            if vesc_lib.vesc_parse_adc_values(data_array, length, ctypes.byref(adc)):
                self.latest_adc_values = {
                    'adc1': adc.adc1,
                    'adc2': adc.adc2,
                    'adc3': adc.adc3,
                    'v_in': adc.v_in
                }
        
        elif command == 31:  # COMM_GET_DECODED_PPM
            ppm = VescPpmValues()
            if vesc_lib.vesc_parse_ppm_values(data_array, length, ctypes.byref(ppm)):
                self.latest_ppm_values = {
                    'ppm': ppm.ppm,
                    'pulse_len': ppm.pulse_len
                }
        
        elif command == 25:  # COMM_DETECT_MOTOR_R_L
            motor_rl = VescMotorRLResponse()
            if vesc_lib.vesc_parse_motor_rl_response(data_array, length, ctypes.byref(motor_rl)):
                self.latest_motor_rl = {
                    'resistance': motor_rl.resistance,
                    'inductance': motor_rl.inductance,
                    'ld_lq_diff': motor_rl.ld_lq_diff
                }
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        import socket
        
        while self.running:
            try:
                # Read CAN frames
                frame = self.can_socket.recv(16)
                if len(frame) == 16:
                    can_id = struct.unpack("<I", frame[:4])[0]
                    length = frame[4]
                    data = frame[5:5+length]
                    
                    # Process frame
                    data_array = (c_uint8 * length)(*data)
                    vesc_lib.vesc_process_can_frame(can_id, data_array, length)
            
            except socket.error:
                # No data available (non-blocking)
                pass
            
            time.sleep(0.001)  # 1ms delay
    
    def get_firmware_version(self):
        """Get firmware version"""
        vesc_lib.vesc_get_fw_version(self.vesc_id)
        time.sleep(0.1)
        return self.latest_fw_version
    
    def get_motor_values(self):
        """Get motor values"""
        vesc_lib.vesc_get_values(self.vesc_id)
        time.sleep(0.1)
        return self.latest_values
    
    def get_adc_values(self):
        """Get ADC values"""
        vesc_lib.vesc_get_decoded_adc(self.vesc_id)
        time.sleep(0.1)
        return self.latest_adc_values
    
    def get_ppm_values(self):
        """Get PPM values"""
        vesc_lib.vesc_get_decoded_ppm(self.vesc_id)
        time.sleep(0.1)
        return self.latest_ppm_values
    
    def detect_motor_rl(self):
        """Detect motor resistance and inductance"""
        vesc_lib.vesc_detect_motor_r_l(self.vesc_id)
        time.sleep(2.0)  # Detection takes time
        return self.latest_motor_rl
    
    def print_status(self):
        """Print current status"""
        print("\n" + "="*60)
        print(f"VESC Monitor - Interface: {self.can_interface}, ID: {self.vesc_id}")
        print("="*60)
        
        # Firmware version
        if self.latest_fw_version:
            print(f"Firmware: {self.latest_fw_version['major']}.{self.latest_fw_version['minor']}")
            print(f"Hardware: {self.latest_fw_version['hw_name']}")
        
        # Motor values
        if self.latest_values:
            print(f"\nMotor Status:")
            print(f"  Temperature FET: {self.latest_values['temp_fet']:.1f}°C")
            print(f"  Temperature Motor: {self.latest_values['temp_motor']:.1f}°C")
            print(f"  Current Motor: {self.latest_values['current_motor']:.2f}A")
            print(f"  Current Input: {self.latest_values['current_in']:.2f}A")
            print(f"  Duty Cycle: {self.latest_values['duty_cycle']*100:.1f}%")
            print(f"  RPM: {self.latest_values['rpm']:.0f}")
            print(f"  Input Voltage: {self.latest_values['v_in']:.1f}V")
            print(f"  Consumed Ah: {self.latest_values['amp_hours']:.2f}Ah")
            print(f"  Consumed Wh: {self.latest_values['watt_hours']:.2f}Wh")
            print(f"  Fault Code: {self.latest_values['fault_code']}")
        
        # ADC values
        if self.latest_adc_values:
            print(f"\nADC Values:")
            print(f"  ADC1: {self.latest_adc_values['adc1']:.3f}")
            print(f"  ADC2: {self.latest_adc_values['adc2']:.3f}")
            print(f"  ADC3: {self.latest_adc_values['adc3']:.3f}")
            print(f"  Input Voltage: {self.latest_adc_values['v_in']:.1f}V")
        
        # PPM values
        if self.latest_ppm_values:
            print(f"\nPPM Values:")
            print(f"  PPM: {self.latest_ppm_values['ppm']:.3f}")
            print(f"  Pulse Length: {self.latest_ppm_values['pulse_len']:.1f}μs")
        
        # Motor R/L
        if self.latest_motor_rl:
            print(f"\nMotor R/L Detection:")
            print(f"  Resistance: {self.latest_motor_rl['resistance']:.6f}Ω")
            print(f"  Inductance: {self.latest_motor_rl['inductance']:.8f}H")
            print(f"  Ld-Lq Difference: {self.latest_motor_rl['ld_lq_diff']:.8f}H")
        
        print("="*60)
    
    def run(self):
        """Run the monitor"""
        print(f"Starting VESC monitor on {self.can_interface} for VESC ID {self.vesc_id}")
        print("Press Ctrl+C to stop")
        
        # Get initial data
        self.get_firmware_version()
        time.sleep(0.5)
        
        while self.running:
            # Get motor values
            self.get_motor_values()
            time.sleep(0.1)
            
            # Get ADC values
            self.get_adc_values()
            time.sleep(0.1)
            
            # Get PPM values
            self.get_ppm_values()
            time.sleep(0.1)
            
            # Print status
            self.print_status()
            
            # Wait before next update
            time.sleep(2.0)

def main():
    parser = argparse.ArgumentParser(description='VESC CAN Monitor')
    parser.add_argument('interface', help='CAN interface (e.g., can0)')
    parser.add_argument('--id', type=int, default=1, help='VESC controller ID (default: 1)')
    parser.add_argument('--detect-motor', action='store_true', help='Detect motor R/L parameters')
    
    args = parser.parse_args()
    
    try:
        monitor = VescMonitor(args.interface, args.id)
        
        if args.detect_motor:
            print("Detecting motor R/L parameters...")
            result = monitor.detect_motor_rl()
            if result:
                print(f"Motor R/L Detection Results:")
                print(f"  Resistance: {result['resistance']:.6f}Ω")
                print(f"  Inductance: {result['inductance']:.8f}H")
                print(f"  Ld-Lq Difference: {result['ld_lq_diff']:.8f}H")
            else:
                print("Motor R/L detection failed or timed out")
        else:
            monitor.run()
    
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 