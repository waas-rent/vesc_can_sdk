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
import struct
from datetime import datetime
import os

# Try to import python-can
try:
    import can
except ImportError:
    print("Error: python-can library not found")
    print("Install it with: pip install python-can")
    # Don't exit for help command
    if len(sys.argv) > 1 and sys.argv[1] in ['--help', '-h']:
        pass
    else:
        sys.exit(1)

# Load the VESC CAN SDK library
try:
    vesc_lib = cdll.LoadLibrary("../lib/libvesc_can_sdk.so")
except OSError:
    print("Error: Could not load libvesc_can_sdk.so")
    print("Make sure to build the SDK first with 'make'")
    # Don't exit for help command
    if len(sys.argv) > 1 and sys.argv[1] in ['--help', '-h']:
        vesc_lib = None
    else:
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

# Define missing structures
class VescMotorParamResponse(ctypes.Structure):
    _fields_ = [
        ("current", c_float),
        ("min_rpm", c_float),
        ("low_duty", c_float),
        ("valid", c_bool)
    ]

class VescFluxLinkageResponse(ctypes.Structure):
    _fields_ = [
        ("current", c_float),
        ("min_rpm", c_float),
        ("duty", c_float),
        ("resistance", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg1(ctypes.Structure):
    _fields_ = [
        ("temp_fet", c_float),
        ("current_motor", c_float),
        ("rpm", c_float),
        ("duty_cycle", c_float),
        ("v_in", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg2(ctypes.Structure):
    _fields_ = [
        ("amp_hours", c_float),
        ("amp_hours_charged", c_float),
        ("watt_hours", c_float),
        ("watt_hours_charged", c_float),
        ("tachometer", c_int32),
        ("tachometer_abs", c_int32),
        ("valid", c_bool)
    ]

class VescStatusMsg3(ctypes.Structure):
    _fields_ = [
        ("fault_code", c_uint8),
        ("pid_pos", c_float),
        ("controller_id", c_uint8),
        ("valid", c_bool)
    ]

class VescStatusMsg4(ctypes.Structure):
    _fields_ = [
        ("temp_mos1", c_float),
        ("temp_mos2", c_float),
        ("temp_mos3", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg5(ctypes.Structure):
    _fields_ = [
        ("vd", c_float),
        ("vq", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg6(ctypes.Structure):
    _fields_ = [
        ("status", c_uint8),
        ("valid", c_bool)
    ]

class VescDebugConfig(ctypes.Structure):
    _fields_ = [
        ("level", c_uint8),
        ("categories", c_uint16),
        ("output_func", ctypes.c_void_p),
        ("enable_timestamps", c_bool),
        ("enable_statistics", c_bool)
    ]

class VescDebugStats(ctypes.Structure):
    _fields_ = [
        ("rx_packets", c_uint32),
        ("tx_packets", c_uint32),
        ("rx_errors", c_uint32),
        ("tx_errors", c_uint32),
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

# Add missing parsing functions
vesc_lib.vesc_parse_motor_param_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescMotorParamResponse)]
vesc_lib.vesc_parse_motor_param_response.restype = c_bool

vesc_lib.vesc_parse_flux_linkage_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescFluxLinkageResponse)]
vesc_lib.vesc_parse_flux_linkage_response.restype = c_bool

vesc_lib.vesc_parse_status_msg_1.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg1)]
vesc_lib.vesc_parse_status_msg_1.restype = c_bool

vesc_lib.vesc_parse_status_msg_2.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg2)]
vesc_lib.vesc_parse_status_msg_2.restype = c_bool

vesc_lib.vesc_parse_status_msg_3.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg3)]
vesc_lib.vesc_parse_status_msg_3.restype = c_bool

vesc_lib.vesc_parse_status_msg_4.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg4)]
vesc_lib.vesc_parse_status_msg_4.restype = c_bool

vesc_lib.vesc_parse_status_msg_5.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg5)]
vesc_lib.vesc_parse_status_msg_5.restype = c_bool

vesc_lib.vesc_parse_status_msg_6.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescStatusMsg6)]
vesc_lib.vesc_parse_status_msg_6.restype = c_bool

# Add debug configuration functions
vesc_lib.vesc_debug_configure.argtypes = [ctypes.POINTER(VescDebugConfig)]
vesc_lib.vesc_debug_configure.restype = c_bool

vesc_lib.vesc_debug_enable.argtypes = [c_uint8, c_uint16]
vesc_lib.vesc_debug_enable.restype = c_bool

vesc_lib.vesc_debug_get_stats.argtypes = [ctypes.POINTER(VescDebugStats)]
vesc_lib.vesc_debug_get_stats.restype = c_bool

class VescMonitor:
    def __init__(self, can_interface: str, vesc_id: int = 1, enable_logging: bool = False, bustype: str = 'socketcan', baudrate: int = 500000, enable_debug: bool = False):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.bustype = bustype
        self.baudrate = baudrate
        self.enable_debug = enable_debug
        self.running = True
        self.latest_values = {}
        self.latest_fw_version = None
        self.latest_adc_values = None
        self.latest_ppm_values = None
        self.latest_motor_rl = None
        self.enable_logging = enable_logging
        self.log_file = None
        self.session_start_time = datetime.now()
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Initialize logging if enabled
        if self.enable_logging:
            self._init_logging()
        
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
        if self.log_file:
            self.log_file.close()
        if hasattr(self, 'can_obj'):
            self.can_obj.shutdown()
    
    def _init_logging(self):
        """Initialize CAN message logging"""
        # Create log filename with timestamp
        timestamp = self.session_start_time.strftime("%Y%m%d_%H%M%S")
        log_filename = f"vesc_can_log_{timestamp}.csv"
        
        # Open log file
        self.log_file = open(log_filename, 'w')
        
        # Write header
        self.log_file.write("# Logger type: VESC CAN SDK Python Monitor\n")
        self.log_file.write("# HW rev: VESC CAN SDK\n")
        self.log_file.write("# FW rev: Python Monitor\n")
        self.log_file.write(f"# Logger ID: VESC_Monitor_{self.vesc_id}\n")
        self.log_file.write("# Session No.: 1\n")
        self.log_file.write("# Split No.: 1\n")
        self.log_file.write(f"# Time: {self.session_start_time.strftime('%Y%m%dT%H%M%S')}\n")
        self.log_file.write('# Value separator: ";"\n')
        self.log_file.write("# Time format: 6\n")
        self.log_file.write('# Time separator: "."\n')
        self.log_file.write('# Time separator ms: ","\n')
        self.log_file.write('# Date separator: "/"\n')
        self.log_file.write('# Time and date separator: " "\n')
        self.log_file.write(f"# Bit-rate: {self.baudrate}\n")
        self.log_file.write("# Silent mode: false\n")
        self.log_file.write("# Cyclic mode: false\n")
        self.log_file.write("Timestamp;Lost;Type;ID;Length;Data\n")
        
        print(f"CAN logging enabled: {log_filename}")
    
    def _init_can(self):
        """Initialize CAN interface using python-can"""
        # Create CAN object with baudrate
        if self.bustype == 'canalystii':
            # CANalyst-II requires specific constructor parameters
            from can.interfaces.canalystii import CANalystIIBus
            self.can_obj = CANalystIIBus(channel=0, bitrate=self.baudrate)
        else:
            # Standard python-can interface
            self.can_obj = can.Bus(interface=self.bustype, channel=self.can_interface, bitrate=self.baudrate)
        
        print(f"CAN interface {self.can_interface} (type: {self.bustype}, baudrate: {self.baudrate}) initialized")
    
    def _init_vesc_sdk(self):
        """Initialize VESC CAN SDK"""
        # Define CAN send function
        def can_send(id: int, data_ptr, length: int) -> bool:
            try:
                # Convert ctypes pointer to bytes
                data_bytes = bytes(data_ptr[:length])
                
                # Create CAN message with extended ID
                can_id = id | 0x80000000  # Extended frame
                # Pad data to 8 bytes if needed
                padded_data = data_bytes + b'\x00' * (8 - length)
                can_msg = can.Message(arbitration_id=can_id, data=padded_data, is_extended_id=True)
                self.can_obj.send(can_msg)
                
                # Log transmitted message
                self._log_can_message(id, data_bytes, length, is_tx=True)
                
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
        
        # Initialize debugging if enabled
        if self.enable_debug:
            self._init_debug()
        
        print("VESC CAN SDK initialized")
    
    def _init_debug(self):
        """Initialize VESC CAN SDK debugging"""
        try:
            # Define debug output function
            def debug_output(level: int, category: int, message_ptr):
                try:
                    # Convert C string pointer to Python string
                    if message_ptr:
                        message = message_ptr.decode('utf-8', errors='replace')
                    else:
                        message = "<null message>"
                    
                    level_names = {0: "ERROR", 1: "WARN", 2: "INFO", 3: "VERBOSE"}
                    category_names = {0: "GENERAL", 1: "CAN", 2: "PARSER", 3: "COMM"}
                    
                    level_name = level_names.get(level, f"LEVEL_{level}")
                    category_name = category_names.get(category, f"CAT_{category}")
                    
                    print(f"[VESC_DEBUG] {level_name}/{category_name}: {message}")
                except Exception as e:
                    print(f"[VESC_DEBUG] Error in debug callback: {e}")
            
            # Convert Python function to C callback
            self.debug_output_func = ctypes.CFUNCTYPE(None, c_uint8, c_uint8, ctypes.c_char_p)(debug_output)
        except Exception as e:
            print(f"Warning: Failed to create debug callback function: {e}")
            return
        
        # Configure debug settings
        try:
            debug_config = VescDebugConfig()
            debug_config.level = c_uint8(3)  # DEBUG level (VESC_DEBUG_VERBOSE)
            debug_config.categories = c_uint16(0x0F)  # All categories (GENERAL, CAN, PARSER, COMM)
            debug_config.output_func = ctypes.cast(self.debug_output_func, ctypes.c_void_p)
            debug_config.enable_timestamps = c_bool(True)
            debug_config.enable_statistics = c_bool(True)
        except Exception as e:
            print(f"Warning: Failed to create debug configuration structure: {e}")
            return
        
        # Enable debugging
        if not vesc_lib.vesc_debug_configure(ctypes.byref(debug_config)):
            print("Warning: Failed to configure VESC CAN SDK debugging (vesc_debug_configure returned False)")
            return
        
        if not vesc_lib.vesc_debug_enable(2, 0x0F):  # Enable DEBUG level for all categories
            print("Warning: Failed to enable VESC CAN SDK debugging (vesc_debug_enable returned False)")
            return
        
        print("VESC CAN SDK debugging enabled")
    
    def get_debug_stats(self):
        """Get debug statistics"""
        if not self.enable_debug:
            return None
        
        stats = VescDebugStats()
        if vesc_lib.vesc_debug_get_stats(ctypes.byref(stats)):
            return {
                'rx_packets': stats.rx_packets,
                'tx_packets': stats.tx_packets,
                'rx_errors': stats.rx_errors,
                'tx_errors': stats.tx_errors
            }
        return None
    
    def _log_can_message(self, can_id: int, data: bytes, length: int, is_tx: bool = False):
        """Log a CAN message to file"""
        if not self.log_file:
            return
        
        # Get current timestamp
        now = datetime.now()
        timestamp = now.strftime("%Y/%m/%d %H.%M.%S,%f")[:-3]  # Format: YYYY/MM/DD HH.MM.SS,mmm
        
        # Determine message type (0=RX, 8=TX)
        msg_type = "8" if is_tx else "0"
        
        # Format CAN ID as hex (remove extended frame bit)
        can_id_hex = f"{can_id & 0x1FFFFFFF:x}"
        
        # Format data as hex string
        data_hex = ''.join([f"{b:02x}" for b in data[:length]])
        
        # Write log entry
        log_line = f"{timestamp};0;{msg_type};{can_id_hex};{length};{data_hex}\n"
        self.log_file.write(log_line)
        self.log_file.flush()  # Ensure data is written immediately
    
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
        while self.running:
            try:
                # Read CAN frames
                msg = self.can_obj.recv(timeout=0.001)
                if msg:
                    can_id = msg.arbitration_id
                    data = msg.data
                    length = len(data)
                    
                    # Log received message
                    self._log_can_message(can_id, data, length, is_tx=False)
                    
                    # Process frame
                    data_array = (c_uint8 * length)(*data)
                    vesc_lib.vesc_process_can_frame(can_id, data_array, length)
            
            except can.CanError:
                # No data available (non-blocking)
                pass
    
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
        time.sleep(20.0)  # Detection takes time
        return self.latest_motor_rl
    
    def print_status(self):
        """Print current status"""
        print("\n" + "="*60)
        print(f"VESC Monitor - Interface: {self.can_interface} ({self.bustype}, {self.baudrate} bps), ID: {self.vesc_id}")
        if self.enable_logging:
            print(f"CAN Logging: Enabled ({self.log_file.name if self.log_file else 'Unknown'})")
        if self.enable_debug:
            print("VESC Debug: Enabled")
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
        
        # Debug statistics
        if self.enable_debug:
            debug_stats = self.get_debug_stats()
            if debug_stats:
                print(f"\nDebug Statistics:")
                print(f"  RX Packets: {debug_stats['rx_packets']}")
                print(f"  TX Packets: {debug_stats['tx_packets']}")
                print(f"  RX Errors: {debug_stats['rx_errors']}")
                print(f"  TX Errors: {debug_stats['tx_errors']}")
        
        print("="*60)
    
    def run(self):
        """Run the monitor"""
        print(f"Starting VESC monitor on {self.can_interface} ({self.bustype}, {self.baudrate} bps) for VESC ID {self.vesc_id}")
        if self.enable_logging:
            print(f"CAN logging enabled: {self.log_file.name if self.log_file else 'Unknown'}")
        if self.enable_debug:
            print("VESC debugging enabled")
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
    parser.add_argument('interface', help='CAN interface (e.g., can0, vcan0, or full python-can URL like socketcan://can0). For CANalyst-II, this parameter is ignored.')
    parser.add_argument('--id', type=int, default=1, help='VESC controller ID (default: 1)')
    parser.add_argument('--detect-motor', action='store_true', help='Detect motor R/L parameters')
    parser.add_argument('--log', action='store_true', help='Enable CAN message logging to CSV file')
    parser.add_argument('--bustype', default='socketcan', help='CAN bus type (default: socketcan, options: socketcan, pcan, vector, canalystii, etc.). For CANalyst-II, install with: pip install "python-can[canalystii]"')
    parser.add_argument('--baudrate', type=int, default=500000, help='CAN bus baudrate in bits per second (default: 500000)')
    parser.add_argument('--debug', action='store_true', help='Enable VESC CAN SDK debugging output')
    
    args = parser.parse_args()
    
    try:
        monitor = VescMonitor(args.interface, args.id, args.log, args.bustype, args.baudrate, args.debug)
        
        if args.detect_motor:
            print("Detecting motor R/L parameters (20 seconds)...")
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