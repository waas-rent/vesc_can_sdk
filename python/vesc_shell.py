#!/usr/bin/env python3
"""
VESC CAN SDK - Python Interactive Shell

This script provides an interactive shell for communicating with VESC motor
controllers over CAN.

Copyright (c) 2025 waas AG (waas.rent)

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
from typing import Dict, Any, Optional, List
import ctypes
from ctypes import cdll, c_bool, c_uint8, c_uint32, c_float, c_int32, c_char, c_int16, c_uint16, c_uint64
import struct
from datetime import datetime
import os
import faulthandler
import cmd
import shlex

# Enable fault handler
faulthandler.enable()

# Try to import python-can
try:
    import can
except ImportError:
    print("Error: python-can library not found")
    print("Install it with: pip install python-can")
    sys.exit(1)

# Load the VESC CAN SDK library
try:
    vesc_lib = cdll.LoadLibrary("../lib/libvesc_can_sdk.so")
except OSError as e:
    print(f"Error: Could not load libvesc_can_sdk.so: {e}")
    print("Make sure to build the SDK first with 'make'")
    sys.exit(1)
except Exception as e:
    print(f"Unexpected error loading library: {e}")
    import traceback
    traceback.print_exc()
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

# Chuck values structure
class VescChuckValues(ctypes.Structure):
    _fields_ = [
        ("js_y", c_float),
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
        ("controller_id", c_uint8),
        ("rpm", c_float),
        ("current", c_float),
        ("duty", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg2(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
        ("amp_hours", c_float),
        ("amp_hours_charged", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg3(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
        ("watt_hours", c_float),
        ("watt_hours_charged", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg4(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
        ("temp_fet", c_float),
        ("temp_motor", c_float),
        ("current_in", c_float),
        ("pid_pos_now", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg5(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
        ("tacho_value", c_int32),
        ("v_in", c_float),
        ("valid", c_bool)
    ]

class VescStatusMsg6(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
        ("adc_1", c_float),
        ("adc_2", c_float),
        ("adc_3", c_float),
        ("ppm", c_float),
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
vesc_lib.vesc_can_init.argtypes = [ctypes.c_void_p, c_uint8, c_uint8]
vesc_lib.vesc_can_init.restype = c_bool

vesc_lib.vesc_set_response_callback.argtypes = [ctypes.c_void_p]

vesc_lib.vesc_process_can_frame.argtypes = [c_uint32, ctypes.POINTER(c_uint8), c_uint8]

vesc_lib.vesc_get_values.argtypes = [c_uint8]
vesc_lib.vesc_get_decoded_adc.argtypes = [c_uint8]
vesc_lib.vesc_get_decoded_ppm.argtypes = [c_uint8]
vesc_lib.vesc_get_fw_version.argtypes = [c_uint8]
vesc_lib.vesc_reboot.argtypes = [c_uint8]
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

vesc_lib.vesc_get_decoded_chuck = vesc_lib.vesc_get_decoded_chuck
vesc_lib.vesc_get_decoded_chuck.argtypes = [c_uint8]
vesc_lib.vesc_set_chuck_data = vesc_lib.vesc_set_chuck_data
vesc_lib.vesc_set_chuck_data.argtypes = [c_uint8, ctypes.c_void_p]
vesc_lib.vesc_parse_chuck_values = vesc_lib.vesc_parse_chuck_values
vesc_lib.vesc_parse_chuck_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescChuckValues)]
vesc_lib.vesc_parse_chuck_values.restype = c_bool

class VescShell(cmd.Cmd):
    intro = 'VESC CAN Shell. Type help or ? to list commands.\n'
    prompt = 'vesc> '
    
    def __init__(self, can_interface: str, vesc_id: int = 1, bustype: str = 'socketcan', 
                 baudrate: int = 500000, sender_id: int = 42):
        super().__init__()
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.sender_id = sender_id
        self.bustype = bustype
        self.baudrate = baudrate
        self.running = True
        self.verbose = False
        self.logging_enabled = False
        self.log_file = None
        self.session_start_time = datetime.now()
        
        # Response tracking
        self.response_received = threading.Event()
        self.response_data = None
        self.response_command = None
        
        # Latest data storage
        self.latest_values = {}
        self.latest_fw_version = None
        self.latest_adc_values = None
        self.latest_ppm_values = None
        self.latest_chuck_values = None
        self.latest_motor_rl = None
        
        # Status message storage
        self.latest_status_msg1 = None
        self.latest_status_msg2 = None
        self.latest_status_msg3 = None
        self.latest_status_msg4 = None
        self.latest_status_msg5 = None
        self.latest_status_msg6 = None
        
        # Status listening control
        self.status_listening = False
        self.status_listen_thread = None
        self.listen_command_filter = set()  # Empty set means listen to all commands
        
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
        if self.status_listening:
            self.status_listening = False
            print("\nStopped listening for status messages")
            return
        
        print(f"\nReceived signal {signum}, shutting down...")
        self.running = False
        if self.log_file:
            self.log_file.close()
        if hasattr(self, 'can_obj'):
            self.can_obj.shutdown()
        sys.exit(0)
    
    def _get_command_name(self, command_id: int) -> str:
        """Get human-readable name for a command ID"""
        command_names = {
            0: "FW_VERSION",
            4: "GET_VALUES", 
            9: "STATUS_1",
            14: "STATUS_2",
            15: "STATUS_3",
            16: "STATUS_4",
            25: "DETECT_MOTOR_R_L",
            27: "STATUS_5",
            29: "REBOOT",
            30: "GET_DECODED_ADC",
            31: "GET_DECODED_PPM",
            33: "GET_DECODED_CHUK",
            58: "STATUS_6"
        }
        return command_names.get(command_id, f"UNKNOWN_{command_id}")
    
    def _map_status_number_to_command_id(self, status_num: str) -> int:
        """Map status number (1-6) to internal command ID"""
        status_mapping = {
            "1": 9,   # STATUS_1
            "2": 14,  # STATUS_2
            "3": 15,  # STATUS_3
            "4": 16,  # STATUS_4
            "5": 27,  # STATUS_5
            "6": 58   # STATUS_6
        }
        return status_mapping.get(status_num)
    
    def _init_can(self):
        """Initialize CAN interface using python-can"""
        try:
            if self.bustype == 'canalystii':
                from can.interfaces.canalystii import CANalystIIBus
                self.can_obj = CANalystIIBus(channel=0, bitrate=self.baudrate)
            else:
                self.can_obj = can.Bus(interface=self.bustype, channel=self.can_interface, bitrate=self.baudrate)
        except Exception as e:
            print(f"Error initializing CAN interface: {e}")
            raise
    
    def _init_vesc_sdk(self):
        """Initialize VESC CAN SDK"""
        try:
            # Define CAN send function
            def can_send(id: int, data_ptr, length: int) -> bool:
                try:
                    data_bytes = bytes(data_ptr[:length])
                    can_id = id | 0x80000000  # Extended frame
                    padded_data = data_bytes + b'\x00' * (8 - length)
                    can_msg = can.Message(arbitration_id=can_id, data=padded_data, is_extended_id=True)
                    
                    if self.verbose:
                        print(f"[CAN TX] ID: 0x{can_id:08X}, Data: {data_bytes.hex()}")
                    
                    self.can_obj.send(can_msg)
                    self._log_can_message(id, data_bytes, length, is_tx=True)
                    return True
                except Exception as e:
                    print(f"CAN send error: {e}")
                    return False
            
            self.can_send_func = ctypes.CFUNCTYPE(c_bool, c_uint32, ctypes.POINTER(c_uint8), c_uint8)(can_send)
            
            if not vesc_lib.vesc_can_init(self.can_send_func, self.vesc_id, self.sender_id):
                raise RuntimeError("Failed to initialize VESC CAN SDK")
            
            # Set response callback
            def response_callback(controller_id: int, command: int, data_ptr, length: int):
                try:
                    self._handle_response(controller_id, command, data_ptr, length)
                except Exception as e:
                    print(f"Error in response callback: {e}")
            
            self.response_callback = ctypes.CFUNCTYPE(None, c_uint8, c_uint8, ctypes.POINTER(c_uint8), c_uint8)(response_callback)
            vesc_lib.vesc_set_response_callback(self.response_callback)
            
        except Exception as e:
            print(f"ERROR during VESC SDK initialization: {e}")
            raise
    
    def _log_can_message(self, can_id: int, data: bytes, length: int, is_tx: bool = False):
        """Log a CAN message to file"""
        if not self.log_file:
            return
        
        now = datetime.now()
        timestamp = now.strftime("%Y/%m/%d %H.%M.%S,%f")[:-3]
        msg_type = "8" if is_tx else "0"
        can_id_hex = f"{can_id & 0x1FFFFFFF:x}"
        data_hex = ''.join([f"{b:02x}" for b in data[:length]])
        
        log_line = f"{timestamp};0;{msg_type};{can_id_hex};{length};{data_hex}\n"
        self.log_file.write(log_line)
        self.log_file.flush()
    
    def _handle_response(self, controller_id: int, command: int, data: bytes, length: int):
        """Handle VESC response"""
        try:
            if hasattr(data, 'contents'):
                data_array = ctypes.cast(data, ctypes.POINTER(c_uint8 * length)).contents
            elif isinstance(data, (bytes, bytearray)):
                data_bytes = bytes(data[:length])
                data_array = (c_uint8 * length)(*data_bytes)
            else:
                data_bytes = bytes(data[:length])
                data_array = (c_uint8 * length)(*data_bytes)
            
            if self.verbose:
                data_bytes = bytes(data_array)
                print(f"[CAN RX] Controller: {controller_id}, Command: {command}, Data: {data_bytes.hex()}")
            
            # Set response received event
            self.response_received.set()
            self.response_data = data_array
            self.response_command = command
            
            if command == 4:  # COMM_GET_VALUES
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
                        'cfg_num': version.cfg_num,
                        'uuid': version.uuid
                    }
            
            elif command == 32:  # COMM_GET_DECODED_ADC
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
            
            elif command == 33:  # COMM_GET_DECODED_CHUK
                chuck = VescChuckValues()
                if vesc_lib.vesc_parse_chuck_values(data_array, length, ctypes.byref(chuck)):
                    self.latest_chuck_values = {
                        'js_y': chuck.js_y
                    }
            
            elif command == 25:  # COMM_DETECT_MOTOR_R_L
                motor_rl = VescMotorRLResponse()
                if vesc_lib.vesc_parse_motor_rl_response(data_array, length, ctypes.byref(motor_rl)):
                    self.latest_motor_rl = {
                        'resistance': motor_rl.resistance,
                        'inductance': motor_rl.inductance,
                        'ld_lq_diff': motor_rl.ld_lq_diff
                    }
            
            # Handle status messages
            elif command == 9:  # COMM_GET_STATUS_1
                try:
                    status1 = VescStatusMsg1()
                    if vesc_lib.vesc_parse_status_msg_1(data_array, length, ctypes.byref(status1)):
                        self.latest_status_msg1 = {
                            'controller_id': status1.controller_id,
                            'rpm': status1.rpm,
                            'current': status1.current,
                            'duty': status1.duty
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status1] Controller: {status1.controller_id}, "
                                  f"RPM: {status1.rpm:.0f}, Current: {status1.current:.2f}A, "
                                  f"Duty: {status1.duty*100:.1f}%")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status1] Error parsing status message: {e}")
            
            elif command == 14:  # COMM_GET_STATUS_2
                try:
                    status2 = VescStatusMsg2()
                    if vesc_lib.vesc_parse_status_msg_2(data_array, length, ctypes.byref(status2)):
                        self.latest_status_msg2 = {
                            'controller_id': status2.controller_id,
                            'amp_hours': status2.amp_hours,
                            'amp_hours_charged': status2.amp_hours_charged
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status2] Controller: {status2.controller_id}, "
                                  f"Ah: {status2.amp_hours:.2f}, Ah_charged: {status2.amp_hours_charged:.2f}")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status2] Error parsing status message: {e}")
            
            elif command == 15:  # COMM_GET_STATUS_3
                try:
                    status3 = VescStatusMsg3()
                    if vesc_lib.vesc_parse_status_msg_3(data_array, length, ctypes.byref(status3)):
                        self.latest_status_msg3 = {
                            'controller_id': status3.controller_id,
                            'watt_hours': status3.watt_hours,
                            'watt_hours_charged': status3.watt_hours_charged
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status3] Controller: {status3.controller_id}, "
                                  f"Wh: {status3.watt_hours:.2f}, Wh_charged: {status3.watt_hours_charged:.2f}")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status3] Error parsing status message: {e}")
            
            elif command == 16:  # COMM_GET_STATUS_4
                try:
                    status4 = VescStatusMsg4()
                    if vesc_lib.vesc_parse_status_msg_4(data_array, length, ctypes.byref(status4)):
                        self.latest_status_msg4 = {
                            'controller_id': status4.controller_id,
                            'temp_fet': status4.temp_fet,
                            'temp_motor': status4.temp_motor,
                            'current_in': status4.current_in,
                            'pid_pos_now': status4.pid_pos_now
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status4] Controller: {status4.controller_id}, "
                                  f"Temp_FET: {status4.temp_fet:.1f}°C, Temp_Motor: {status4.temp_motor:.1f}°C, "
                                  f"Current_In: {status4.current_in:.2f}A, PID_Pos: {status4.pid_pos_now:.2f}")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status4] Error parsing status message: {e}")
            
            elif command == 27:  # COMM_GET_STATUS_5
                try:
                    status5 = VescStatusMsg5()
                    if vesc_lib.vesc_parse_status_msg_5(data_array, length, ctypes.byref(status5)):
                        self.latest_status_msg5 = {
                            'controller_id': status5.controller_id,
                            'tacho_value': status5.tacho_value,
                            'v_in': status5.v_in
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status5] Controller: {status5.controller_id}, "
                                  f"Tacho: {status5.tacho_value}, V_in: {status5.v_in:.1f}V")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status5] Error parsing status message: {e}")
            
            elif command == 58:  # COMM_GET_STATUS_6
                try:
                    # Validate data length before parsing
                    if length < 8:  # Status6 should have at least 8 bytes (controller_id + 4 floats)
                        if self.verbose:
                            print(f"[Status6] Invalid data length: {length}")
                        return
                    
                    status6 = VescStatusMsg6()
                    if vesc_lib.vesc_parse_status_msg_6(data_array, length, ctypes.byref(status6)):
                        self.latest_status_msg6 = {
                            'controller_id': status6.controller_id,
                            'adc_1': status6.adc_1,
                            'adc_2': status6.adc_2,
                            'adc_3': status6.adc_3,
                            'ppm': status6.ppm
                        }
                        if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                            print(f"[Status6] Controller: {status6.controller_id}, "
                                  f"ADC1: {status6.adc_1:.3f}, ADC2: {status6.adc_2:.3f}, "
                                  f"ADC3: {status6.adc_3:.3f}, PPM: {status6.ppm:.3f}")
                    else:
                        if self.verbose:
                            print(f"[Status6] Failed to parse status message")
                except Exception as e:
                    if self.verbose:
                        print(f"[Status6] Error parsing status message: {e}")
                    # Don't let parsing errors crash the application
            
            # General message handler for listening mode
            if self.status_listening and (not self.listen_command_filter or command in self.listen_command_filter):
                # Only print if not already handled by specific status message handlers
                if command not in [9, 14, 15, 16, 27, 58]:  # Status messages are handled above
                    data_bytes = bytes(data_array)
                    command_name = self._get_command_name(command)
                    print(f"[{command_name}] Controller: {controller_id}, "
                          f"Command: {command}, Data: {data_bytes.hex()}")
            
            self._log_can_message(controller_id, data_array, length, is_tx=False)
            
        except Exception as e:
            print(f"Error in _handle_response: {e}")
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                msg = self.can_obj.recv(timeout=0.001)
                if msg:
                    if self.verbose:
                        print(f"[CAN RX] ID: 0x{msg.arbitration_id:08X}, Data: {msg.data.hex()}")
                    
                    can_id = msg.arbitration_id
                    data = msg.data
                    length = len(data)
                    
                    self._log_can_message(can_id, data, length, is_tx=False)
                    
                    try:
                        data_array = (c_uint8 * length)(*data)
                        vesc_lib.vesc_process_can_frame(can_id, data_array, length)
                    except Exception as e:
                        print(f"Error processing CAN frame: {e}")
            
            except can.CanError:
                pass
            except Exception as e:
                print(f"Unexpected error in monitor loop: {e}")
                time.sleep(0.1)
    
    def _wait_for_response(self, timeout: float = 3.0) -> bool:
        """Wait for response with timeout"""
        self.response_received.clear()
        self.response_data = None
        self.response_command = None
        
        if self.response_received.wait(timeout):
            return True
        else:
            print(f"Timeout waiting for response (after {timeout}s)")
            return False
    
    def _get_firmware_version(self):
        """Get firmware version with timeout"""
        print("Getting firmware version...")
        vesc_lib.vesc_get_fw_version(self.vesc_id)
        
        if self._wait_for_response(3.0):
            if self.latest_fw_version:
                uuid_as_bytes = bytes(self.latest_fw_version['uuid'])
                print(f"VESC {self.latest_fw_version['major']}.{self.latest_fw_version['minor']} "
                      f"{self.latest_fw_version['hw_name']} with UUID: {uuid_as_bytes.hex()}")
                return True
            else:
                print("Failed to parse firmware version")
                return False
        else:
            print("Failed to get firmware version")
            return False
    
    # Command implementations
    def do_help(self, arg):
        """Show help for commands"""
        if arg:
            # Show help for specific command
            super().do_help(arg)
        else:
            # Show general help
            print("\nAvailable commands:")
            print("  values          - Get motor values (temperature, current, RPM, etc.)")
            print("  adc             - Get ADC values")
            print("  ppm             - Get PPM values")
            print("  chuck           - Get decoded chuck (nunchuk) data")
            print("  setchuck        - Set chuck (nunchuk) data")
            print("  motor_rl        - Detect motor resistance and inductance")
            print("  fw_version      - Get firmware version")
            print("  reboot          - Reboot the VESC controller")
            print("  raw_can         - Send raw CAN message")
            print("  listen          - Listen for VESC messages (with optional filtering)")
            print("  verbose         - Toggle verbose logging")
            print("  logging         - Toggle CAN message logging to file")
            print("  status          - Show current status")
            print("  quit/exit       - Exit the shell")
            print("\nType 'help <command>' for detailed help on a specific command.")
    
    def help_values(self):
        """Help for values command"""
        print("values - Get motor values")
        print("  Retrieves current motor status including temperature, current, RPM, voltage, etc.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: values")
    
    def do_values(self, arg):
        """Get motor values"""
        if arg:
            print("Error: values command takes no arguments")
            return
        
        print("Getting motor values...")
        vesc_lib.vesc_get_values(self.vesc_id)
        
        if self._wait_for_response(3.0):
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
            else:
                print("Failed to parse motor values")
        else:
            print("Failed to get motor values")
    
    def help_adc(self):
        """Help for adc command"""
        print("adc - Get ADC values")
        print("  Retrieves decoded ADC values from the VESC.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: adc")
    
    def do_adc(self, arg):
        """Get ADC values"""
        if arg:
            print("Error: adc command takes no arguments")
            return
        
        print("Getting ADC values...")
        vesc_lib.vesc_get_decoded_adc(self.vesc_id)
        
        if self._wait_for_response(3.0):
            if self.latest_adc_values:
                print(f"\nADC Values:")
                print(f"  ADC1: {self.latest_adc_values['adc1']:.3f}")
                print(f"  ADC2: {self.latest_adc_values['adc2']:.3f}")
                print(f"  ADC3: {self.latest_adc_values['adc3']:.3f}")
                print(f"  Input Voltage: {self.latest_adc_values['v_in']:.1f}V")
            else:
                print("Failed to parse ADC values")
        else:
            print("Failed to get ADC values")
    
    def help_ppm(self):
        """Help for ppm command"""
        print("ppm - Get PPM values")
        print("  Retrieves decoded PPM values from the VESC.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: ppm")
    
    def do_ppm(self, arg):
        """Get PPM values"""
        if arg:
            print("Error: ppm command takes no arguments")
            return
        
        print("Getting PPM values...")
        vesc_lib.vesc_get_decoded_ppm(self.vesc_id)
        
        if self._wait_for_response(3.0):
            if self.latest_ppm_values:
                print(f"\nPPM Values:")
                print(f"  PPM: {self.latest_ppm_values['ppm']:.3f}")
                print(f"  Pulse Length: {self.latest_ppm_values['pulse_len']:.1f}μs")
            else:
                print("Failed to parse PPM values")
        else:
            print("Failed to get PPM values")
    
    def help_motor_rl(self):
        """Help for motor_rl command"""
        print("motor_rl - Detect motor resistance and inductance")
        print("  Performs motor R/L detection (takes up to 20 seconds).")
        print("  Waits up to 20 seconds for response.")
        print("  Usage: motor_rl")
    
    def do_motor_rl(self, arg):
        """Detect motor resistance and inductance"""
        if arg:
            print("Error: motor_rl command takes no arguments")
            return
        
        print("Detecting motor R/L parameters (this may take up to 20 seconds)...")
        vesc_lib.vesc_detect_motor_r_l(self.vesc_id)
        
        if self._wait_for_response(20.0):
            if self.latest_motor_rl:
                print(f"\nMotor R/L Detection Results:")
                print(f"  Resistance: {self.latest_motor_rl['resistance']:.6f}Ω")
                print(f"  Inductance: {self.latest_motor_rl['inductance']:.8f}H")
                print(f"  Ld-Lq Difference: {self.latest_motor_rl['ld_lq_diff']:.8f}H")
            else:
                print("Failed to parse motor R/L values")
        else:
            print("Failed to get motor R/L values")
    
    def help_fw_version(self):
        """Help for fw_version command"""
        print("fw_version - Get firmware version")
        print("  Retrieves the firmware version information from the VESC.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: fw_version")
    
    def do_fw_version(self, arg):
        """Get firmware version"""
        if arg:
            print("Error: fw_version command takes no arguments")
            return
        
        self._get_firmware_version()
    
    def help_reboot(self):
        """Help for reboot command"""
        print("reboot - Reboot the VESC controller")
        print("  Sends a reboot command to the VESC controller.")
        print("  The controller will restart after receiving this command.")
        print("  Usage: reboot")
        print("  Warning: This will disconnect the controller temporarily.")
    
    def do_reboot(self, arg):
        """Reboot the VESC controller"""
        if arg:
            print("Error: reboot command takes no arguments")
            return
        
        print("Rebooting VESC controller...")
        print("Warning: The controller will restart and may be temporarily unavailable.")
        
        # Send reboot command
        vesc_lib.vesc_reboot(self.vesc_id)
        
        print("Reboot command sent successfully.")
        print("The VESC controller should restart momentarily.")
    
    def help_raw_can(self):
        """Help for raw_can command"""
        print("raw_can - Send raw CAN message")
        print("  Sends a raw CAN message with specified ID and data.")
        print("  Usage: raw_can <can_id> <data_hex>")
        print("  Example: raw_can 0x123 01 02 03 04")
        print("  Note: Data should be provided as space-separated hex bytes")
    
    def do_raw_can(self, arg):
        """Send raw CAN message"""
        if not arg:
            print("Error: raw_can requires arguments")
            print("Usage: raw_can <can_id> <data_hex>")
            return
        
        try:
            args = shlex.split(arg)
            if len(args) < 2:
                print("Error: raw_can requires at least can_id and data")
                return
            
            # Parse CAN ID
            can_id_str = args[0]
            if can_id_str.startswith('0x'):
                can_id = int(can_id_str, 16)
            else:
                can_id = int(can_id_str)
            
            # Parse data bytes
            data_bytes = []
            for hex_byte in args[1:]:
                if hex_byte.startswith('0x'):
                    data_bytes.append(int(hex_byte, 16))
                else:
                    data_bytes.append(int(hex_byte, 16))
            
            # Send CAN message
            can_id_extended = can_id | 0x80000000
            padded_data = bytes(data_bytes) + b'\x00' * (8 - len(data_bytes))
            can_msg = can.Message(arbitration_id=can_id_extended, data=padded_data, is_extended_id=True)
            
            self.can_obj.send(can_msg)
            print(f"Sent CAN message: ID=0x{can_id:08X}, Data={bytes(data_bytes).hex()}")
            
            self._log_can_message(can_id, bytes(data_bytes), len(data_bytes), is_tx=True)
            
        except ValueError as e:
            print(f"Error parsing arguments: {e}")
        except Exception as e:
            print(f"Error sending CAN message: {e}")
    
    def help_listen(self):
        """Help for listen command"""
        print("listen - Listen for VESC messages")
        print("  Starts listening for VESC messages and prints them as they arrive.")
        print("  Press Ctrl+C to stop listening.")
        print("  Usage: listen [command1 command2 ...]")
        print("  Examples:")
        print("    listen                    - Listen to all messages")
        print("    listen 1 2 3              - Listen only to Status1, Status2, Status3")
        print("    listen 4                  - Listen only to GET_VALUES")
        print("    listen all                - Listen to all messages (same as no args)")
        print("  Status messages: 1(STATUS_1), 2(STATUS_2), 3(STATUS_3), 4(STATUS_4), 5(STATUS_5), 6(STATUS_6)")

    def do_listen(self, arg):
        """Listen for VESC messages"""
        if self.status_listening:
            print("Already listening for messages")
            return
        
        # Parse command filter
        self.listen_command_filter = set()
        if arg:
            args = shlex.split(arg)
            for cmd_arg in args:
                if cmd_arg.lower() == 'all':
                    self.listen_command_filter = set()  # Empty set means listen to all
                    break
                
                # Check if it's a status number (1-6)
                status_cmd_id = self._map_status_number_to_command_id(cmd_arg)
                if status_cmd_id is not None:
                    self.listen_command_filter.add(status_cmd_id)
                    continue
                
                # Try as direct command ID
                try:
                    cmd_id = int(cmd_arg)
                    self.listen_command_filter.add(cmd_id)
                except ValueError:
                    print(f"Error: '{cmd_arg}' is not a valid command ID or status number (1-6)")
                    return
        
        # Show what we're listening to
        if not self.listen_command_filter:
            print("Listening for all VESC messages... Press Ctrl+C to stop")
        else:
            cmd_names = [f"{cmd_id}({self._get_command_name(cmd_id)})" for cmd_id in sorted(self.listen_command_filter)]
            print(f"Listening for commands: {', '.join(cmd_names)}... Press Ctrl+C to stop")
        
        self.status_listening = True
        
        try:
            # Keep listening until interrupted
            while self.status_listening and self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.status_listening = False
            self.listen_command_filter = set()  # Reset filter
    
    def help_verbose(self):
        """Help for verbose command"""
        print("verbose - Toggle verbose logging")
        print("  Toggles verbose CAN message logging on/off.")
        print("  When enabled, shows all CAN messages sent and received.")
        print("  Usage: verbose")
    
    def do_verbose(self, arg):
        """Toggle verbose logging"""
        if arg:
            print("Error: verbose command takes no arguments")
            return
        
        self.verbose = not self.verbose
        status = "enabled" if self.verbose else "disabled"
        print(f"Verbose logging {status}")
    
    def help_logging(self):
        """Help for logging command"""
        print("logging - Toggle CAN message logging to file")
        print("  Toggles logging of all CAN messages to a CSV file.")
        print("  Log file is created with timestamp in the current directory.")
        print("  Usage: logging")
    
    def do_logging(self, arg):
        """Toggle CAN message logging to file"""
        if arg:
            print("Error: logging command takes no arguments")
            return
        
        if self.logging_enabled:
            # Disable logging
            if self.log_file:
                self.log_file.close()
                self.log_file = None
            self.logging_enabled = False
            print("CAN logging disabled")
        else:
            # Enable logging
            timestamp = self.session_start_time.strftime("%Y%m%d_%H%M%S")
            log_filename = f"vesc_can_log_{timestamp}.csv"
            
            try:
                self.log_file = open(log_filename, 'w')
                
                # Write header
                self.log_file.write("# Logger type: VESC CAN SDK Python Shell\n")
                self.log_file.write("# HW rev: VESC CAN SDK\n")
                self.log_file.write("# FW rev: Python Shell\n")
                self.log_file.write(f"# Logger ID: VESC_Shell_{self.vesc_id}_{self.sender_id}\n")
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
                
                self.logging_enabled = True
                print(f"CAN logging enabled: {log_filename}")
                
            except Exception as e:
                print(f"Error enabling logging: {e}")
    
    def help_status(self):
        """Help for status command"""
        print("status - Show current status")
        print("  Shows current VESC status including firmware version, motor values,")
        print("  ADC values, PPM values, and motor R/L if available.")
        print("  Usage: status")
    
    def do_status(self, arg):
        """Show current status"""
        if arg:
            print("Error: status command takes no arguments")
            return
        
        print("\n" + "="*60)
        print(f"VESC Shell - Interface: {self.can_interface} ({self.bustype}, {self.baudrate} bps)")
        print(f"VESC ID: {self.vesc_id}, Sender ID: {self.sender_id}")
        print(f"Verbose: {'Enabled' if self.verbose else 'Disabled'}")
        print(f"Logging: {'Enabled' if self.logging_enabled else 'Disabled'}")
        if self.logging_enabled and self.log_file:
            print(f"Log file: {self.log_file.name}")
        print("="*60)
        
        # Firmware version
        if self.latest_fw_version:
            print(f"Firmware: {self.latest_fw_version['major']}.{self.latest_fw_version['minor']}")
            print(f"Hardware: {self.latest_fw_version['hw_name']}")
            print(f"Hardware Type: {self.latest_fw_version['hw_type']}")
        else:
            print("Firmware: Not available")
        
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
        else:
            print("\nMotor Status: Not available")
        
        # ADC values
        if self.latest_adc_values:
            print(f"\nADC Values:")
            print(f"  ADC1: {self.latest_adc_values['adc1']:.3f}")
            print(f"  ADC2: {self.latest_adc_values['adc2']:.3f}")
            print(f"  ADC3: {self.latest_adc_values['adc3']:.3f}")
            print(f"  Input Voltage: {self.latest_adc_values['v_in']:.1f}V")
        else:
            print("\nADC Values: Not available")
        
        # PPM values
        if self.latest_ppm_values:
            print(f"\nPPM Values:")
            print(f"  PPM: {self.latest_ppm_values['ppm']:.3f}")
            print(f"  Pulse Length: {self.latest_ppm_values['pulse_len']:.1f}μs")
        else:
            print("\nPPM Values: Not available")
        
        # Chuck values
        if hasattr(self, 'latest_chuck_values') and self.latest_chuck_values:
            print(f"\nChuck Values:")
            print(f"  Joystick Y: {self.latest_chuck_values['js_y']:.6f}")
        else:
            print("\nChuck Values: Not available")
        
        # Motor R/L
        if self.latest_motor_rl:
            print(f"\nMotor R/L Detection:")
            print(f"  Resistance: {self.latest_motor_rl['resistance']:.6f}Ω")
            print(f"  Inductance: {self.latest_motor_rl['inductance']:.8f}H")
            print(f"  Ld-Lq Difference: {self.latest_motor_rl['ld_lq_diff']:.8f}H")
        else:
            print("\nMotor R/L: Not available")
        
        # Status messages
        print(f"\nStatus Messages:")
        if self.latest_status_msg1:
            print(f"  Status1: Controller: {self.latest_status_msg1['controller_id']}, "
                  f"RPM: {self.latest_status_msg1['rpm']:.0f}, Current: {self.latest_status_msg1['current']:.2f}A, "
                  f"Duty: {self.latest_status_msg1['duty']*100:.1f}%")
        else:
            print("  Status1: Not available")
        
        if self.latest_status_msg2:
            print(f"  Status2: Controller: {self.latest_status_msg2['controller_id']}, "
                  f"Ah: {self.latest_status_msg2['amp_hours']:.2f}, Ah_charged: {self.latest_status_msg2['amp_hours_charged']:.2f}")
        else:
            print("  Status2: Not available")
        
        if self.latest_status_msg3:
            print(f"  Status3: Controller: {self.latest_status_msg3['controller_id']}, "
                  f"Wh: {self.latest_status_msg3['watt_hours']:.2f}, Wh_charged: {self.latest_status_msg3['watt_hours_charged']:.2f}")
        else:
            print("  Status3: Not available")
        
        if self.latest_status_msg4:
            print(f"  Status4: Controller: {self.latest_status_msg4['controller_id']}, "
                  f"Temp_FET: {self.latest_status_msg4['temp_fet']:.1f}°C, "
                  f"Temp_Motor: {self.latest_status_msg4['temp_motor']:.1f}°C, "
                  f"Current_In: {self.latest_status_msg4['current_in']:.2f}A, "
                  f"PID_Pos: {self.latest_status_msg4['pid_pos_now']:.2f}")
        else:
            print("  Status4: Not available")
        
        if self.latest_status_msg5:
            print(f"  Status5: Controller: {self.latest_status_msg5['controller_id']}, "
                  f"Tacho: {self.latest_status_msg5['tacho_value']}, "
                  f"V_in: {self.latest_status_msg5['v_in']:.1f}V")
        else:
            print("  Status5: Not available")
        
        if self.latest_status_msg6:
            print(f"  Status6: Controller: {self.latest_status_msg6['controller_id']}, "
                  f"ADC1: {self.latest_status_msg6['adc_1']:.3f}, ADC2: {self.latest_status_msg6['adc_2']:.3f}, "
                  f"ADC3: {self.latest_status_msg6['adc_3']:.3f}, PPM: {self.latest_status_msg6['ppm']:.3f}")
        else:
            print("  Status6: Not available")
        
        print("="*60)
    
    def do_quit(self, arg):
        """Exit the shell"""
        print("Exiting...")
        self.running = False
        if self.log_file:
            self.log_file.close()
        if hasattr(self, 'can_obj'):
            self.can_obj.shutdown()
        return True
    
    def do_exit(self, arg):
        """Exit the shell"""
        return self.do_quit(arg)
    
    def do_EOF(self, arg):
        """Exit on EOF (Ctrl+D)"""
        return self.do_quit(arg)

    def help_chuck(self):
        print("chuck - Get decoded chuck (nunchuk) data")
        print("  Retrieves decoded chuck data from the VESC.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: chuck")

    def do_chuck(self, arg):
        if arg:
            print("Error: chuck command takes no arguments")
            return
        print("Getting chuck data...")
        vesc_lib.vesc_get_decoded_chuck(self.vesc_id)
        if self._wait_for_response(3.0):
            if hasattr(self, 'latest_chuck_values') and self.latest_chuck_values:
                print(f"\nChuck Values:")
                print(f"  Joystick Y: {self.latest_chuck_values['js_y']:.6f}")
            else:
                print("Failed to parse chuck values")
        else:
            print("Failed to get chuck values")

    def help_setchuck(self):
        print("setchuck - Set chuck (nunchuk) data")
        print("  Sends chuck data to the VESC.")
        print("  Usage: setchuck <js_x> <js_y> <bt_c> <bt_z> <acc_x> <acc_y> <acc_z> [rev_has_state] [is_rev]")
        print("  Example: setchuck 128 200 0 1 1000 2000 3000 1 0")

    def do_setchuck(self, arg):
        args = shlex.split(arg)
        if len(args) < 7:
            print("Usage: setchuck <js_x> <js_y> <bt_c> <bt_z> <acc_x> <acc_y> <acc_z> [rev_has_state] [is_rev]")
            return
        try:
            js_x = int(args[0])
            js_y = int(args[1])
            bt_c = int(args[2])
            bt_z = int(args[3])
            acc_x = int(args[4])
            acc_y = int(args[5])
            acc_z = int(args[6])
            rev_has_state = bool(int(args[7])) if len(args) > 7 else False
            is_rev = bool(int(args[8])) if len(args) > 8 else False
            # Build C struct for chuck data
            class VescChuckData(ctypes.Structure):
                _fields_ = [
                    ("js_x", c_uint8),
                    ("js_y", c_uint8),
                    ("bt_c", c_uint8),
                    ("bt_z", c_uint8),
                    ("acc_x", c_int16),
                    ("acc_y", c_int16),
                    ("acc_z", c_int16),
                    ("rev_has_state", c_bool),
                    ("is_rev", c_bool)
                ]
            chuck_data = VescChuckData(js_x, js_y, bt_c, bt_z, acc_x, acc_y, acc_z, rev_has_state, is_rev)
            vesc_lib.vesc_set_chuck_data(self.vesc_id, ctypes.byref(chuck_data))
            print("Chuck data sent.")
        except Exception as e:
            print(f"Error: {e}")

def main():
    parser = argparse.ArgumentParser(description='VESC CAN Interactive Shell')
    parser.add_argument('interface', help='CAN interface (e.g., can0, vcan0, or full python-can URL like socketcan://can0). For CANalyst-II, this parameter is ignored.')
    parser.add_argument('--id', type=int, default=1, help='VESC controller ID (default: 1)')
    parser.add_argument('--bustype', default='socketcan', help='CAN bus type (default: socketcan, options: socketcan, pcan, vector, canalystii, etc.). For CANalyst-II, install with: pip install "python-can[canalystii]"')
    parser.add_argument('--baudrate', type=int, default=500000, help='CAN bus baudrate in bits per second (default: 500000)')
    parser.add_argument('--sender-id', type=int, default=42, help='Sender ID for CAN messages (default: 42)')
    
    args = parser.parse_args()
    
    try:
        shell = VescShell(args.interface, args.id, args.bustype, args.baudrate, args.sender_id)
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\nShell stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 