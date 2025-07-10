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

# Import VESC CAN SDK bindings
try:
    from vesc_can_sdk import (
        vesc_lib, VescValues, VescMotorRLResponse, VescFwVersion, VescAdcValues, 
        VescPpmValues, VescChuckValues, VescChuckData, VescMotorParamResponse, 
        VescFluxLinkageResponse, VescStatusMsg1, VescStatusMsg2, VescStatusMsg3, 
        VescStatusMsg4, VescStatusMsg5, VescStatusMsg6, VescPongResponse, 
        VescDebugConfig, VescDebugStats, VESC_DEBUG_NONE, VESC_DEBUG_BASIC, 
        VESC_DEBUG_DETAILED, VESC_DEBUG_VERBOSE, VESC_DEBUG_CAN, VESC_DEBUG_COMMANDS, 
        VESC_DEBUG_RESPONSES, VESC_DEBUG_BUFFERS, VESC_DEBUG_ERRORS, VESC_DEBUG_PERFORMANCE, 
        VESC_DEBUG_ALL, get_command_name, map_status_number_to_command_id,
        c_bool, c_uint8, c_uint32, c_float, c_int32, c_char, c_int16, c_uint16, c_uint64
    )
except ImportError as e:
    print(f"Error: Could not import VESC CAN SDK bindings: {e}")
    print("Make sure vesc_can_sdk.py is in the same directory")
    sys.exit(1)





class VescShell(cmd.Cmd):
    intro = 'VESC CAN Shell. Type help or ? to list commands.'
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
        self.processing_response = False  # Flag to track when we're processing a VESC response

        self.blocking_response_received = threading.Event()
        self.blocking_response_data = None
        self.blocking_response_command = None
        self.blocking_response_timeout = 10.0
        
        # Latest data storage
        self.latest_values = {}
        self.latest_fw_version = None
        self.latest_adc_values = None
        self.latest_ppm_values = None
        self.latest_chuck_values = None
        self.latest_motor_rl = None
        self.latest_flux_linkage = None
        self.latest_pong = None
        
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

        self.connected = False
        
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
                    can_msg = can.Message(arbitration_id=can_id, data=padded_data, is_extended_id=True, dlc=length)
                    
                    if self.verbose:
                        # Extract command from first byte if available
                        command_name = "UNKNOWN"
                        if length > 0:
                            command_id = data_bytes[0]
                            command_name = get_command_name(command_id)
                        print(f"[CAN TX] ID: 0x{can_id:08X}, Command: {command_name}({data_bytes[0] if length > 0 else 'N/A'}), Data: {data_bytes.hex()}")
                    
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
        
        # Extract command/packet name if this is a VESC message
        comment = ""
        can_id_base = can_id & 0x1FFFFFFF
        controller_id = can_id_base & 0xFF
        packet_type = (can_id_base >> 8) & 0xFF
        
        # Check if this is a VESC message by looking at the packet type
        # VESC uses packet types 0-63, so any packet type in this range is a VESC message
        if packet_type <= 63:
            # Check if length > 8 - this indicates a summary/buffer dump, not a valid CAN message
            if length > 8:
                comment = "SUMMARY"
            # For PROCESS_RX_BUFFER, always label as such
            elif packet_type == 7:
                comment = "PROCESS_RX_BUFFER"
            # For PROCESS_SHORT_BUFFER, only use command name if data[2] is a valid command
            elif packet_type == 8:
                if length > 2:
                    command_id = data[2]
                    valid_command_ids = {0, 4, 14, 25, 26, 27, 29, 31, 32, 33, 34, 35, 158}
                    if command_id in valid_command_ids:
                        command_name = self._get_command_name(command_id)
                        comment = command_name
                    else:
                        comment = "PROCESS_SHORT_BUFFER"
                else:
                    comment = "PROCESS_SHORT_BUFFER"
            # Check if this is a direct CAN packet
            else:
                packet_names = {
                    0: "SET_DUTY", 1: "SET_CURRENT", 2: "SET_CURRENT_BRAKE", 3: "SET_RPM", 4: "SET_POS",
                    5: "FILL_RX_BUFFER", 6: "FILL_RX_BUFFER_LONG", 7: "PROCESS_RX_BUFFER", 8: "PROCESS_SHORT_BUFFER",
                    9: "STATUS_1", 10: "SET_CURRENT_REL", 11: "SET_CURRENT_BRAKE_REL", 12: "SET_CURRENT_HANDBRAKE",
                    13: "SET_CURRENT_HANDBRAKE_REL", 14: "STATUS_2", 15: "STATUS_3", 16: "STATUS_4",
                    17: "PING", 18: "PONG", 19: "DETECT_APPLY_ALL_FOC", 20: "DETECT_APPLY_ALL_FOC_RES",
                    21: "CONF_CURRENT_LIMITS", 22: "CONF_STORE_CURRENT_LIMITS", 23: "CONF_CURRENT_LIMITS_IN",
                    24: "CONF_STORE_CURRENT_LIMITS_IN", 25: "CONF_FOC_ERPMS", 26: "CONF_STORE_FOC_ERPMS",
                    27: "STATUS_5", 28: "POLL_TS5700N8501_STATUS", 29: "CONF_BATTERY_CUT", 30: "CONF_STORE_BATTERY_CUT",
                    31: "SHUTDOWN", 32: "IO_BOARD_ADC_1_TO_4", 33: "IO_BOARD_ADC_5_TO_8", 34: "IO_BOARD_ADC_9_TO_12",
                    35: "IO_BOARD_DIGITAL_IN", 36: "IO_BOARD_SET_OUTPUT_DIGITAL", 37: "IO_BOARD_SET_OUTPUT_PWM",
                    38: "BMS_V_TOT", 39: "BMS_I", 40: "BMS_AH_WH", 41: "BMS_V_CELL", 42: "BMS_BAL",
                    43: "BMS_TEMPS", 44: "BMS_HUM", 45: "BMS_SOC_SOH_TEMP_STAT", 46: "PSW_STAT",
                    47: "PSW_SWITCH", 48: "BMS_HW_DATA_1", 49: "BMS_HW_DATA_2", 50: "BMS_HW_DATA_3",
                    51: "BMS_HW_DATA_4", 52: "BMS_HW_DATA_5", 53: "BMS_AH_WH_CHG_TOTAL", 54: "BMS_AH_WH_DIS_TOTAL",
                    55: "UPDATE_PID_POS_OFFSET", 56: "POLL_ROTOR_POS", 57: "NOTIFY_BOOT", 58: "STATUS_6",
                    59: "GNSS_TIME", 60: "GNSS_LAT", 61: "GNSS_LON", 62: "GNSS_ALT_SPEED_HDOP", 63: "UPDATE_BAUD"
                }
                comment = packet_names.get(packet_type, f"PACKET_{packet_type}")
        
        log_line = f"{timestamp};0;{msg_type};{can_id_hex};{length};{data_hex};{comment}\n"
        self.log_file.write(log_line)
        self.log_file.flush()
    
    def _handle_response(self, controller_id: int, command: int, data: bytes, length: int):
        """Handle VESC response"""
        self.processing_response = True
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
                command_name = get_command_name(command)
                print(f"[CAN RX] Controller: {controller_id}, Command: {command_name}({command}), Data: {data_bytes.hex()}")
            
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
                    if not self.connected:
                        self.connected = True
                        print(f"VESC {self.latest_fw_version['major']}.{self.latest_fw_version['minor']} {self.latest_fw_version['hw_name']} connected\n")
            
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
                self.blocking_response_received.set()
                self.blocking_response_data = data_array
                self.blocking_response_command = command

                motor_rl = VescMotorRLResponse()
                if vesc_lib.vesc_parse_motor_rl_response(data_array, length, ctypes.byref(motor_rl)):
                    self.latest_motor_rl = {
                        'resistance': motor_rl.resistance,
                        'inductance': motor_rl.inductance,
                        'ld_lq_diff': motor_rl.ld_lq_diff
                    }
                else:
                    print("Something went wrong when trying to parse motor r/l response")
            
            elif command == 27:  # COMM_DETECT_MOTOR_FLUX_LINKAGE or STATUS_5
                # Distinguish between STATUS_5 and DETECT_MOTOR_FLUX_LINKAGE based on message characteristics
                # STATUS_5: 8 bytes, starts parsing from index 0 (no command byte)
                # FLUX_LINKAGE: 4+ bytes, starts parsing from index 1 (has command byte)
                
                if length == 8:  # STATUS_5
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
                
                else:  # DETECT_MOTOR_FLUX_LINKAGE
                    self.blocking_response_received.set()
                    self.blocking_response_data = data_array
                    self.blocking_response_command = command

                    flux_linkage = VescFluxLinkageResponse()
                    if vesc_lib.vesc_parse_flux_linkage_response(data_array, length, ctypes.byref(flux_linkage)):
                        self.latest_flux_linkage = {
                            'flux_linkage': flux_linkage.flux_linkage
                        }
            
            elif command == 18:  # CAN_PACKET_PONG
                pong = VescPongResponse()
                if vesc_lib.vesc_parse_pong_response(data_array, length, ctypes.byref(pong)):
                    self.latest_pong = {
                        'controller_id': pong.controller_id,
                        'valid': pong.valid
                    }
                    if self.verbose:
                        print(f"[PONG] VESC#{pong.controller_id} responded to ping")
            
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
                    command_name = get_command_name(command)
                    print(f"[{command_name}] Controller: {controller_id}, "
                          f"Command: {command}, Data: {data_bytes.hex()}")
            
            self._log_can_message(controller_id, data_array, length, is_tx=False)
            
        except Exception as e:
            print(f"Error in _handle_response: {e}")
        finally:
            self.processing_response = False
    
    def _monitor_loop(self):
        """Main monitoring loop"""

        print("Connecting to VESC...")
        vesc_lib.vesc_get_fw_version(self.vesc_id)

        while self.running:
            try:
                msg = self.can_obj.recv(timeout=0.001)
                if msg:
                    can_id = msg.arbitration_id
                    data = msg.data
                    length = len(data)
                    
                    # Only log raw CAN messages if verbose is enabled and we're not processing a VESC response
                    # This prevents duplicate output since _handle_response will print the parsed message
                    if self.verbose and not self.processing_response:
                        print(f"[CAN RX] ID: 0x{can_id:08X}, Data: {data.hex()}")
                    
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
                break
    
    def _wait_for_response(self, timeout: float = 3.0, command_to_wait_for: int = None, is_blocking: bool = False) -> bool:
        """Wait for response with timeout"""
        if is_blocking:
            self.blocking_response_received.clear()
            self.blocking_response_data = None
            self.blocking_response_command = None
        else:
            self.response_received.clear()
            self.response_data = None
            self.response_command = None
        
        if is_blocking:
            if self.blocking_response_received.wait(timeout) and (command_to_wait_for is None or self.blocking_response_command == command_to_wait_for):
                return True
            else:
                print(f"Timeout waiting for response (after {timeout}s)")
                return False
        else:
            if self.response_received.wait(timeout) and (command_to_wait_for is None or self.response_command == command_to_wait_for):
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
            print("  setcurrent      - Set motor current for specified duration")
            print("  setduty         - Set motor duty cycle for specified duration")
            print("  adc             - Get ADC values")
            print("  ppm             - Get PPM values")
            print("  chuck           - Get decoded chuck (nunchuk) data")
            print("  setchuck        - Set chuck (nunchuk) data")
            print("  motor_rl        - Detect motor resistance and inductance")
            print("  flux_linkage    - Detect motor flux linkage [current] [min_rpm] [duty]")
            print("  fw_version      - Get firmware version")
            print("  reboot          - Reboot the VESC controller")
            print("  ping            - Send PING to VESC and wait for PONG response")
            print("  raw_can         - Send raw CAN message")
            print("  listen          - Listen for VESC status messages (1-6, with optional filtering)")
            print("  logging         - Toggle CAN message logging to file")
            print("  debug           - Enable/disable SDK debugging")
            print("  debug_stats     - Show SDK debug statistics")
            print("  debug_reset     - Reset SDK debug statistics")
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
        
        if self._wait_for_response(3.0, command_to_wait_for=4):
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
    
    def help_setcurrent(self):
        """Help for setcurrent command"""
        print("setcurrent - Set motor current for specified duration")
        print("  Sets the motor current and repeats the command for the specified duration.")
        print("  WARNING: This command will turn the motor!")
        print("  Usage: setcurrent <current> [duration]")
        print("  Parameters:")
        print("    current  - Motor current in Amperes (positive or negative)")
        print("    duration - Duration in seconds (default: 1.0)")
        print("  Examples:")
        print("    setcurrent 5.0              - Set 5A current for 1 second")
        print("    setcurrent -3.0 2.5         - Set -3A current for 2.5 seconds")
        print("    setcurrent 0.0              - Stop motor (0A current)")
        print("  Note: Positive current turns motor forward, negative turns backward.")
        print("  The command is repeated every 200ms for the specified duration.")
    
    def do_setcurrent(self, arg):
        """Set motor current for specified duration"""
        if not arg:
            print("Error: setcurrent requires at least current argument")
            print("Usage: setcurrent <current> [duration]")
            return
        
        args = shlex.split(arg)
        if len(args) < 1:
            print("Error: setcurrent requires at least current argument")
            print("Usage: setcurrent <current> [duration]")
            return
        
        if len(args) > 2:
            print("Error: Too many arguments")
            print("Usage: setcurrent <current> [duration]")
            return
        
        # Parse current
        try:
            current = float(args[0])
        except ValueError:
            print("Error: Current must be a valid number")
            return
        
        # Parse duration (default to 1.0 second)
        duration = 1.0
        if len(args) > 1:
            try:
                duration = float(args[1])
                if duration <= 0:
                    print("Error: Duration must be positive")
                    return
            except ValueError:
                print("Error: Duration must be a valid number")
                return
        
        # Warn user about motor movement
        if current != 0.0:
            print("WARNING: This command will turn the motor!")
            print("Make sure the motor is free to rotate and not connected to any load.")
            print(f"Current: {current:.2f}A, Duration: {duration:.1f}s")
            
            # Ask for confirmation
            try:
                confirm = input("Do you want to continue? (yes/no): ").strip().lower()
                if confirm not in ['yes', 'y']:
                    print("Current setting cancelled.")
                    return
            except KeyboardInterrupt:
                print("\nCurrent setting cancelled.")
                return
        
        print(f"Setting motor current to {current:.2f}A for {duration:.1f} seconds...")
        
        # Calculate number of commands to send (every 200ms)
        command_interval = 0.2  # 200ms
        num_commands = int(duration / command_interval) + 1  # +1 to ensure we cover the full duration
        
        start_time = time.time()
        
        try:
            for i in range(num_commands):
                # Send current command
                vesc_lib.vesc_set_current(self.vesc_id, current)
                
                # Calculate remaining time
                elapsed = time.time() - start_time
                remaining = duration - elapsed
                
                if remaining <= 0:
                    break
                
                # Sleep for command interval (but don't sleep on the last iteration)
                if i < num_commands - 1:
                    time.sleep(command_interval)
            
            print(f"Current command completed. Total time: {time.time() - start_time:.1f}s")
            
        except KeyboardInterrupt:
            print("\nCurrent command interrupted by user")
            # Send 0 current to stop the motor
            vesc_lib.vesc_set_current(self.vesc_id, 0.0)
            print("Motor stopped (0A current sent)")
    
    def help_setduty(self):
        """Help for setduty command"""
        print("setduty - Set motor duty cycle for specified duration")
        print("  Sets the motor duty cycle and repeats the command for the specified duration.")
        print("  WARNING: This command will turn the motor!")
        print("  Usage: setduty <duty_cycle> [duration]")
        print("  Parameters:")
        print("    duty_cycle - Motor duty cycle as a percentage (0-100%)")
        print("    duration - Duration in seconds (default: 1.0)")
        print("  Examples:")
        print("    setduty 50.0              - Set 50% duty cycle for 1 second")
        print("    setduty 75.0 2.5         - Set 75% duty cycle for 2.5 seconds")
        print("    setduty 0.0              - Stop motor (0% duty cycle)")
        print("  Note: Positive duty cycle turns motor forward, negative turns backward.")
        print("  The command is repeated every 200ms for the specified duration.")
    
    def do_setduty(self, arg):
        """Set motor duty cycle for specified duration"""
        if not arg:
            print("Error: setduty requires at least duty_cycle argument")
            print("Usage: setduty <duty_cycle> [duration]")
            return
        
        args = shlex.split(arg)
        if len(args) < 1:
            print("Error: setduty requires at least duty_cycle argument")
            print("Usage: setduty <duty_cycle> [duration]")
            return
        
        if len(args) > 2:
            print("Error: Too many arguments")
            print("Usage: setduty <duty_cycle> [duration]")
            return
        
        # Parse duty cycle
        try:
            duty_cycle = float(args[0])
            if duty_cycle < 0 or duty_cycle > 100:
                print("Error: Duty cycle must be between 0 and 100%")
                return
        except ValueError:
            print("Error: Duty cycle must be a valid number")
            return
        
        # Parse duration (default to 1.0 second)
        duration = 1.0
        if len(args) > 1:
            try:
                duration = float(args[1])
                if duration <= 0:
                    print("Error: Duration must be positive")
                    return
            except ValueError:
                print("Error: Duration must be a valid number")
                return
        
        # Warn user about motor movement
        if duty_cycle != 0.0:
            print("WARNING: This command will turn the motor!")
            print("Make sure the motor is free to rotate and not connected to any load.")
            print(f"Duty Cycle: {duty_cycle:.1f}%, Duration: {duration:.1f}s")
            
            # Ask for confirmation
            try:
                confirm = input("Do you want to continue? (yes/no): ").strip().lower()
                if confirm not in ['yes', 'y']:
                    print("Duty cycle setting cancelled.")
                    return
            except KeyboardInterrupt:
                print("\nDuty cycle setting cancelled.")
                return
        
        print(f"Setting motor duty cycle to {duty_cycle:.1f}% for {duration:.1f} seconds...")
        
        # Calculate number of commands to send (every 200ms)
        command_interval = 0.2  # 200ms
        num_commands = int(duration / command_interval) + 1  # +1 to ensure we cover the full duration
        
        start_time = time.time()
        
        try:
            for i in range(num_commands):
                # Send duty cycle command
                vesc_lib.vesc_set_duty(self.vesc_id, duty_cycle)
                
                # Calculate remaining time
                elapsed = time.time() - start_time
                remaining = duration - elapsed
                
                if remaining <= 0:
                    break
                
                # Sleep for command interval (but don't sleep on the last iteration)
                if i < num_commands - 1:
                    time.sleep(command_interval)
            
            print(f"Duty cycle command completed. Total time: {time.time() - start_time:.1f}s")
            
        except KeyboardInterrupt:
            print("\nDuty cycle command interrupted by user")
            # Send 0 duty cycle to stop the motor
            vesc_lib.vesc_set_duty(self.vesc_id, 0.0)
            print("Motor stopped (0% duty cycle sent)")
    
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
        
        if self._wait_for_response(20.0, command_to_wait_for=25, is_blocking=True):
            if self.latest_motor_rl:
                print(f"\nMotor R/L Detection Results:")
                print(f"  Resistance: {self.latest_motor_rl['resistance']:.6f}Ω")
                print(f"  Inductance: {self.latest_motor_rl['inductance']:.8f}µH")
                print(f"  Ld-Lq Difference: {self.latest_motor_rl['ld_lq_diff']:.8f}µH")
            else:
                print("Did not receive motor values yet")
        else:
            print("Failed to get motor R/L values")
    
    def help_flux_linkage(self):
        """Help for flux_linkage command"""
        print("flux_linkage - Detect motor flux linkage")
        print("  Performs motor flux linkage detection (takes up to 20 seconds).")
        print("  WARNING: This command will turn the motor!")
        print("  Requires motor R/L values to be available (run 'motor_rl' first).")
        print("  Waits up to 20 seconds for response.")
        print("  Usage: flux_linkage [current] [min_rpm] [duty]")
        print("  Parameters:")
        print("    current  - Detection current in Amperes (default: 5.0A, range: 1-20A)")
        print("    min_rpm  - Minimum RPM for detection (default: 1000, range: 100-5000)")
        print("    duty     - Duty cycle for detection (default: 0.1 = 10%, range: 0.05-0.3)")
        print("  Examples:")
        print("    flux_linkage                    - Use default values")
        print("    flux_linkage 3.0                - 3A current, default RPM/duty")
        print("    flux_linkage 3.0 500            - 3A current, 500 RPM, default duty")
        print("    flux_linkage 3.0 500 0.05       - 3A current, 500 RPM, 5% duty")
        print("  Note: Lower current and RPM values are safer but may take longer.")
    
    def do_flux_linkage(self, arg):
        """Detect motor flux linkage"""
        # Parse arguments
        args = shlex.split(arg) if arg else []

        if args[0] == "debug":
            print("Debug command with 10A, 2000RPM, 30% duty and 0.026479Ω resistance")
            self.do_debug("verbose all")
            vesc_lib.vesc_detect_motor_flux_linkage(self.vesc_id, 10.0, 2000, 0.3, 0.026479)
            self.do_debug("none")
            return
        
        # Set default values
        current = 5.0  # 5A detection current
        min_rpm = 1000.0  # 1000 RPM minimum
        duty = 0.1  # 10% duty cycle
        
        # Parse provided arguments
        if len(args) > 0:
            try:
                current = float(args[0])
                if current <= 0:
                    print("Error: Current must be positive")
                    return
                if current > 20:
                    print("Warning: Current > 20A may be dangerous. Continue anyway? (yes/no): ", end="")
                    try:
                        if input().strip().lower() not in ['yes', 'y']:
                            print("Flux linkage detection cancelled.")
                            return
                    except KeyboardInterrupt:
                        print("\nFlux linkage detection cancelled.")
                        return
            except ValueError:
                print("Error: Current must be a valid number")
                return
        
        if len(args) > 1:
            try:
                min_rpm = float(args[1])
                if min_rpm <= 0:
                    print("Error: Min RPM must be positive")
                    return
                if min_rpm > 5000:
                    print("Warning: Min RPM > 5000 may be dangerous. Continue anyway? (yes/no): ", end="")
                    try:
                        if input().strip().lower() not in ['yes', 'y']:
                            print("Flux linkage detection cancelled.")
                            return
                    except KeyboardInterrupt:
                        print("\nFlux linkage detection cancelled.")
                        return
            except ValueError:
                print("Error: Min RPM must be a valid number")
                return
        
        if len(args) > 2:
            try:
                duty = float(args[2])
                if duty <= 0 or duty > 1:
                    print("Error: Duty cycle must be between 0 and 1")
                    return
                if duty > 0.3:
                    print("Warning: Duty cycle > 30% may be dangerous. Continue anyway? (yes/no): ", end="")
                    try:
                        if input().strip().lower() not in ['yes', 'y']:
                            print("Flux linkage detection cancelled.")
                            return
                    except KeyboardInterrupt:
                        print("\nFlux linkage detection cancelled.")
                        return
            except ValueError:
                print("Error: Duty cycle must be a valid number")
                return
        
        if len(args) > 3:
            print("Error: Too many arguments")
            print("Usage: flux_linkage [current] [min_rpm] [duty]")
            return
        
        # Check if motor R/L values are available
        if not self.latest_motor_rl:
            print("Error: Motor R/L values not available.")
            print("Please run 'motor_rl' command first to detect motor resistance and inductance.")
            return
        
        # Warn user about motor movement
        print("WARNING: This command will turn the motor!")
        print("Make sure the motor is free to rotate and not connected to any load.")
        print("The motor will be driven with current to measure flux linkage.")
        
        # Show parameters that will be used
        print(f"\nParameters to be used:")
        print(f"  Current: {current:.1f}A")
        print(f"  Min RPM: {min_rpm:.0f}")
        print(f"  Duty Cycle: {duty*100:.1f}%")
        print(f"  Motor Resistance: {self.latest_motor_rl['resistance']:.6f}Ω")
        
        # Ask for confirmation
        try:
            confirm = input("\nDo you want to continue? (yes/no): ").strip().lower()
            if confirm not in ['yes', 'y']:
                print("Flux linkage detection cancelled.")
                return
        except KeyboardInterrupt:
            print("\nFlux linkage detection cancelled.")
            return
        
        print("Detecting motor flux linkage (this may take up to 20 seconds)...")
        
        # Call the flux linkage detection function with user-specified parameters
        resistance = self.latest_motor_rl['resistance']
        vesc_lib.vesc_detect_motor_flux_linkage(self.vesc_id, current, min_rpm, duty, resistance)
        
        if self._wait_for_response(12.0, command_to_wait_for=26, is_blocking=True):
            if self.latest_flux_linkage:
                print(f"\nMotor Flux Linkage Detection Results:")
                print(f"  Flux Linkage: {self.latest_flux_linkage['flux_linkage']:.8f} Wb")
                print(f"  Parameters used:")
                print(f"    Current: {current:.1f}A")
                print(f"    Min RPM: {min_rpm:.0f}")
                print(f"    Duty Cycle: {duty*100:.1f}%")
                print(f"    Motor Resistance: {resistance:.6f}Ω")
            else:
                print("Failed to parse flux linkage values")
        else:
            print("Failed to get flux linkage values")
    
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
    
    def help_ping(self):
        """Help for ping command"""
        print("ping - Send PING to VESC and wait for PONG response")
        print("  Sends a PING packet to the VESC controller and waits for a PONG response.")
        print("  This is useful to check if the VESC is present and responding on the CAN bus.")
        print("  Waits up to 3 seconds for response.")
        print("  Usage: ping [controller_id]")
        print("  Example: ping 1    - Ping VESC with ID 1")
        print("  Example: ping      - Ping VESC with default ID")
    
    def do_ping(self, arg):
        """Send PING to VESC and wait for PONG response"""
        # Parse controller ID from argument, use default if not provided
        controller_id = self.vesc_id
        if arg:
            try:
                controller_id = int(arg)
                if controller_id < 0 or controller_id > 255:
                    print("Error: controller ID must be between 0 and 255")
                    return
            except ValueError:
                print("Error: controller ID must be a number")
                return
        
        print(f"Sending PING to VESC#{controller_id}...")
        
        # Clear any previous PONG response
        self.latest_pong = None
        
        # Send PING
        vesc_lib.vesc_ping(controller_id)
        
        # Wait for PONG response
        if self._wait_for_response(3.0):
            if self.latest_pong and self.latest_pong['valid']:
                print(f"✓ PONG received from VESC#{self.latest_pong['controller_id']}")
            else:
                print("✗ PONG response received but failed to parse")
        else:
            print("✗ No PONG response received (timeout after 3 seconds)")
            print(f"  VESC#{controller_id} may not be present or not responding on the CAN bus")
    
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
                status_cmd_id = map_status_number_to_command_id(cmd_arg)
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
            cmd_names = [f"{cmd_id}({get_command_name(cmd_id)})" for cmd_id in sorted(self.listen_command_filter)]
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
                print(f"CAN logging disabled, find your log file at: {self.log_file.name}")
                self.log_file.close()
                self.log_file = None
            self.logging_enabled = False
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
                self.log_file.write("Timestamp;Lost;Type;ID;Length;Data;Comment\n")
                
                self.logging_enabled = True
                print(f"CAN logging enabled: {log_filename}")
                
            except Exception as e:
                print(f"Error enabling logging: {e}")
    
    def help_debug(self):
        """Help for debug command"""
        print("debug - Enable/disable SDK debugging")
        print("  Enables or disables debugging output from the VESC CAN SDK.")
        print("  Usage: debug [level] [categories]")
        print("  Parameters:")
        print("    level     - Debug level: none(0), basic(1), detailed(2), verbose(3)")
        print("    categories - Debug categories (space-separated):")
        print("                 can, commands, responses, buffers, errors, performance, all")
        print("  Examples:")
        print("    debug                    - Show current debug status")
        print("    debug basic can          - Enable basic CAN debugging")
        print("    debug detailed all       - Enable detailed debugging for all categories")
        print("    debug verbose commands   - Enable verbose command debugging")
        print("    debug none               - Disable all debugging")
        print("  Debug Levels:")
        print("    none(0)     - No debug output")
        print("    basic(1)    - Basic debug information")
        print("    detailed(2) - Detailed debug information with hex dumps")
        print("    verbose(3)  - Very detailed debug information")
        print("  Debug Categories:")
        print("    can         - CAN communication (TX/RX)")
        print("    commands    - Command sending")
        print("    responses   - Response parsing")
        print("    buffers     - Buffer management")
        print("    errors      - Error conditions")
        print("    performance - Performance metrics")
        print("    all         - All categories")
    
    def do_debug(self, arg):
        """Enable/disable SDK debugging"""
        if not arg:
            # Show current debug status
            config = VescDebugConfig()
            if vesc_lib.vesc_debug_get_config(ctypes.byref(config)):
                level_names = {VESC_DEBUG_NONE: "none", VESC_DEBUG_BASIC: "basic", 
                              VESC_DEBUG_DETAILED: "detailed", VESC_DEBUG_VERBOSE: "verbose"}
                level_name = level_names.get(config.level, f"unknown({config.level})")
                
                category_names = []
                if config.categories & VESC_DEBUG_CAN:
                    category_names.append("can")
                if config.categories & VESC_DEBUG_COMMANDS:
                    category_names.append("commands")
                if config.categories & VESC_DEBUG_RESPONSES:
                    category_names.append("responses")
                if config.categories & VESC_DEBUG_BUFFERS:
                    category_names.append("buffers")
                if config.categories & VESC_DEBUG_ERRORS:
                    category_names.append("errors")
                if config.categories & VESC_DEBUG_PERFORMANCE:
                    category_names.append("performance")
                
                if config.level == VESC_DEBUG_NONE:
                    print("Debug status: Disabled")
                else:
                    print(f"Debug status: Enabled (level={level_name}, categories={', '.join(category_names)})")
            else:
                print("Debug status: Unknown (failed to get configuration)")
            print("Use 'debug <level> <categories>' to change debugging")
            return
        
        args = shlex.split(arg)
        
        # Parse debug level
        level = VESC_DEBUG_BASIC  # Default to basic
        if args[0].lower() in ['none', '0']:
            level = VESC_DEBUG_NONE
        elif args[0].lower() in ['basic', '1']:
            level = VESC_DEBUG_BASIC
        elif args[0].lower() in ['detailed', '2']:
            level = VESC_DEBUG_DETAILED
        elif args[0].lower() in ['verbose', '3']:
            level = VESC_DEBUG_VERBOSE
        else:
            print(f"Error: Invalid debug level '{args[0]}'")
            print("Valid levels: none(0), basic(1), detailed(2), verbose(3)")
            return
        
        # Parse debug categories
        categories = 0
        if len(args) > 1:
            for category_arg in args[1:]:
                category_arg = category_arg.lower()
                if category_arg == 'can':
                    categories |= VESC_DEBUG_CAN
                elif category_arg == 'commands':
                    categories |= VESC_DEBUG_COMMANDS
                elif category_arg == 'responses':
                    categories |= VESC_DEBUG_RESPONSES
                elif category_arg == 'buffers':
                    categories |= VESC_DEBUG_BUFFERS
                elif category_arg == 'errors':
                    categories |= VESC_DEBUG_ERRORS
                elif category_arg == 'performance':
                    categories |= VESC_DEBUG_PERFORMANCE
                elif category_arg == 'all':
                    categories = VESC_DEBUG_ALL
                else:
                    print(f"Error: Invalid debug category '{category_arg}'")
                    print("Valid categories: can, commands, responses, buffers, errors, performance, all")
                    return
        else:
            # Default to all categories if none specified
            categories = VESC_DEBUG_ALL
        
        # Enable or disable debugging
        if level == VESC_DEBUG_NONE:
            vesc_lib.vesc_debug_disable()
            print("Debugging disabled")
        else:
            if vesc_lib.vesc_debug_enable(level, categories):
                level_names = {VESC_DEBUG_NONE: "none", VESC_DEBUG_BASIC: "basic", 
                              VESC_DEBUG_DETAILED: "detailed", VESC_DEBUG_VERBOSE: "verbose"}
                category_names = []
                if categories & VESC_DEBUG_CAN:
                    category_names.append("can")
                if categories & VESC_DEBUG_COMMANDS:
                    category_names.append("commands")
                if categories & VESC_DEBUG_RESPONSES:
                    category_names.append("responses")
                if categories & VESC_DEBUG_BUFFERS:
                    category_names.append("buffers")
                if categories & VESC_DEBUG_ERRORS:
                    category_names.append("errors")
                if categories & VESC_DEBUG_PERFORMANCE:
                    category_names.append("performance")
                
                print(f"Debugging enabled: level={level_names[level]}, categories={', '.join(category_names)}")
            else:
                print("Error: Failed to enable debugging")
    
    def help_debug_stats(self):
        """Help for debug_stats command"""
        print("debug_stats - Show SDK debug statistics")
        print("  Displays statistics collected by the VESC CAN SDK debug system.")
        print("  Usage: debug_stats")
        print("  Statistics include:")
        print("    - CAN transmit/receive counts")
        print("    - Commands sent and responses received")
        print("    - Error counts (CRC errors, buffer overflows)")
        print("    - Total bytes transmitted/received")
    
    def do_debug_stats(self, arg):
        """Show SDK debug statistics"""
        if arg:
            print("Error: debug_stats command takes no arguments")
            return
        
        # Get debug statistics
        stats = VescDebugStats()
        if vesc_lib.vesc_debug_get_stats(ctypes.byref(stats)):
            print("\nVESC CAN SDK Debug Statistics:")
            print(f"  CAN Transmit Count: {stats.can_tx_count}")
            print(f"  CAN Receive Count: {stats.can_rx_count}")
            print(f"  Commands Sent: {stats.command_count}")
            print(f"  Responses Received: {stats.response_count}")
            print(f"  Total Errors: {stats.error_count}")
            print(f"  CRC Errors: {stats.crc_error_count}")
            print(f"  Buffer Overflows: {stats.buffer_overflow_count}")
            print(f"  Total TX Bytes: {stats.total_tx_bytes}")
            print(f"  Total RX Bytes: {stats.total_rx_bytes}")
        else:
            print("Error: Failed to get debug statistics")
    
    def help_debug_reset(self):
        """Help for debug_reset command"""
        print("debug_reset - Reset SDK debug statistics")
        print("  Resets all debug statistics counters to zero.")
        print("  Usage: debug_reset")
    
    def do_debug_reset(self, arg):
        """Reset SDK debug statistics"""
        if arg:
            print("Error: debug_reset command takes no arguments")
            return
        
        vesc_lib.vesc_debug_reset_stats()
        print("Debug statistics reset")
    

    
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
        # wait until VESC is connected
        while not shell.connected:
            time.sleep(0.1)
        shell.cmdloop()
    except KeyboardInterrupt:
        print("\nShell stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 