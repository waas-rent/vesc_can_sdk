#!/usr/bin/env python3
"""
VESC CAN SDK Python Bindings

This module provides Python bindings for the VESC CAN SDK library.
It includes C types, constants, DLL loading, and function mappings.

Copyright (c) 2025 waas AG (waas.rent)
"""

import sys
import ctypes
from ctypes import cdll, c_bool, c_uint8, c_uint32, c_float, c_int32, c_char, c_int16, c_uint16, c_uint64

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

# Debug constants
VESC_DEBUG_NONE = 0
VESC_DEBUG_BASIC = 1
VESC_DEBUG_DETAILED = 2
VESC_DEBUG_VERBOSE = 3

VESC_DEBUG_CAN = 0x0001
VESC_DEBUG_COMMANDS = 0x0002
VESC_DEBUG_RESPONSES = 0x0004
VESC_DEBUG_BUFFERS = 0x0008
VESC_DEBUG_ERRORS = 0x0010
VESC_DEBUG_PERFORMANCE = 0x0020
VESC_DEBUG_ALL = 0x003F

# Command ID constants
COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_GET_STATUS_1 = 9
COMM_GET_STATUS_2 = 14
COMM_GET_STATUS_3 = 15
COMM_GET_STATUS_4 = 16
COMM_PING = 17
COMM_PONG = 18
COMM_DETECT_MOTOR_R_L = 25
COMM_DETECT_MOTOR_FLUX_LINKAGE = 27
COMM_GET_STATUS_5 = 27  # Same as FLUX_LINKAGE
COMM_REBOOT = 29
COMM_GET_DECODED_ADC = 30
COMM_GET_DECODED_PPM = 31
COMM_GET_DECODED_CHUK = 33
COMM_GET_STATUS_6 = 58

# C Structure Definitions
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

class VescChuckValues(ctypes.Structure):
    _fields_ = [
        ("js_y", c_float),
        ("valid", c_bool)
    ]

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

class VescMotorParamResponse(ctypes.Structure):
    _fields_ = [
        ("current", c_float),
        ("min_rpm", c_float),
        ("low_duty", c_float),
        ("valid", c_bool)
    ]

class VescFluxLinkageResponse(ctypes.Structure):
    _fields_ = [
        ("flux_linkage", c_float),
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

class VescPongResponse(ctypes.Structure):
    _fields_ = [
        ("controller_id", c_uint8),
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
        ("can_tx_count", c_uint32),
        ("can_rx_count", c_uint32),
        ("command_count", c_uint32),
        ("response_count", c_uint32),
        ("error_count", c_uint32),
        ("crc_error_count", c_uint32),
        ("buffer_overflow_count", c_uint32),
        ("total_tx_bytes", c_uint64),
        ("total_rx_bytes", c_uint64),
        ("valid", c_bool)
    ]

# Function signature definitions
def setup_function_signatures():
    """Setup all function signatures for the VESC CAN SDK library"""
    
    # Core initialization functions
    vesc_lib.vesc_can_init.argtypes = [ctypes.c_void_p, c_uint8, c_uint8]
    vesc_lib.vesc_can_init.restype = c_bool
    
    vesc_lib.vesc_set_response_callback.argtypes = [ctypes.c_void_p]
    
    vesc_lib.vesc_process_can_frame.argtypes = [c_uint32, ctypes.POINTER(c_uint8), c_uint8]
    
    # Command functions
    vesc_lib.vesc_get_values.argtypes = [c_uint8]
    vesc_lib.vesc_get_decoded_adc.argtypes = [c_uint8]
    vesc_lib.vesc_get_decoded_ppm.argtypes = [c_uint8]
    vesc_lib.vesc_get_decoded_chuck.argtypes = [c_uint8]
    vesc_lib.vesc_get_fw_version.argtypes = [c_uint8]
    vesc_lib.vesc_reboot.argtypes = [c_uint8]
    vesc_lib.vesc_detect_motor_r_l.argtypes = [c_uint8]
    vesc_lib.vesc_ping.argtypes = [c_uint8]
    
    # Motor control functions
    vesc_lib.vesc_set_current.argtypes = [c_uint8, c_float]
    vesc_lib.vesc_set_current.restype = None
    vesc_lib.vesc_set_duty.argtypes = [c_uint8, c_float]
    vesc_lib.vesc_set_duty.restype = None
    
    # Chuck data function
    vesc_lib.vesc_set_chuck_data.argtypes = [c_uint8, ctypes.c_void_p]
    
    # Flux linkage detection
    vesc_lib.vesc_detect_motor_flux_linkage.argtypes = [c_uint8, c_float, c_float, c_float, c_float]
    vesc_lib.vesc_detect_motor_flux_linkage.restype = None
    
    # Parsing functions
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
    
    vesc_lib.vesc_parse_chuck_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescChuckValues)]
    vesc_lib.vesc_parse_chuck_values.restype = c_bool
    
    vesc_lib.vesc_parse_motor_param_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescMotorParamResponse)]
    vesc_lib.vesc_parse_motor_param_response.restype = c_bool
    
    vesc_lib.vesc_parse_flux_linkage_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescFluxLinkageResponse)]
    vesc_lib.vesc_parse_flux_linkage_response.restype = c_bool
    
    # Status message parsing functions
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
    
    vesc_lib.vesc_parse_pong_response.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescPongResponse)]
    vesc_lib.vesc_parse_pong_response.restype = c_bool
    
    # Debug functions
    vesc_lib.vesc_debug_configure.argtypes = [ctypes.POINTER(VescDebugConfig)]
    vesc_lib.vesc_debug_configure.restype = c_bool
    
    vesc_lib.vesc_debug_enable.argtypes = [c_uint8, c_uint16]
    vesc_lib.vesc_debug_enable.restype = c_bool
    
    vesc_lib.vesc_debug_disable.argtypes = []
    
    vesc_lib.vesc_debug_get_stats.argtypes = [ctypes.POINTER(VescDebugStats)]
    vesc_lib.vesc_debug_get_stats.restype = c_bool
    
    vesc_lib.vesc_debug_get_config.argtypes = [ctypes.POINTER(VescDebugConfig)]
    vesc_lib.vesc_debug_get_config.restype = c_bool
    
    vesc_lib.vesc_debug_reset_stats.argtypes = []
    vesc_lib.vesc_debug_print_stats.argtypes = []
    vesc_lib.vesc_debug_print_buffer_state.argtypes = []

# Command name mapping
def get_command_name(command_id: int) -> str:
    """Get human-readable name for a command ID"""
    command_names = {
        0: "FW_VERSION",
        4: "GET_VALUES", 
        9: "STATUS_1",
        14: "STATUS_2",
        15: "STATUS_3",
        16: "STATUS_4",
        17: "PING",
        18: "PONG",
        25: "DETECT_MOTOR_R_L",
        27: "STATUS_5/FLUX_LINKAGE",  # Conflict: both STATUS_5 and DETECT_MOTOR_FLUX_LINKAGE use ID 27
        29: "REBOOT",
        30: "GET_DECODED_ADC",
        31: "GET_DECODED_PPM",
        33: "GET_DECODED_CHUK",
        58: "STATUS_6"
    }
    return command_names.get(command_id, f"UNKNOWN_{command_id}")

def map_status_number_to_command_id(status_num: str) -> int:
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

# Initialize function signatures when module is imported
setup_function_signatures() 

# Export ctypes types for convenience
c_bool = ctypes.c_bool
c_uint8 = ctypes.c_uint8
c_uint32 = ctypes.c_uint32
c_float = ctypes.c_float
c_int32 = ctypes.c_int32
c_char = ctypes.c_char
c_int16 = ctypes.c_int16
c_uint16 = ctypes.c_uint16
c_uint64 = ctypes.c_uint64 