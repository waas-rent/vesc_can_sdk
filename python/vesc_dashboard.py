#!/usr/bin/env python3
"""
VESC CAN SDK - Terminal Dashboard

This script provides a real-time dashboard for monitoring VESC motor
controller values over CAN.

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
from typing import Dict, Any, Optional, List, Deque
from collections import deque
import ctypes
from ctypes import cdll, c_bool, c_uint8, c_uint32, c_float, c_int32, c_char, c_int16, c_uint16, c_uint64
from datetime import datetime
import os
import faulthandler
import random
import math

# Enable fault handler
faulthandler.enable()

# Try to import required libraries
try:
    import can
except ImportError:
    print("Error: python-can library not found")
    print("Install it with: pip install python-can")
    sys.exit(1)

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.live import Live
    from rich.text import Text
    from rich.progress import Progress, SpinnerColumn, TextColumn
    from rich.align import Align
    from rich import box
    from rich.style import Style
except ImportError:
    print("Error: rich library not found")
    print("Install it with: pip install rich")
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

# Define C types for structures (same as vesc_shell.py)
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

# Define function signatures
vesc_lib.vesc_can_init.argtypes = [ctypes.c_void_p, c_uint8, c_uint8]
vesc_lib.vesc_can_init.restype = c_bool

vesc_lib.vesc_set_response_callback.argtypes = [ctypes.c_void_p]

vesc_lib.vesc_process_can_frame.argtypes = [c_uint32, ctypes.POINTER(c_uint8), c_uint8]

vesc_lib.vesc_get_values.argtypes = [c_uint8]

vesc_lib.vesc_parse_get_values.argtypes = [ctypes.POINTER(c_uint8), c_uint8, ctypes.POINTER(VescValues)]
vesc_lib.vesc_parse_get_values.restype = c_bool

class VescDashboard:
    def __init__(self, can_interface: str, vesc_id: int = 1, bustype: str = 'socketcan', 
                 baudrate: int = 500000, sender_id: int = 42, update_rate: float = 1.0, demo: bool = False):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.sender_id = sender_id
        self.bustype = bustype
        self.baudrate = baudrate
        self.update_rate = update_rate
        self.running = True
        self.demo = demo
        
        # Console and layout
        self.console = Console()
        self.layout = Layout()
        
        # Data storage
        self.latest_values = {}
        self.response_received = threading.Event()
        self.response_data = None
        self.response_command = None
        
        # Log messages
        self.log_messages: Deque[str] = deque(maxlen=100)
        self.log_lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'messages_received': 0,
            'last_update': None,
            'connection_status': 'Disconnected',
            'errors': 0
        }
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        if not self.demo:
            # Initialize CAN interface
            self._init_can()
            # Initialize VESC SDK
            self._init_vesc_sdk()
            # Start monitoring thread
            self.monitor_thread = threading.Thread(target=self._monitor_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            # Start update thread
            self.update_thread = threading.Thread(target=self._update_loop)
            self.update_thread.daemon = True
            self.update_thread.start()
        else:
            self.stats['connection_status'] = 'Demo Mode'
            self._add_log('Running in demo mode with dummy values.')
            self.demo_thread = threading.Thread(target=self._demo_loop)
            self.demo_thread.daemon = True
            self.demo_thread.start()
        
        # Set up layout
        self._setup_layout()
    
    def _signal_handler(self, signum, frame):
        print(f"\nReceived signal {signum}, shutting down...")
        self.running = False
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
            self.stats['connection_status'] = 'Connected'
            self._add_log(f"CAN interface initialized: {self.can_interface}")
        except Exception as e:
            self.stats['connection_status'] = 'Error'
            self._add_log(f"Error initializing CAN interface: {e}")
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
                    
                    self.can_obj.send(can_msg)
                    return True
                except Exception as e:
                    self._add_log(f"CAN send error: {e}")
                    self.stats['errors'] += 1
                    return False
            
            self.can_send_func = ctypes.CFUNCTYPE(c_bool, c_uint32, ctypes.POINTER(c_uint8), c_uint8)(can_send)
            
            if not vesc_lib.vesc_can_init(self.can_send_func, self.vesc_id, self.sender_id):
                raise RuntimeError("Failed to initialize VESC CAN SDK")
            
            # Set response callback
            def response_callback(controller_id: int, command: int, data_ptr, length: int):
                try:
                    self._handle_response(controller_id, command, data_ptr, length)
                except Exception as e:
                    self._add_log(f"Error in response callback: {e}")
                    self.stats['errors'] += 1
            
            self.response_callback = ctypes.CFUNCTYPE(None, c_uint8, c_uint8, ctypes.POINTER(c_uint8), c_uint8)(response_callback)
            vesc_lib.vesc_set_response_callback(self.response_callback)
            
            self._add_log(f"VESC SDK initialized for controller ID {self.vesc_id}")
            
        except Exception as e:
            self._add_log(f"ERROR during VESC SDK initialization: {e}")
            raise
    
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
                    self.stats['last_update'] = datetime.now()
                    self.stats['messages_received'] += 1
            
        except Exception as e:
            self._add_log(f"Error in _handle_response: {e}")
            self.stats['errors'] += 1
    
    def _monitor_loop(self):
        """Main monitoring loop"""
        while self.running:
            try:
                msg = self.can_obj.recv(timeout=0.001)
                if msg:
                    can_id = msg.arbitration_id
                    data = msg.data
                    length = len(data)
                    
                    try:
                        data_array = (c_uint8 * length)(*data)
                        vesc_lib.vesc_process_can_frame(can_id, data_array, length)
                    except Exception as e:
                        self._add_log(f"Error processing CAN frame: {e}")
                        self.stats['errors'] += 1
            
            except can.CanError:
                pass
            except Exception as e:
                self._add_log(f"Unexpected error in monitor loop: {e}")
                self.stats['errors'] += 1
                time.sleep(0.1)
    
    def _update_loop(self):
        """Update loop for requesting values"""
        while self.running:
            try:
                vesc_lib.vesc_get_values(self.vesc_id)
                time.sleep(self.update_rate)
            except Exception as e:
                self._add_log(f"Error in update loop: {e}")
                self.stats['errors'] += 1
                time.sleep(1.0)
    
    def _demo_loop(self):
        """Update loop for demo mode with animated dummy values"""
        t = 0.0
        while self.running:
            # Animate values with some plausible math
            self.latest_values = {
                'temp_fet': 40 + 10 * (1 + math.sin(t)) + random.uniform(-0.5, 0.5),
                'temp_motor': 38 + 8 * (1 + math.sin(t + 1)) + random.uniform(-0.5, 0.5),
                'temp_mos1': 39 + 7 * (1 + math.sin(t + 2)) + random.uniform(-0.5, 0.5),
                'temp_mos2': 39 + 7 * (1 + math.sin(t + 2.5)) + random.uniform(-0.5, 0.5),
                'temp_mos3': 39 + 7 * (1 + math.sin(t + 3)) + random.uniform(-0.5, 0.5),
                'current_motor': 20 + 10 * math.sin(t/2) + random.uniform(-1, 1),
                'current_in': 15 + 8 * math.sin(t/2 + 1) + random.uniform(-1, 1),
                'current_iq': 5 + 2 * math.sin(t/2 + 2) + random.uniform(-0.5, 0.5),
                'duty_cycle': 0.5 + 0.4 * math.sin(t/3),
                'rpm': 3000 + 1000 * math.sin(t/4),
                'tachometer': int(10000 + 1000 * math.sin(t/5)),
                'tachometer_abs': int(12000 + 1000 * math.sin(t/5 + 1)),
                'v_in': 50 + 2 * math.sin(t/6),
                'amp_hours': 5 + 0.1 * t,
                'amp_hours_charged': 2 + 0.05 * t,
                'watt_hours': 200 + 0.5 * t,
                'watt_hours_charged': 100 + 0.2 * t,
                'fault_code': 0,
                'pid_pos': 0.5 + 0.5 * math.sin(t/7),
                'controller_id': self.vesc_id,
                'vd': 1.0 + 0.2 * math.sin(t/8),
                'vq': 1.2 + 0.2 * math.sin(t/8 + 1),
                'status': 1
            }
            self.stats['last_update'] = datetime.now()
            self.stats['messages_received'] += 1
            if int(t) % 10 == 0:
                self._add_log(f"Demo: RPM={self.latest_values['rpm']:.0f} Current={self.latest_values['current_motor']:.1f}A")
            t += self.update_rate
            time.sleep(self.update_rate)
    
    def _add_log(self, message: str):
        """Add a log message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        with self.log_lock:
            self.log_messages.append(log_entry)
    
    def _setup_layout(self):
        """Set up the dashboard layout"""
        self.layout.split_column(
            Layout(name="header", size=3),
            Layout(name="main", ratio=1),
            Layout(name="footer", size=8)
        )
        
        self.layout["main"].split_row(
            Layout(name="left", ratio=1),
            Layout(name="right", ratio=1)
        )
    
    def _create_header(self) -> Panel:
        """Create the header panel"""
        title = Text("VESC CAN Dashboard", style="bold blue")
        subtitle = Text(f"Controller ID: {self.vesc_id} | Interface: {self.can_interface} | Update Rate: {self.update_rate}Hz", style="dim")
        
        status_color = "green" if self.stats['connection_status'] == 'Connected' else "red"
        status_text = Text(f"Status: {self.stats['connection_status']}", style=f"bold {status_color}")
        
        header_content = Align.center(
            title + "\n" + subtitle + "\n" + status_text
        )
        
        return Panel(header_content, title="VESC Motor Controller", border_style="blue")
    
    def _create_motor_values_panel(self) -> Panel:
        """Create the motor values panel"""
        table = Table(show_header=True, header_style="bold magenta", box=box.ROUNDED)
        table.add_column("Parameter", style="cyan", no_wrap=True)
        table.add_column("Value", style="white")
        table.add_column("Unit", style="dim")
        
        if self.latest_values:
            # Temperature section
            table.add_row("FET Temperature", f"{self.latest_values['temp_fet']:.1f}", "°C")
            table.add_row("Motor Temperature", f"{self.latest_values['temp_motor']:.1f}", "°C")
            table.add_row("MOS1 Temperature", f"{self.latest_values['temp_mos1']:.1f}", "°C")
            table.add_row("MOS2 Temperature", f"{self.latest_values['temp_mos2']:.1f}", "°C")
            table.add_row("MOS3 Temperature", f"{self.latest_values['temp_mos3']:.1f}", "°C")
            
            # Current section
            table.add_row("Motor Current", f"{self.latest_values['current_motor']:.2f}", "A")
            table.add_row("Input Current", f"{self.latest_values['current_in']:.2f}", "A")
            table.add_row("Duty Cycle", f"{self.latest_values['duty_cycle']*100:.1f}", "%")
            
            # Speed and position
            table.add_row("RPM", f"{self.latest_values['rpm']:.0f}", "")
            table.add_row("Tachometer", f"{self.latest_values['tachometer']}", "")
            table.add_row("Tachometer Abs", f"{self.latest_values['tachometer_abs']}", "")
            
            # Voltage
            table.add_row("Input Voltage", f"{self.latest_values['v_in']:.1f}", "V")
            
            # Energy
            table.add_row("Consumed Ah", f"{self.latest_values['amp_hours']:.2f}", "Ah")
            table.add_row("Consumed Wh", f"{self.latest_values['watt_hours']:.2f}", "Wh")
            
            # Status
            table.add_row("Fault Code", f"{self.latest_values['fault_code']}", "")
            table.add_row("Status", f"{self.latest_values['status']}", "")
        else:
            table.add_row("No data", "Waiting...", "")
        
        return Panel(table, title="Motor Values", border_style="green")
    
    def _create_advanced_values_panel(self) -> Panel:
        """Create the advanced values panel"""
        table = Table(show_header=True, header_style="bold magenta", box=box.ROUNDED)
        table.add_column("Parameter", style="cyan", no_wrap=True)
        table.add_column("Value", style="white")
        table.add_column("Unit", style="dim")
        
        if self.latest_values:
            # Advanced motor parameters
            table.add_row("Controller ID", f"{self.latest_values['controller_id']}", "")
            table.add_row("Current Iq", f"{self.latest_values['current_iq']:.3f}", "A")
            table.add_row("Vd", f"{self.latest_values['vd']:.3f}", "V")
            table.add_row("Vq", f"{self.latest_values['vq']:.3f}", "V")
            table.add_row("PID Position", f"{self.latest_values['pid_pos']:.3f}", "")
            
            # Charging values
            table.add_row("Ah Charged", f"{self.latest_values['amp_hours_charged']:.2f}", "Ah")
            table.add_row("Wh Charged", f"{self.latest_values['watt_hours_charged']:.2f}", "Wh")
            
            # Last update info
            if self.stats['last_update']:
                time_diff = (datetime.now() - self.stats['last_update']).total_seconds()
                table.add_row("Last Update", f"{time_diff:.1f}s ago", "")
            else:
                table.add_row("Last Update", "Never", "")
            
            table.add_row("Messages Rx", f"{self.stats['messages_received']}", "")
            table.add_row("Errors", f"{self.stats['errors']}", "")
        else:
            table.add_row("No data", "Waiting...", "")
        
        return Panel(table, title="Advanced Values", border_style="yellow")
    
    def _create_log_panel(self) -> Panel:
        """Create the log panel"""
        with self.log_lock:
            log_text = "\n".join(list(self.log_messages)[-20:])  # Show last 20 messages
        
        if not log_text:
            log_text = "No log messages yet..."
        
        return Panel(log_text, title="Log Messages", border_style="red")
    
    def _create_dashboard(self) -> Layout:
        """Create the complete dashboard"""
        self.layout["header"].update(self._create_header())
        self.layout["main"]["left"].update(self._create_motor_values_panel())
        self.layout["main"]["right"].update(self._create_advanced_values_panel())
        self.layout["footer"].update(self._create_log_panel())
        
        return self.layout
    
    def run(self):
        """Run the dashboard"""
        with Live(self._create_dashboard(), refresh_per_second=4, screen=True) as live:
            while self.running:
                try:
                    live.update(self._create_dashboard())
                    time.sleep(0.25)  # Update 4 times per second
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    self._add_log(f"Dashboard error: {e}")
                    time.sleep(1.0)

def main():
    parser = argparse.ArgumentParser(description='VESC CAN Terminal Dashboard')
    parser.add_argument('interface', nargs='?', default='demo', help='CAN interface (e.g., can0, vcan0, or full python-can URL like socketcan://can0). For CANalyst-II, this parameter is ignored. Use "demo" for demo mode.')
    parser.add_argument('--id', type=int, default=1, help='VESC controller ID (default: 1)')
    parser.add_argument('--bustype', default='socketcan', help='CAN bus type (default: socketcan, options: socketcan, pcan, vector, canalystii, etc.). For CANalyst-II, install with: pip install "python-can[canalystii]"')
    parser.add_argument('--baudrate', type=int, default=500000, help='CAN bus baudrate in bits per second (default: 500000)')
    parser.add_argument('--sender-id', type=int, default=42, help='Sender ID for CAN messages (default: 42)')
    parser.add_argument('--update-rate', type=float, default=1.0, help='Update rate in Hz (default: 1.0)')
    parser.add_argument('--demo', action='store_true', help='Run dashboard in demo mode with dummy values')
    
    args = parser.parse_args()
    
    try:
        dashboard = VescDashboard(
            args.interface, 
            args.id, 
            args.bustype, 
            args.baudrate, 
            args.sender_id,
            args.update_rate,
            args.demo or args.interface == 'demo'
        )
        dashboard.run()
    except KeyboardInterrupt:
        print("\nDashboard stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 