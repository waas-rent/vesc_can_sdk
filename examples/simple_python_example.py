#!/usr/bin/env python3
"""
Simple VESC CAN SDK Python Example

This example demonstrates basic usage of the VESC CAN SDK:
- Initialize the SDK with CAN interface
- Set motor current
- Retrieve motor values
- Parse and display the results

Copyright (c) 2025 waas AG (waas.rent)
"""

import sys
import time
import ctypes
import threading

# Try to import python-can
try:
    import can
except ImportError:
    print("Error: python-can library not found")
    print("Install it with: pip install python-can")
    sys.exit(1)

# Import VESC CAN SDK bindings
try:
    sys.path.append('../python')  # Add python directory to path
    from vesc_can_sdk import vesc_lib, VescValues
except ImportError as e:
    print(f"Error: Could not import VESC CAN SDK bindings: {e}")
    print("Make sure vesc_can_sdk.py is in the python directory")
    sys.exit(1)

class SimpleVescExample:
    def __init__(self, can_interface="can0", vesc_id=1, sender_id=42):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.sender_id = sender_id
        self.running = True
        
        # Response tracking
        self.response_received = threading.Event()
        self.response_data = None
        self.response_command = None
        self.latest_values = None
        
        # Initialize CAN interface
        self._init_can()
        
        # Initialize VESC SDK
        self._init_vesc_sdk()
        
        # Start monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def _init_can(self):
        """Initialize CAN interface using python-can"""
        try:
            self.can_obj = can.Bus(interface='socketcan', channel=self.can_interface, bitrate=500000)
            print(f"CAN interface initialized: {self.can_interface}")
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
                    self.can_obj.send(can_msg)
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
            
            print("VESC CAN SDK initialized successfully")
            
        except Exception as e:
            print(f"ERROR during VESC SDK initialization: {e}")
            raise
    
    def _handle_response(self, controller_id: int, command: int, data: bytes, length: int):
        """Handle VESC response"""
        try:
            if hasattr(data, 'contents'):
                data_array = ctypes.cast(data, ctypes.POINTER(c_uint8 * length)).contents
            else:
                data_bytes = bytes(data[:length])
                data_array = (c_uint8 * length)(*data_bytes)
            
            # Set response received event
            self.response_received.set()
            self.response_data = data_array
            self.response_command = command
            
            # Parse GET_VALUES response
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
                        'watt_hours': values.watt_hours,
                        'fault_code': values.fault_code
                    }
            
        except Exception as e:
            print(f"Error in _handle_response: {e}")
    
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
    
    def get_values(self):
        """Get motor values"""
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
                return True
            else:
                print("Failed to parse motor values")
                return False
        else:
            print("Failed to get motor values")
            return False
    
    def set_current(self, current: float, duration: float = 1.0):
        """Set motor current for specified duration"""
        print(f"Setting motor current to {current:.2f}A for {duration:.1f} seconds...")
        
        # Calculate number of commands to send (every 200ms)
        command_interval = 0.2  # 200ms
        num_commands = int(duration / command_interval) + 1
        
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
    
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self, 'can_obj'):
            self.can_obj.shutdown()

def main():
    """Main function demonstrating basic VESC usage"""
    print("VESC CAN SDK Simple Python Example")
    print("===================================")
    
    # Create VESC example instance
    try:
        vesc = SimpleVescExample(can_interface="can0", vesc_id=1, sender_id=42)
    except Exception as e:
        print(f"Failed to initialize VESC: {e}")
        return
    
    try:
        # Get initial motor values
        print("\n1. Getting initial motor values...")
        if not vesc.get_values():
            print("Failed to get initial values, exiting...")
            return
        
        # Set motor current (be careful!)
        print("\n2. Setting motor current...")
        print("WARNING: This will turn the motor!")
        print("Make sure the motor is free to rotate and not connected to any load.")
        
        # Ask for confirmation
        try:
            confirm = input("Do you want to continue? (yes/no): ").strip().lower()
            if confirm not in ['yes', 'y']:
                print("Motor control cancelled.")
                return
        except KeyboardInterrupt:
            print("\nMotor control cancelled.")
            return
        
        # Set a small current for a short duration
        vesc.set_current(2.0, 1.0)  # 2A for 1 second
        
        # Wait a moment for motor to stop
        time.sleep(0.5)
        
        # Get motor values again
        print("\n3. Getting motor values after current command...")
        vesc.get_values()
        
        print("\nExample completed successfully!")
        
    except KeyboardInterrupt:
        print("\nExample interrupted by user")
    except Exception as e:
        print(f"Error during example: {e}")
    finally:
        vesc.cleanup()

if __name__ == "__main__":
    main() 