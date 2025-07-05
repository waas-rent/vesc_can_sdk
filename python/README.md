# VESC CAN SDK - Python Monitor

This Python script demonstrates how to use the VESC CAN SDK to monitor VESC motor controllers over CAN using the `python-can` library.

## Requirements

- Python 3.6+
- python-can library with CANalyst-II support
- VESC CAN SDK (built C library)

## Installation

1. Install the required Python dependencies with CANalyst-II support:
   ```bash
   pip install -r requirements.txt
   pip install "python-can[canalystii]"
   ```

2. Build the VESC CAN SDK:
   ```bash
   make
   ```

## Usage

### Basic Usage

Monitor a VESC on a SocketCAN interface:
```bash
python vesc_monitor.py can0
```

### CANalyst-II Usage

The CANalyst-II is a USB to CAN Analyzer device that supports dual-channel CAN monitoring. It's particularly useful for VESC development and debugging.

#### CANalyst-II Setup

1. **Driver Installation**:
   - The backend driver depends on `pyusb`, so a `pyusb` backend driver library such as `libusb` must be installed
   - On Windows, use a tool such as [Zadig](https://zadig.akeo.ie/) to set the CANalyst-II USB device driver to `libusb-win32`

2. **Install CANalyst-II Support**:
   ```bash
   pip install "python-can[canalystii]"
   ```

3. **Usage Examples**:
   ```bash
   # Monitor on channel 0 (default)
   python vesc_monitor.py 0 --bustype canalystii

   # Monitor on channel 1
   python vesc_monitor.py 1 --bustype canalystii

   # Monitor on both channels (0,1)
   python vesc_monitor.py "0,1" --bustype canalystii

   # Monitor with custom bitrate (default is 500000)
   python vesc_monitor.py 0 --bustype canalystii --bitrate 250000

   # Monitor with logging enabled
   python vesc_monitor.py 0 --bustype canalystii --log
   ```

#### CANalyst-II Features

- **Dual Channel Support**: Monitor two CAN buses simultaneously
- **Hardware Timestamps**: Accurate message timing from hardware
- **Cross-Platform**: Works on Windows, Linux, and macOS
- **High Performance**: USB 2.0 interface for fast data transfer

#### CANalyst-II Limitations

- **Message Ordering**: Messages received on channel 0 and channel 1 may be returned out of order between the two channels (although inside each channel, all messages are in order)
- **CAN FD Not Supported**: Only standard CAN is supported, not CAN FD
- **Bit Timing**: The f_clock value must be set to 8,000,000 (8MHz) for standard CAN

### Advanced Usage

Monitor with custom VESC ID:
```bash
python vesc_monitor.py can0 --id 2
```

Enable CAN message logging:
```bash
python vesc_monitor.py can0 --log
```

Detect motor R/L parameters:
```bash
python vesc_monitor.py can0 --detect-motor
```

Use different CAN bus types:
```bash
# PCAN-USB
python vesc_monitor.py PCAN_USBBUS1 --bustype pcan

# Vector CAN
python vesc_monitor.py 0 --bustype vector

# Virtual CAN (for testing)
python vesc_monitor.py vcan0 --bustype socketcan
```

### Supported CAN Bus Types

The script supports all CAN bus types supported by python-can:

- `canalystii` - **CANalyst-II USB to CAN Analyzer** (recommended for VESC development)
- `socketcan` - Linux SocketCAN (default)
- `pcan` - PCAN-USB adapters
- `vector` - Vector CAN interfaces
- `kvaser` - Kvaser CAN interfaces
- `ixxat` - IXXAT CAN interfaces
- `nican` - NI-CAN interfaces
- `iscan` - ISCAN interfaces
- `neovi` - NeoVI interfaces
- `usb2can` - USB2CAN adapters
- `slcan` - SLCAN (Serial Line CAN)
- `robotell` - Robotell CAN interfaces
- `nixnet` - NI-XNET interfaces
- `seeedstudio` - SeeedStudio CAN interfaces
- `cantact` - Cantact interfaces
- `gs_usb` - GS_USB interfaces
- `serial` - Serial CAN interfaces
- `udp_multicast` - UDP Multicast
- `remote` - Remote CAN interfaces

## Features

- Real-time monitoring of VESC motor controllers
- Support for multiple CAN bus types including **CANalyst-II**
- Dual-channel CAN monitoring (with CANalyst-II)
- Hardware timestamp support (with CANalyst-II)
- CAN message logging to CSV files
- Motor R/L parameter detection
- Firmware version detection
- ADC and PPM value monitoring
- Cross-platform compatibility

## Troubleshooting

### Common Issues

1. **Library not found**: Make sure to build the VESC CAN SDK first with `make`
2. **CAN interface not available**: Ensure the CAN interface is up and running
3. **Permission denied**: You may need to run with sudo for some CAN interfaces
4. **python-can not installed**: Install with `pip install python-can`
5. **CANalyst-II not detected**: 
   - Ensure the device is properly connected via USB
   - On Windows, use Zadig to set the driver to `libusb-win32`
   - Install the CANalyst-II backend: `pip install "python-can[canalystii]"`
6. **CANalyst-II driver issues**: The backend driver depends on `pyusb`, ensure `libusb` is installed

### Testing with Virtual CAN

For testing without hardware, you can use a virtual CAN interface:

```bash
# Create virtual CAN interface
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Run monitor on virtual interface
python vesc_monitor.py vcan0
```

### CANalyst-II Testing

To test the CANalyst-II interface without VESC hardware:

```bash
# Test basic connectivity
python vesc_monitor.py 0 --bustype canalystii

# Test dual-channel monitoring
python vesc_monitor.py "0,1" --bustype canalystii

# Test with different bitrates
python vesc_monitor.py 0 --bustype canalystii --bitrate 125000
```

**Note**: The CANalyst-II backend driver is unofficial and based on reverse engineering. For production use, ensure compatibility with your specific hardware version.

## CANalyst-II Hardware Information

The CANalyst-II is a USB to CAN Analyzer device produced by Chuangxin Technology. It features:

- **Dual CAN Channels**: Two independent CAN interfaces
- **USB 2.0 Interface**: High-speed data transfer
- **Hardware Timestamps**: Accurate message timing
- **Cross-Platform Support**: Windows, Linux, and macOS
- **Standard CAN Support**: Up to 1 Mbps bitrate
- **LED Indicators**: Status and activity indicators

### Hardware Compatibility

- **Supported Platforms**: Windows, Linux, and macOS
- **USB Requirements**: USB 2.0 or higher
- **Driver Requirements**: libusb backend driver
- **Bit Timing**: 8MHz clock frequency for standard CAN
- **Message Format**: Standard CAN 2.0A/2.0B

For more information about the CANalyst-II interface, visit the [python-can documentation](https://python-can.readthedocs.io/en/stable/interfaces/canalystii.html).

## License

This script is part of the VESC CAN SDK and is licensed under the MIT License. 