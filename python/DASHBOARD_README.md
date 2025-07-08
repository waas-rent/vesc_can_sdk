# VESC CAN Dashboard

A real-time terminal dashboard for monitoring VESC motor controller values over CAN.

## Features

- **Real-time monitoring** of VESC motor values
- **Beautiful terminal interface** using Rich library
- **Live log stream** showing CAN messages and events
- **Configurable update rate** for data polling
- **Connection status** monitoring
- **Error tracking** and statistics

## Installation

1. Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

2. Build the VESC CAN SDK:
   ```bash
   make
   ```

## Usage

### Basic Usage

```bash
python3 vesc_dashboard.py can0
```

### Advanced Usage

```bash
# Custom VESC ID and update rate
python3 vesc_dashboard.py can0 --id 2 --update-rate 2.0

# Different CAN interface
python3 vesc_dashboard.py vcan0 --bustype socketcan

# CANalyst-II interface
python3 vesc_dashboard.py dummy --bustype canalystii --baudrate 500000
```

### Command Line Options

- `interface`: CAN interface (e.g., can0, vcan0)
- `--id`: VESC controller ID (default: 1)
- `--bustype`: CAN bus type (default: socketcan)
- `--baudrate`: CAN bus baudrate (default: 500000)
- `--sender-id`: Sender ID for CAN messages (default: 42)
- `--update-rate`: Update rate in Hz (default: 1.0)

## Dashboard Layout

The dashboard is divided into three main sections:

### Header
- Dashboard title and VESC controller information
- Connection status indicator
- Interface and update rate information

### Main Area (Left Panel - Motor Values)
- **Temperature**: FET, Motor, and MOS temperatures
- **Current**: Motor and input current, duty cycle
- **Speed**: RPM, tachometer values
- **Voltage**: Input voltage
- **Energy**: Consumed Ah and Wh

### Main Area (Right Panel - Advanced Values)
- **Motor Control**: Id, Iq currents, Vd, Vq voltages
- **Position**: PID position value
- **Charging**: Ah and Wh charged
- **Statistics**: Messages received, errors, last update time

### Footer (Log Messages)
- Real-time log of CAN messages and events
- Timestamps for all log entries
- Error messages and status updates

## Controls

- **Ctrl+C**: Exit the dashboard
- The dashboard updates automatically based on the specified update rate

## Troubleshooting

### Common Issues

1. **Library not found**: Make sure to build the SDK with `make`
2. **CAN interface error**: Check if the CAN interface exists and is up
3. **Permission denied**: Run with sudo if needed for CAN access
4. **No data received**: Check VESC ID and connection

### Debug Mode

For debugging, you can also use the interactive shell:
```bash
python3 vesc_shell.py can0
```

## Requirements

- Python 3.7+
- python-can library
- Rich library
- VESC CAN SDK (built)
- CAN interface access

## License

Same as the VESC CAN SDK - MIT License 