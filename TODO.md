# TODO

## Roadmap and Next Steps

### 1. DBC File Support
- Create DBC files for VESC CAN traffic.
- Ensure compatibility with popular CAN analysis tools like Vector CANalyzer and PCAN.
- Document the mapping of VESC commands and responses to DBC signals.

### 2. Extended Python Monitoring
- Enhance the Python monitoring script to log messages in common CAN formats (e.g., CSV, JSON, or DBC-compatible logs).
- Add options for filtering and exporting specific types of messages.
- Provide examples for integrating with third-party tools.

### 3. Python-Based Shell
- Develop an interactive Python shell for controlling VESC devices.
- Include command autocompletion and help documentation.
- Support real-time feedback and response parsing.

### 4. Terminal-Based Dashboard
- Build a terminal-based dashboard to display real-time VESC status values.
- Include features like RPM, current, voltage, and temperature monitoring.
- Add support for customizable views and alerts.

### 5. Integration with Canalystii Python Package
- Add support for the Canalystii Python package to enable CAN logging and analysis.
- Provide examples for using Canalystii with VESC devices.
- Ensure compatibility with recorded CAN traces.

### 6. Testing Against Recorded CAN Traces
- Develop a suite of tests using recorded CAN traces.
- Validate SDK functionality against real-world scenarios.
- Include tests for edge cases and error handling.

### 7. Documentation and Examples
- Improve Zephyr CAN filter setup documentation

### 8. Community Contributions
- Encourage community contributions for additional features and improvements through Discord.

### 9. Continuous Integration and Deployment
- Implement CI/CD pipelines for automated testing and deployment.
- Ensure compatibility with multiple platforms and environments.
- Create releases of the static library that can be included without compilation
