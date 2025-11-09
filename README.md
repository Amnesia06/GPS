# RTK-Based Farm Rover Navigation System

## Problem Statement

Modern precision agriculture demands accurate and real-time position data to improve efficiency and productivity. Traditional GNSS systems often lack the required accuracy for tasks such as row alignment, field mapping, and autonomous vehicle navigation, leading to inefficient farm operations and resource wastage. There is an urgent need for an accessible, reliable, and cost-effective system that enables farmers and researchers to visualize, log, and utilize centimeter-level GNSS corrections in real-time for advanced autonomous fieldwork.

## Project Overview

**RTK-Based Farm Rover Navigation** is a comprehensive Python project designed to facilitate high-precision navigation for agricultural robots or rovers. Leveraging the Emlid M2 Reach RTK GNSS module, the system connects to NTRIP services to receive RTK corrections, processes the rover's GNSS data, and provides real-time visual feedback on navigation paths. This makes it easier to monitor, log, and analyze farm vehicle movements for enhanced productivity and reproducibility.

Key highlights of this project include:

- Establishing a stable RTK connection using NTRIP caster for precision corrections.
- Logging and visualization of rover trajectory over farm fields.
- Real-time display of navigation data, including position, heading, and correction status.
- Modular Python codebase for adaptability and further customization.

## Features

- **NTRIP RTK Correction Integration:** Connects to NTRIP casters to receive real-time correction data for centimeter-level positioning accuracy using Emlid M2 Reach.
- **Data Logging:** Records position, heading, and RTK fix status for post-mission analysis.
- **Live Visualization:** Provides a GUI/map view to track rover navigation visually as it traverses the field.
- **Status Monitoring:** Displays GNSS quality, fix status, and correction stream to the operator.
- **Customizable:** Written in 100% Python for easy adaptation to different rover platforms and sensors.

## Hardware Required

- **Emlid Reach M2 GNSS Module:** For RTK GNSS positioning.
- **Linux-Compatible Rover Computer:** (e.g., Raspberry Pi, Jetson Nano) for running the main software.
- **Internet Connectivity:** For accessing NTRIP caster and data streaming.
- **Optional:** Additional sensors (IMU, cameras), display module for in-field status updates.

## Software Stack

- **Python 3.7+**
- [Matplotlib/Plotly](https://matplotlib.org/) or [PyQt5](https://riverbankcomputing.com/software/pyqt/intro) for visualization *(depending on your implementation)*
- [Requests](https://docs.python-requests.org/) for HTTP/NTRIP communications
- Other dependencies as specified in `requirements.txt`

## System Architecture

1. **NTRIP Client:** Establishes a connection to the correction stream and feeds RTCM data to the Emlid M2 module.
2. **GNSS Data Processing:** Continuously reads rover position from Emlid M2, parses NMEA/JSON data.
3. **Visualization Engine:** Updates the map with the rover's real-time position and historical trajectory.
4. **Logging Module:** Writes all navigation data to disk for further analysis.

## Setup & Installation

1. **Clone the Repository**
   ```sh
   git clone https://github.com/Amnesia06/GPS.git
   cd GPS
   ```

2. **Install Dependencies**
   ```sh
   pip install -r requirements.txt
   ```

3. **Configure NTRIP Credentials & Serial Ports**
   - Update your NTRIP caster credentials and serial port config in the provided config file or within the script as needed.

4. **Connect Your Hardware**
   - Ensure the Emlid M2 is powered and connected (typically via serial/USB).
   - Confirm your computer or SBC has internet access.

5. **Run the Application**
   ```sh
   python farm_simulation.py
   ```
 

## Usage

- Follow on-screen prompts or GUI to start the navigation session.
- Visual feedback of rover movements and correction status will be displayed in real-time.
- After completion, review the automatically generated log files for details.

## Sample Output

- Real-time map of farm with the rover’s path traced.
- Console or GUI display with GNSS fix type (Single, Float, or Fix), satellite count, and NTRIP status.
- Chronological log file of all navigation data.

## Known Limitations

- Reliable operation depends on constant internet access and line-of-sight to satellites.
- Accuracy is limited by RTK corrections and quality of setup/calibration.
- Codebase may require hardware-specific adjustments for non-Emlid or alternative rover platforms.

## Contributing

Contributions are welcome! Please open issues or submit pull requests for improvements, bug fixes, or additional features.

## License

This project is released under the MIT License. See `LICENSE` for details.

## Acknowledgements

- [Emlid](https://emlid.com/reach/) for the Reach M2 RTK module and documentation.
- Open-source NTRIP casters and data providers.
- Python and the scientific computing community.

