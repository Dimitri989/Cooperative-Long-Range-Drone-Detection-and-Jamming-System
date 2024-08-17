# Cooperative-Long-Range-Drone-Detection-and-Jamming-System
This script is designed to detect and jam drone signals
Here's a simplified explanation of its features and why it's effective:

Detection and Jamming: The primary purpose of this script is to detect drone signals and then jam them to prevent the drone from operating correctly. It uses a software-defined radio (SDR) to scan for signals in specific frequency bands commonly used by drones.
2.
Cooperative Mode: The script can operate in a cooperative mode where multiple devices work together to detect drones. This mode uses Wi-Fi Direct for peer-to-peer communication, allowing devices to share signal data and improve detection accuracy.
3.
Standalone Mode: If cooperative mode is not enabled, the script runs in standalone mode, where it independently scans for and jams drone signals.
4.
Adaptive Filtering and Spectral Subtraction: The script uses advanced signal processing techniques like adaptive filtering and spectral subtraction to reduce noise and improve the accuracy of signal detection. This helps in distinguishing drone signals from background noise.
5.
Frequency Hopping: To make jamming more effective, the script uses frequency hopping. It changes the jamming frequency periodically to cover a broader range of frequencies, making it harder for the drone to avoid being jammed.
6.
User Interaction: The script includes a button for user interaction. Pressing the button can start or stop the detection and jamming process. It also plays an alarm sound when a drone signal is detected.
7.
Logging and Configuration: The script logs important events and errors, which helps in monitoring its operation. It also loads configuration settings from a YAML file, allowing users to customize various parameters like detection thresholds, jamming power, and frequency bands.
8.
Resource Management: The script ensures proper resource management by cleaning up GPIO settings and closing network sockets when it stops running. This prevents resource leaks and ensures the system remains stable.

Overall, this script is effective because it combines multiple advanced techniques to accurately detect and jam drone signals, operates in both cooperative and standalone modes, and provides user-friendly features for interaction and customization.

Cooperative Mode Overview
Cooperative mode allows multiple devices to work together to detect and jam drone signals. This mode leverages peer-to-peer communication using Wi-Fi Direct, enabling devices to share signal data and improve the overall detection accuracy.

Key Features of Cooperative Mode
1.
Device Discovery:
The script discovers nearby devices using Wi-Fi Direct. Each device broadcasts its presence and listens for broadcasts from other devices.
This is handled by the discover_devices method, which starts a thread to periodically broadcast discovery messages and listens for incoming messages from other devices.
2.
Signal Data Sharing:
Devices share signal data with each other. When a device detects a signal, it sends the signal data to all discovered devices.
The send_signal_data method is responsible for sending the signal data, while the receive_signal_data method handles receiving data from other devices.
3.
Adjusting Jamming Parameters:
Based on the received signal data from other devices, each device can adjust its jamming parameters to optimize the jamming effectiveness.
The adjust_jamming_parameters method calculates the average signal power from the received data and adjusts the jamming power accordingly.
4.
Synchronization:
Devices synchronize their jamming efforts to avoid interference and maximize the jamming coverage. This is achieved by sharing jamming frequencies and power levels.
The run_cooperative method coordinates the overall process, including device discovery, signal data sharing, and jamming parameter adjustments.


Benefits of Cooperative Mode
Improved Detection Accuracy: By sharing signal data, devices can detect drones more accurately, even if the signal is weak or intermittent.
Enhanced Jamming Effectiveness: Coordinated jamming efforts ensure that the drone is effectively jammed, making it difficult for the drone to operate.
Redundancy and Reliability: Multiple devices working together provide redundancy, ensuring that the system remains effective even if one device fails.


How It Works
1.
Initialization:
When the script starts in cooperative mode, it initializes the necessary components, including the SDR, network socket, and device ID.
2.
Device Discovery:
The script starts broadcasting discovery messages and listens for incoming messages to build a list of nearby devices.
3.
Signal Detection and Sharing:
Each device scans the specified frequency bands for drone signals. When a signal is detected, the device sends the signal data to all discovered devices.
4.
Jamming Coordination:
Devices adjust their jamming parameters based on the received signal data and synchronize their jamming efforts to cover a broader range of frequencies.
5.
Continuous Operation:
The script continues to run in a loop, periodically scanning for signals, sharing data, and adjusting jamming parameters until it is stopped.


In summary, cooperative mode enhances the detection and jamming capabilities of the system by enabling multiple devices to work together, share information, and coordinate their actions. This results in a more robust and effective solution for detecting and jamming drone signals.

