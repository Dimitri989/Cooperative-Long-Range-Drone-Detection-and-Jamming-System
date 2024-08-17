# Drone Detector Setup Instructions

To run the drone detector script on a Raspberry Pi and use it with or without cooperative detection mode, follow these step-by-step instructions:

## Prerequisites:

- A Raspberry Pi running Raspberry Pi OS (formerly Raspbian) with Python 3 installed.
- An RTLSDR dongle for receiving/transmitting radio signals.
- Access to the Raspberry Pi via SSH or directly through a monitor, keyboard, and mouse.

## Steps to run the script:

### 1. Set up the Raspberry Pi:
- Connect your Raspberry Pi to the internet (wired or wireless).
- Enable SSH access if you want to control the Raspberry Pi remotely.

### 2. Install required packages:
- Update the package list and upgrade installed packages:
  ```sh
  sudo apt update
  sudo apt upgrade
  ```

- Install Git, NumPy, SciPy, RTLSDR, RPi.GPIO, PyYAML, and PyAudio (for audio playback) using `apt`:
  ```sh
  sudo apt install git numpy scipy python3-rtlsdr rpi.gpio pyaudio python3-pyaudio
  ```

### 3. Clone the drone detector repository:
- Clone the repository from GitHub:
  ```sh
  git clone https://github.com/yourusername/drone-detector.git
  cd drone-detector
  ```

### 4. Install the RTLSDR driver and libraries:
- Run the provided script to install the RTLSDR driver and libraries (if not already installed):
  ```sh
  sudo ./install_rtlsdr.sh
  ```

### 5. Configure the script:
- Open the `drone_detector.py` file in a text editor:
  ```sh
  nano drone_detector.py
  ```

- Customize the constants at the beginning of the script, such as sample rate, gain values, frequency bands, jamming frequencies, and jamming power limits.
- Save and close the file by pressing `Ctrl+X`, then `Y`, followed by `Enter`.

### 6. Configure the configuration file:
- Open the `drone_detector_config.yaml` file in a text editor:
  ```sh
  nano drone_detector_config.yaml
  ```

- Customize the configuration settings, such as device prefix, RTLSDR device number, gain, detection threshold, jamming power, logging level, and log file path.
- Save and close the file by pressing `Ctrl+X`, then `Y`, followed by `Enter`.

### 7. Run the script:

#### Cooperative mode (with Wi-Fi Direct):
  ```sh
  python drone_detector.py --cooperative [--device_id <CUSTOM_DEVICE_ID>]
  ```
  - Replace `<CUSTOM_DEVICE_ID>` with a unique identifier for your device if you want to set a custom device ID.
  - The script will start in cooperative detection mode using Wi-Fi Direct for peer-to-peer communication.

#### Standalone mode (without cooperative features):
  ```sh
  python drone_detector.py [--device_id <CUSTOM_DEVICE_ID>]
  ```
  - Replace `<CUSTOM_DEVICE_ID>` with a unique identifier for your device if you want to set a custom device ID.
  - The script will start in standalone detection mode without using Wi-Fi Direct.

### 8. Interact with the detector:
- In either cooperative or standalone mode, press the button connected to GPIO pin 18 to toggle detection and jamming on/off.
- The alarm sound will play when detection and jamming are active.

### 9. Stop the script:
- To stop the script, press `Ctrl+C` in the terminal where it's running.

## Additional notes:

- Make sure your RTLSDR dongle is connected to the Raspberry Pi before running the script.
- The script assumes that you have a button connected to GPIO pin 18 for toggling detection and jamming. If you want to use a different GPIO pin, update the `BUTTON_PIN` constant in the script accordingly.
- You can adjust the configuration settings on the fly by modifying the `drone_detector_config.yaml` file while the script is running. The script will reload the configuration file periodically or when the `CONFIG_FILE` constant is updated.
- For cooperative mode, ensure that other devices with the drone detector script running are within range and have Wi-Fi Direct enabled.

That's it! You should now be able to run the drone detector script on your Raspberry Pi in either cooperative or standalone detection modes. Happy jamming!
