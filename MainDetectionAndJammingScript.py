import os
import numpy as np
from rtlsdr import RtlSdr
from scipy.signal import welch, medfilt
import time
import threading
import RPi.GPIO as GPIO
import pygame
import logging
from argparse import ArgumentParser
import socket
import struct
import pickle
import uuid
import yaml

# Constants
SAMPLE_RATE = 2.4e6 # Hz
GAINS = [50, 70] # dBm (allow users to select gain from config file)
SCAN_TIME = 2 # s
FREQUENCY_BANDS = [2.4e9, 5.8e9] # Hz
JAMMING_FREQS = [2.412e9, 2.472e9, 5.735e9] # Hz (additional jamming frequencies)
JAMMING_POWER_MIN = -50 # dBm
JAMMING_POWER_MAX = 10 # dBm
JAMMING_POWER_INIT = JAMMING_POWER_MIN # Initialize jamming power at minimum value
JAMMING_HOP_RATE = 100e6 # Hz
ALARM_SOUND_FILE = "/home/pi/drone_detector/alarm.wav"

# GPIO setup
BUTTON_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Audio setup
pygame.mixer.init()
ALARM_SOUND = pygame.mixer.Sound(ALARM_SOUND_FILE)

class AdaptiveFilter:
    def __init__(self, filter_order, step_size):
        self.filter_order = filter_order
        self.step_size = step_size
        self.weights = np.zeros(filter_order)

    def filter(self, input_signal, desired_signal):
        """
        Apply the adaptive filter to the input signal.

        Args:
            input_signal (numpy.ndarray): The input signal.
            desired_signal (numpy.ndarray): The desired signal.

        Returns:
            numpy.ndarray: The filtered signal.
        """
        n_samples = len(input_signal)
        output_signal = np.zeros(n_samples)
        error_signal = np.zeros(n_samples)

        for n in range(self.filter_order, n_samples):
            x = input_signal[n:n-self.filter_order:-1]
            y = np.dot(self.weights, x)
            error = desired_signal[n] - y
            self.weights += 2 * self.step_size * error * x
            output_signal[n] = y
            error_signal[n] = error

        return output_signal, error_signal

def spectral_subtraction(input_signal, sample_rate, noise_estimation_time=1.0):
    """
    Apply spectral subtraction to reduce noise in the input signal.

    Args:
        input_signal (numpy.ndarray): The input signal.
        sample_rate (float): The sample rate of the input signal.
        noise_estimation_time (float): The duration of the noise estimation period in seconds.

    Returns:
        numpy.ndarray: The denoised signal.
    """
    n_samples = len(input_signal)
    n_noise_samples = int(noise_estimation_time * sample_rate)
    noise_samples = input_signal[:n_noise_samples]

    # Estimate the noise power spectral density (PSD)
    f, Pxx_noise = welch(noise_samples, fs=sample_rate, nperseg=1024)

    # Estimate the signal power spectral density (PSD)
    f, Pxx_signal = welch(input_signal, fs=sample_rate, nperseg=1024)

    # Perform spectral subtraction
    Pxx_denoised = np.maximum(Pxx_signal - Pxx_noise, 0)

    # Reconstruct the denoised signal using inverse FFT
    denoised_signal = np.fft.irfft(np.sqrt(Pxx_denoised) * np.exp(1j * np.angle(np.fft.rfft(input_signal))))

    return denoised_signal

class CooperativeLongRangeDroneDetector:
    BROADCAST_INTERVAL = 5 # seconds (configurable from YAML config file)
    CONFIG_FILE = "drone_detector_config.yaml"

    def __init__(self, device_id=None, cooperative_mode=True):
        """
        Initialize the cooperative long-range drone detector.

        Args:
            device_id (str, optional): The unique identifier for each device. Defaults to None.
            cooperative_mode (bool, optional): Enable cooperative detection mode. Defaults to True.
        """
        self.config = {}
        self.load_config()
        self.sdr = RtlSdr(self.config.get("rtlsdr_device", 0))
        self.sdr.sample_rate = SAMPLE_RATE
        self.sdr.gain = self.config.get("gain", GAINS[0])
        self.running = False
        GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=self.button_callback, bouncetime=300)
        self.logging = logging.getLogger(__name__)
        self.setup_logging()
        self.cooperative_mode = cooperative_mode
        self.devices = []
        self.lock = threading.Lock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 0))  # Bind to any available port
        self.device_id = device_id or self.get_device_id()
        self.broadcast_message = {
            'device_id': self.device_id,
            'ip': socket.gethostbyname(socket.gethostname()),
            'port': self.sock.getsockname()[1]
        }
        self.blacklist_frequencies = self.config.get("blacklist_frequencies", [])
        self.whitelist_frequencies = self.config.get("whitelist_frequencies", [])
        self.jammer_power_history = []
        self.jamming_freq_index = 0
        self.detection_threshold = self.config.get("detection_threshold", -60)
        self.last_jam_time = time.time()
        global JAMMING_POWER
        JAMMING_POWER = self.config.get("jamming_power", JAMMING_POWER_INIT)
        
    def jam(self, freq):
        """
        Jam the detected frequency using the RTLSDR as a transmitter.

        Args:
            freq (float, optional): The frequency to jam. Defaults to JAMMING_FREQ.
        """
        global JAMMING_POWER
        if time.time() - self.last_jam_time < self.config.get("jamming_cooldown", 10):
            return

        # Check if the frequency is in the blacklist
        if freq in self.blacklist_frequencies:
            print(f"Frequency {freq/1e6:.2f} MHz is in the blacklist. Skipping jamming.")
            return

        # Check if the frequency is not in the whitelist
        if freq not in self.whitelist_frequencies:
            print(f"Frequency {freq/1e6:.2f} MHz is not in the whitelist. Skipping jamming.")
            return

        self.sdr.set_center_freq(JAMMING_FREQS[self.jamming_freq_index])
        self.sdr.set_am_mode(True)
        self.sdr.set_am_gain(0)

        # Store the current jamming power in history
        with self.lock:
            self.jammer_power_history.append(JAMMING_POWER)

        print(f"Jamming at {self.sdr.center_freq/1e6:.2f} MHz with power {JAMMING_POWER:.2f} dBm")
        self.last_jam_time = time.time()

        # Increment the jamming frequency index for adaptive hopping
        self.jamming_freq_index = (self.jamming_freq_index + 1) % len(JAMMING_FREQS)
    
    def load_config(self):
        """
        Load configuration settings from the YAML config file.
        """
        try:
            with open(self.CONFIG_FILE, "r") as f:
                config = yaml.safe_load(f)
                self.config.update(config)
        except (FileNotFoundError, yaml.YAMLError) as e:
            print(f"Error loading config file: {e}")

    def save_config(self):
        """
        Save configuration settings to the YAML config file.
        """
        try:
            with open(self.CONFIG_FILE, "w") as f:
                yaml.dump(self.config, f)
        except (IOError, PermissionError) as e:
            print(f"Error saving config file: {e}")

    def setup_logging(self):
        """
        Configure logging to write messages to a file and the console.

        Raises:
            IOError, PermissionError: If there's an error creating the log file.
        """
        try:
            logging.basicConfig(level=self.config.get("logging_level", logging.INFO),
                                format='%(asctime)s [%(levelname)s] %(message)s',
                                handlers=[logging.FileHandler(self.config.get("log_file", "drone_detector.log")), logging.StreamHandler()])
        except (IOError, PermissionError) as e:
            print(f"Error configuring logging: {e}")

    def get_device_id(self):
        """
        Get the unique identifier for the device.

        Returns:
            str: The device ID.
        """
        mac_address = ':'.join(['{:02x}'.format((uuid.getnode() >> i) & 0xff) for i in range(0, 8 * 6, 8)])
        return f"{self.config.get('device_prefix', 'drone_detector')}_{mac_address}"

    def discover_devices(self):
        """
        Discover nearby devices using Wi-Fi Direct.
        """
        self.devices = []
        self.broadcast_thread = threading.Thread(target=self._broadcast)
        self.broadcast_thread.start()

        try:
            while True:
                signals, addresses = self.sock.recvfrom(1024)
                if signals:
                    device_data = pickle.loads(signals)
                    print(f"Received signal data from {device_data['device_id']}: {device_data}")
                    with self.lock:
                        if device_data not in self.devices and device_data['device_id'] != self.device_id:
                            self.devices.append(device_data)
        except KeyboardInterrupt:
            print("Stopping broadcast thread...")
            self.broadcast_thread.join()
        except socket.error as e:
            print(f"Socket error: {e}")

    def _broadcast(self):
        """
        Broadcast discovery messages periodically.
        """
        while True:
            discovery_message = {
                'device_id': self.device_id,
                'ip': socket.gethostbyname(socket.gethostname()),
                'port': self.sock.getsockname()[1],
                'discovery': True
            }
            with self.lock:
                for device in self.devices:
                    self.sock.sendto(pickle.dumps(discovery_message), (device['ip'], device['port']))
            time.sleep(self.BROADCAST_INTERVAL)

    def analyze_spectrum(self, samples):
        """
        Analyze the spectrum of the input samples using advanced filtering techniques.

        Args:
            samples (numpy.ndarray): The input samples.

        Returns:
            tuple: A tuple containing the frequency array and power spectral density.
        """
        # Apply spectral subtraction for noise reduction
        denoised_samples = spectral_subtraction(samples, self.sdr.sample_rate)

        # Optionally apply adaptive filtering
        # adaptive_filter = AdaptiveFilter(filter_order=32, step_size=0.01)
        # filtered_samples, _ = adaptive_filter.filter(denoised_samples, denoised_samples)

        # Perform spectrum analysis on the denoised (and optionally filtered) samples
        _, fs = self.sdr.get_info()
        _, _, Sxx = welch(denoised_samples, fs=fs, nperseg=int(fs * SCAN_TIME))
        freqs = np.fft.rfftfreq(Sxx.shape[1], 1 / fs)
        return freqs, Sxx.median(axis=-1)

    def send_signal_data(self):
        """
        Send signal data to nearby devices using Wi-Fi Direct.
        """
        global JAMMING_POWER
        for freq in FREQUENCY_BANDS:
            print(f"Scanning frequency: {freq/1e6:.2f} MHz")
            samples = self.sdr.read_samples(int(self.sdr.sample_rate * SCAN_TIME))
            frequencies, psd = self.analyze_spectrum(samples)
            signal_data = {
                'device_id': self.device_id,
                'freq': freq,
                'signal_data': psd[np.abs(frequencies - freq) < SAMPLE_RATE / 2].tolist(),
                'jamming_power': JAMMING_POWER
            }
            with self.lock:
                for device in self.devices:
                    self.sock.sendto(pickle.dumps(signal_data), (device['ip'], device['port']))

    def receive_signal_data(self):
        """
        Receive signal data from nearby devices using Wi-Fi Direct.

        Returns:
            list: A list of received signal data.
        """
        received_signals = []
        try:
            signals, addresses = self.sock.recvfrom(1024)
            if signals:
                device_data = pickle.loads(signals)
                print(f"Received signal data from {device_data['device_id']}: {device_data}")
                with self.lock:
                    received_signals.append(device_data)
        except (pickle.UnpicklingError, EOFError, socket.error) as e:
            print(f"Error receiving signal data: {e}")
        return received_signals

    def adjust_jamming_parameters(self, received_signals):
        """
        Adjust jamming parameters based on feedback from nearby devices.

        Args:
            received_signals (list): A list of received signal data.
        """
        global JAMMING_POWER
        if not received_signals or time.time() - self.last_jam_time < self.config.get("jamming_cooldown", 10):
            return

        total_power = sum([signal['signal_data'] for signal in received_signals])
        avg_power = total_power / len(received_signals)

        # Calculate a weighted average power considering both local and remote devices
        weight_local = 0.6
        weight_remote = 0.4
        adjusted_avg_power = (weight_local * JAMMING_POWER + weight_remote * avg_power) / (weight_local + weight_remote)

        if adjusted_avg_power > JAMMING_POWER + self.config.get("jamming_power_increment", 1):
            JAMMING_POWER += self.config.get("jamming_power_increment", 1)
        elif adjusted_avg_power < JAMMING_POWER - self.config.get("jamming_power_decrement", 1):
            JAMMING_POWER -= self.config.get("jamming_power_decrement", 1)

        # Limit jamming power within predefined bounds
        JAMMING_POWER = max(JAMMING_POWER_MIN, min(JAMMING_POWER_MAX, JAMMING_POWER))

        print(f"Adjusting jamming power: {JAMMING_POWER:.2f} dBm")
        self.config["jamming_power"] = JAMMING_POWER
        self.save_config()

    def button_callback(self, channel):
        """
        Handle button press events for user input.

        Args:
            channel (int): The GPIO channel associated with the button.
        """
        if GPIO.input(channel) == False:
            self.running = not self.running
            if self.running:
                ALARM_SOUND.play()
            else:
                pygame.mixer.stop()

    def run_cooperative(self):
    """
    Run cooperative detection mode using Wi-Fi Direct for peer-to-peer communication.
    """
    self.discover_devices()
    while self.running:
        received_signals = []
        try:
            signals, addresses = self.sock.recvfrom(1024)
            if signals:
                signal_data = pickle.loads(signals)
                print(f"Received signal data from {signal_data['device_id']}: {signal_data}")
                with self.lock:
                    received_signals.append(signal_data)
        except (pickle.UnpicklingError, EOFError, socket.error) as e:
            print(f"Error receiving signal data: {e}")

        # Adjust jamming parameters based on feedback
        if received_signals:
            self.adjust_jamming_parameters(received_signals)

        for freq in FREQUENCY_BANDS:
            with self.lock:
                print(f"Scanning frequency: {freq/1e6:.2f} MHz")
                samples = self.sdr.read_samples(int(self.sdr.sample_rate * SCAN_TIME))
                frequencies, psd = self.analyze_spectrum(samples)
                max_power = np.max(psd[np.abs(frequencies - freq) < SAMPLE_RATE / 2])
                if max_power > self.detection_threshold and self.running:
                    print(f"Detected signal at {freq/1e6:.2f} MHz with power {max_power:.2f} dBm")
                    self.jam(freq)

    def run_standalone(self):
        """
        Run standalone detection mode without cooperative features.
        """
        while True:
            if GPIO.input(BUTTON_PIN) == False:
                self.running = not self.running

            if self.running:
                for freq in FREQUENCY_BANDS:
                    samples = self.sdr.read_samples(int(self.sdr.sample_rate * SCAN_TIME))
                    frequencies, psd = self.analyze_spectrum(samples)
                    max_power = np.max(psd[np.abs(frequencies - freq) < SAMPLE_RATE / 2])
                    if max_power > self.detection_threshold:
                        print(f"Detected signal at {freq/1e6:.2f} MHz with power {max_power:.2f} dBm")
                        self.jam(freq)

    def jam(self, freq):
        """
        Jam the detected frequency using the RTLSDR as a transmitter.

        Args:
            freq (float, optional): The frequency to jam. Defaults to JAMMING_FREQ.
        """
        global JAMMING_POWER
        if time.time() - self.last_jam_time < self.config.get("jamming_cooldown", 10):
            return

        self.sdr.set_center_freq(JAMMING_FREQS[self.jamming_freq_index])
        self.sdr.set_am_mode(True)
        self.sdr.set_am_gain(0)

        # Store the current jamming power in history
        with self.lock:
            self.jammer_power_history.append(JAMMING_POWER)

        print(f"Jamming at {self.sdr.center_freq/1e6:.2f} MHz with power {JAMMING_POWER:.2f} dBm")
        self.last_jam_time = time.time()

        # Increment the jamming frequency index for adaptive hopping
        self.jamming_freq_index = (self.jamming_freq_index + 1) % len(JAMMING_FREQS)

    def resume_jamming(self):
        """
        Resume jamming using the last stored jamming power from history.
        """
        global JAMMING_POWER
        if self.jammer_power_history:
            with self.lock:
                JAMMING_POWER = self.jammer_power_history.pop()
            print(f"Resuming jamming with power {JAMMING_POWER:.2f} dBm")

    def start(self):
        """
        Start cooperative detection mode if enabled, otherwise run standalone detection mode.
        """
        if self.cooperative_mode:
            print("Starting Cooperative Mode...")
            self.run_cooperative()
        else:
            print("Starting Standalone Mode...")
            self.run_standalone()

    def stop(self):
        """
        Stop the detector and clean up resources.
        """
        GPIO.cleanup()
        self.sock.close()
        pygame.mixer.quit()

if __name__ == "__main__":
    parser = ArgumentParser(description="Long-Range Drone Detector")
    parser.add_argument("--cooperative", action="store_true", help="Enable cooperative detection mode.")
    parser.add_argument("--device_id", type=str, default=None, help="Set a custom device ID (default: automatically generated).")
    args = parser.parse_args()

    detector = CooperativeLongRangeDroneDetector(device_id=args.device_id, cooperative_mode=args.cooperative)
    try:
        print(f"Long-Range Drone Detector ready (Cooperative Mode: {args.cooperative}) Device ID: {detector.device_id}")
        detector.start()
    except KeyboardInterrupt:
        print("Exiting...")
        detector.resume_jamming() # Resume jamming with the last stored power level before exiting
    finally:
        detector.stop()