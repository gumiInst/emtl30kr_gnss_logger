import serial
import time
import collections
from threading import Thread, Event, Lock
import dataclasses
from datetime import datetime
from enum import Enum

from pyubx2 import (
    NMEA_PROTOCOL,
    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
)

from emtl30klr_gnss_logger.utils.logger import logger, ColoredFormatter, ColoredLogger


# For this script to be self-contained, here is a simple implementation 
# of a thread-safe deque.
class ThreadSafeDeque:
    """
    A deque implementation that is safe to use across multiple threads.
    """
    def __init__(self, *args, **kwargs):
        self._deque = collections.deque(*args, **kwargs)
        self._lock = Lock()

    def append(self, item):
        with self._lock:
            self._deque.append(item)

    def popleft(self):
        with self._lock:
            return self._deque.popleft()

    def is_empty(self):
        with self._lock:
            return len(self._deque) == 0
            
    def __len__(self):
        with self._lock:
            return len(self._deque)
        

def get_fix_type(quality: int) -> str:
    """
    Convert GNSS quality indicator to a human-readable fix type.
    """
    if quality == 0:
        return "no-fix"
    elif quality == 1:
        return "sps"  # Standard Positioning Service
    elif quality == 2:
        return "dgps" # Differential GPS
    elif quality == 3:
        return "pps" # Precise Positioning Service
    elif quality == 4:
        return "fixed-rtk" # Real-Time Kinematic (Fixed)
    elif quality == 5:
        return "float-rtk" # Real-Time Kinematic (Float)
    elif quality == 6:
        return "dead-reckoning" # Estimated (Dead Reckoning)
    elif quality == 7:
        return "manual" # Manual Input Mode
    elif quality == 8:
        return "simulation" # Simulation Mode
    else:
        return "unknown" # For any codes not in the standard

@dataclasses.dataclass
class GNSSData:
    timestamp: datetime
    gnss_time: datetime
    lat: float
    lon: float
    alt: float
    fix_type: str

class GNSSService:
    """
    A service that connects to a GNSS device via a serial port and reads
    NMEA messages in a background thread.
    """
    def __init__(
        self, 
        port: str, 
        baudrate: int, 
        timeout: float,
        **kwargs
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        
        self._ubr:UBXReader = None
        self._stop_event = Event()
        self._thread: Thread = None
        self._data_queue = ThreadSafeDeque(maxlen=100)
        
        self._msg_rate:float = None  # Message rate in Hz, can be set later if needed
        
    
    def _device_connect(self):
        """
        Connect to the GNSS device via serial port.
        Returns True on success, False on failure.
        """
        try:
            logger.info(f"Connecting to GNSS device on {self.port} at {self.baudrate} baud...")
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            self._ubr = UBXReader(self.serial_connection, protfilter=NMEA_PROTOCOL)
            logger.info("GNSS device connected successfully.")
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to connect to GNSS device: {e}")
            self.serial_connection = None
            return False
    
    def _device_disconnect(self):
        """
        Disconnect from the GNSS device.
        """
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            logger.info("GNSS device disconnected.")
        self.serial_connection = None
    
    def _loop(self):
        """
        Main loop to read data from the GNSS device.
        This method runs in a background thread.
        """
        msg_count = 0
        last_rate_time = time.time()
        
        while not self._stop_event.is_set():
            # --- Robust Reconnection Logic ---
            if self.serial_connection is None or not self.serial_connection.is_open:
                if not self._device_connect():
                    # Wait for 5 seconds before retrying connection
                    self._stop_event.wait(5)
                    continue
            
            # --- Data Reading Logic ---
            try:
                raw, parsed = self._ubr.read()
                if parsed and hasattr(parsed, "identity"):
                    if parsed.identity == "GNGGA":
                        if hasattr(parsed, "time") \
                            and hasattr(parsed, "lat") \
                            and hasattr(parsed, "lon") \
                            and hasattr(parsed, "alt") \
                            and hasattr(parsed, "quality"):
                                
                                data = GNSSData(
                                    timestamp=datetime.now(),
                                    gnss_time=parsed.time,
                                    lat=parsed.lat,
                                    lon=parsed.lon,
                                    alt=parsed.alt,
                                    fix_type=get_fix_type(parsed.quality)
                                )
                                self._data_queue.append(data)
                                msg_count += 1

                # Frame rate reporting
                now = time.time()
                if now - last_rate_time >= 1.0:
                    # logger.info(f"Received {msg_count} messages in the last second.")
                    self._msg_rate = msg_count
                    msg_count = 0
                    last_rate_time = now
                                
            except Exception as e:
                logger.error(f"Error reading from GNSS device: {e}")
                
    
    def start(self):
        """
        Start the GNSS service in a background thread.
        """
        if self.is_running():
            print("GNSS service is already running.")
            return

        logger.info("Starting GNSS service...")
        self._stop_event.clear() # Clear event in case of a restart
        self._thread = Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("GNSS service started successfully.")
    
    def stop(self):
        """
        Stop the GNSS service gracefully.
        """
        if not self.is_running():
            logger.warning("GNSS service is not running. Nothing to stop.")
            return

        logger.info("Stopping GNSS service...")
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0) # Wait for thread to finish
        
        self._device_disconnect()
        logger.info("GNSS service stopped successfully.")
    
    def is_running(self) -> bool:
        """Check if the service thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    def get_data(self):
        """
        Retrieve one NMEA message from the queue.
        Returns the message string or None if the queue is empty.
        """
        if not self._data_queue.is_empty():
            return self._data_queue.popleft()
        return None
    
    def get_msg_rate(self) -> float:
        """
        Get the message rate in Hz.
        Returns the message rate or None if not set.
        """
        return self._msg_rate


# --- Test Block ---
if __name__ == "__main__":
    # --- Configuration ---
    # On Linux, this might be /dev/ttyUSB0 or /dev/ttyACM0
    # On Windows, this will be something like COM3
    SERIAL_PORT = "COM5" 
    BAUD_RATE = 115200
    
    gnss_service = GNSSService(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        timeout=1.0  # Read timeout in seconds
    )
    
    gnss_service.start()
    
    # Keep the main thread running to process data and handle shutdown
    try:
        while gnss_service.is_running():
            data = gnss_service.get_data()
            if data:
                logger.info(f"Received data: {data}")
            else:
                logger.debug("No data received, waiting...")
            
            # Sleep to prevent high CPU usage
            time.sleep(0.1)

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, stopping GNSS service...")
    finally:
        # --- Graceful Shutdown ---
        gnss_service.stop()
        
    logger.info("GNSS service stopped. Exiting main thread.")
