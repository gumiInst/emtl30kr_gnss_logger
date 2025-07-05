import sys
import argparse
import yaml
import logging
from threading import Thread, Event
from time import sleep
import math
from datetime import datetime, timezone, timedelta

import pyproj

from emtl30klr_gnss_logger.utils.logger import logger, ColoredFormatter, ColoredLogger
from emtl30klr_gnss_logger.gnss_service import GNSSService


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="ublox_gnss_streamer"
    )
    
    # YAML config file
    parser.add_argument(
        "-y", "--yaml-config", type=str,
        help="Path to YAML configuration file",
        default=argparse.SUPPRESS
    )
    
    # Ublox GNSS parameters
    parser.add_argument(
        "-p", "--serial-port", type=str,
        help="Serial port to connect to the GNSS device",
        default=argparse.SUPPRESS
    )
    parser.add_argument(
        "-b", "--serial-baudrate", type=int,
        help="Baudrate for the serial connection",
        default=argparse.SUPPRESS
    )
    parser.add_argument(
        "-t", "--serial-timeout", type=float,
        help="Timeout in secs for the serial connection",
        default=argparse.SUPPRESS
    )

    # TCP publisher parameters
    # parser.add_argument(
    #     "-a", "--tcp-host", type=str,
    #     help="TCP host to publish data to",
    #     default=argparse.SUPPRESS
    # )
    # parser.add_argument(
    #     "-q", "--tcp-port", type=int,
    #     help="TCP port to publish data to",
    #     default=argparse.SUPPRESS
    # )
    
    parser.add_argument(
        "--gt-lat", type=float,
        help="Ground truth latitude for comparison",
        default=None
    )
    parser.add_argument(
        "--gt-lon", type=float,
        help="Ground truth longitude for comparison",
        default=None
    )
    
    parser.add_argument(
        "--log-to-file", type=str, nargs='?',
        help="Path to log file. If not provided, no logging to file will be done. \
            If auto is provided, logging will be done to a file named as current date and time.",
        default=None
    )

    # others
    parser.add_argument(
        "-l", "--logger-level",
        choices=["debug", "info", "warning", "fatal", "error"],
        help="Set the logger level",
        default=argparse.SUPPRESS
    )

    return parser.parse_args(argv)

def get_utm_zone(latitude, longitude):
    """
    Calculates the UTM zone number for a given latitude and longitude.
    """
    if not (-80.0 <= latitude <= 84.0):
        raise ValueError("Latitude out of UTM range (-80 to 84 degrees).")
    return math.floor((longitude + 180) / 6) + 1


def compare_gnss(ground_truth, gnss_data):
    
    utm_zone = get_utm_zone(ground_truth['lat'], ground_truth['lon'])
    transformer = pyproj.Proj(proj='utm', zone=utm_zone, ellps='WGS84', south=ground_truth['lat'] < 0)
    
    easting_gt, northing_gt = transformer(ground_truth['lon'], ground_truth['lat'])
    easting_gnss, northing_gnss = transformer(gnss_data['lon'], gnss_data['lat'])
    
    easting_error = easting_gnss - easting_gt
    northing_error = northing_gnss - northing_gt
    hpe = math.sqrt(easting_error**2 + northing_error**2)
    
    return easting_error, northing_error, hpe


def main(argv=None):
    args = parse_args(argv)

    # Prepare a dictionary of final config values
    config_dict = {}

    # Load YAML config if provided
    if hasattr(args, 'yaml_config'):
        try:
            with open(args.yaml_config, 'r') as file:
                yaml_config = yaml.safe_load(file)
                if yaml_config:
                    config_dict.update(yaml_config)
        except Exception as e:
            raise RuntimeError(f"Failed to load YAML config file {args.yaml_config}: {e}") from e

    # Override YAML config with CLI arguments (if set by user, not default)
    for key, value in vars(args).items():
        config_dict[key] = value
            
    # Set up logging
    logger.setLevel(getattr(logging, config_dict.get('logger_level', 'info').upper()))
    if not logger.hasHandlers():
        # This block ensures that the logger has a handler after class change
        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(getattr(logging, config_dict.get('logger_level', 'info').upper()))
        formatter = ColoredFormatter(ColoredLogger.FORMAT)
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        
    logger.info("Starting ublox_gnss_streamer")
    
    # Log the final configuration
    for key, value in config_dict.items():
        logger.info(f"Config {key}: {value}")
        
    stop_event = Event()
    
    gnss_service = GNSSService(
        port=config_dict.get('serial_port', 'COM5'),
        baudrate=config_dict.get('serial_baudrate', 115200),
        timeout=config_dict.get('serial_timeout', 1.0),
        stop_event=stop_event
    )
    
    gnss_service.start()
    
    gt_lat = config_dict.get('gt_lat')
    gt_lon = config_dict.get('gt_lon')
        
    hpe = None
    northing_error = None
    easting_error = None
    
    log_file_handle = None
    log_to_file = config_dict.get('log_to_file')
    if log_to_file:
        if log_to_file.lower() == 'auto':
            try:
                log_to_file = f"gnss_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                header = "TimestampKST,GNSSTime,Latitude,Longitude,FixType,HPE(m),NorthingError(m),EastingError(m),MessageRate(Hz)\n"
                log_file_handle = open(log_to_file, 'w', encoding='utf-8', newline='')
                log_file_handle.write(header)
                log_file_handle.flush() # Ensure the header is written immediately
                logger.info(f"Logging to file: {log_to_file}")
            except Exception as e:
                logger.error(f"Failed to open log file {log_to_file}: {e}")
                log_file_handle = None

    KST = timezone(timedelta(hours=9))
    
    try:
        while not stop_event.is_set():
            data = gnss_service.get_data()
            if data:
                logger.info(f"Received data: {data}")
                
                if gt_lat is not None and gt_lon is not None:
                    # Compare with ground truth
                    easting_error, northing_error, hpe = compare_gnss(
                        {'lat': gt_lat, 'lon': gt_lon},
                        {'lat': data.lat, 'lon': data.lon}
                    )
                    logger.info(f"Ground truth comparison: "
                                f"Easting Error: {easting_error:.2f} m, "
                                f"Northing Error: {northing_error:.2f} m, "
                                f"HPE: {hpe:.2f} m")
                
                if log_file_handle:
                    ts_kst = data.timestamp
                    if isinstance(ts_kst, datetime):
                        ts_kst = ts_kst.astimezone(KST)
                    else:
                        ts_kst = datetime.fromtimestamp(ts_kst, tz=KST)
                        
                    
                    log_line = [
                        ts_kst,
                        data.gnss_time,
                        f"{data.lat:.6f}",  # Latitude
                        f"{data.lon:.6f}",  # Longitude
                        data.fix_type,  # Fix type
                        f"{hpe:.2f}" if hpe is not None else "N/A",  # HPE
                        f"{northing_error:.2f}" if northing_error is not None else "N/A",  # Northing error
                        f"{easting_error:.2f}" if easting_error is not None else "N/A",  # Easting error
                        gnss_service.get_msg_rate() if gnss_service.get_msg_rate() is not None else "N/A"  # Message rate
                    ]
                    
                    try:
                        log_file_handle.write(','.join(map(str, log_line)) + '\n')
                        log_file_handle.flush()  # Ensure the line is written immediately
                    except Exception as e:
                        logger.error(f"Failed to write to log file {log_to_file}: {e}")
                    
            else:
                logger.debug("No data received, waiting...")
            sleep(0.05)
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received, stopping GNSS service...")
    finally:
        gnss_service.stop()
        logger.info("ublox_gnss_streamer stopped.")
    



if __name__ == "__main__":
    main()