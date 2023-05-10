#!/usr/bin/env python3

import time
import csv
import argparse
import sys
import logging
from signal import signal, SIGINT
import serial

CSV_HEADER = ["Timestamp", "ID", "Length", "Byte 0", "Byte 1", "Byte 2", "Byte 3", "Byte 4", "Byte 5", "Byte 6", "Byte 7"]
TIME_FORMAT = "%Y%m%d-%H%M%S"
BUFFER = []
OUTPUT_FILE = f"can-bus_{time.strftime(TIME_FORMAT)}.csv"

def _sigint_handler(signal_received, frame):
    if len(BUFFER) > 0:
        write_to_csv(BUFFER, OUTPUT_FILE)
    sys.exit(0)

def arg_parser(args):
    parser = argparse.ArgumentParser(description="Parses command.")
    parser.add_argument("-p", "--port", help="Serial port (example: COM5 or /dev/ttyS1)")
    parser.add_argument("-b", "--baudrate", help="Serial baudrate (default: 115200)", default=115200, type=int)
    parser.add_argument("--BUFFER", help="Write BUFFER length (default: 2500)", default=2500, type=int)
    parser.add_argument("--debug", help="Debug mode", action="store_true")

    # no arguments given
    if len(sys.argv) == 1:
        parser.print_help(sys.stderr)
        sys.exit(1)

    options = parser.parse_args(args)
    return options

def write_to_csv(buffer, output_file):
    with open(output_file, "a", newline="", encoding="utf-8") as fd:
        writer = csv.writer(fd)
        for row in buffer:
            writer.writerow(row)

def main():
    global BUFFER

    # parse arguments
    opt = arg_parser(sys.argv[1:])

    # set log level
    log_level = logging.INFO
    if opt.debug:
        log_level = logging.DEBUG
    logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

    # write header
    write_to_csv([CSV_HEADER], OUTPUT_FILE)

    # initialize serial
    logging.info("Waiting for serial port %s", opt.port)
    while True:
        try:
            ser = serial.Serial(opt.port, opt.baudrate)
            ser.flushInput()
            logging.info("Serial port initialization done")
            break
        except (OSError, serial.SerialException):
            time.sleep(1)

    logging.info("Starting logging... Press CTRL+C anytime to quit")
    while ser.is_open:
        try:
            ser_bytes = ser.readline().decode("utf-8").strip().split(";")
            BUFFER.append(ser_bytes)
            logging.debug(ser_bytes)
            # write file after every X lines
            if len(BUFFER) == opt.BUFFER:
                write_to_csv(BUFFER, OUTPUT_FILE)
                BUFFER = []
        except Exception as err:
            write_to_csv(BUFFER, OUTPUT_FILE)
            logging.error(err)
            break

    logging.info("Logging ended")

if __name__ == '__main__':
    signal(SIGINT, _sigint_handler)
    main()
