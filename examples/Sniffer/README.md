# CAN Sniffer

This example lets you log raw CAN messages directly to .csv file via serial
port. Example consist of two separate parts, the STM32 example code and a
python script. The python script connects to the serial port and outputs the
CAN messages in .csv format.

The Python code supports both Linux and Windows.

## Prerequisites

Working Python 3 installation with PIP. Tested with Python 3.10 and 3.11.

To install required Python modules:

```
pip install -r requirements.txt
```

## How to use.

Select the correct pin-configuration in the example (see the main README.md in
the root of this repository), compile and upload to STM32 board.

Once the uploaded example is running and CAN is connected, run the Python
code:

```
python3 stm32_can_to_csv.py -p PORT
```

The PORT being one of COMx | /dev/ttySx | /dev/ttyACMx, depending on your
platform and setup.

To see all possible command line options:

```
python3 stm32_can_to_csv.py --help
```

By default, the code will output **can-log_DATE-TIME.csv** files in the current
directory. Use **CTRL+C** to abort logging at any time.
