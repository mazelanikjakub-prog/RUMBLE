# PC controller (Python)

Not part of the STM32 project. CubeIDE ignores this folder; open it in
VS Code (or any editor) and run it from a terminal on the PC.

`fin.py` reads a gamepad and sends drive commands to the car over the
Bluetooth serial link (HC-05 style SPP, 9600 baud).

## Setup

    pip install pygame pyserial

## Run

1. Pair the car's Bluetooth module with the PC.
2. Set the serial port in `fin.py` (`serial.Serial("COM8", ...)`):
   - Windows: the outgoing COM port of the paired module, e.g. `COM8`
   - Linux: e.g. `/dev/rfcomm0` (`sudo rfcomm bind 0 <MAC>`)
3. Plug in the gamepad and run `python fin.py`.

## Packet format

6 bytes per command: `0x79 Y 0x78 X 0x68 A`

- `Y`, `X`: raw left stick, -127..127 (Y forward = +, X right = +); negative values are sent as `127 + |value|`
- `A`: 1 while the A button is held, else 0
- `79 00 78 00 68 00` = stop
