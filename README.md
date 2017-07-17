
![https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/signal-generator?branch=master&svg=true](https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/signal-generator?branch=master&svg=true)


![https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/signal-generator?branch=master&svg=true](https://ci.appveyor.com/api/projects/status/github/wheeler-microfluidics/signal-generator?branch=master&svg=true)
# signal-generator #

Control package for an open-source [signal generator board][3].

The [signal generator board][4] is part of the [Dropbot][5] digital
microfluidics automation system, designed and built in the [Wheeler Lab][6].

## Overview ##

This package contains:

 - Firmware compatible with Arduino Uno.
 - Installable Python package for interfacing with Arduino firmware through
   serial port or i2c (through a serial-to-i2c proxy).

## Install ##

The Python package can be installed through `pip` using the following command:

    pip install wheeler.signal-generator

## Upload firmware ##

To upload the pre-compiled firmware included in the Python package, run the
following command:

    python -m signal_generator.bin.upload <board type>

replacing `<board type>` with either `uno` or `mega2560`, depending on the
model of the board.

This will attempt to upload the firmware by automatically discovering the
serial port.  On systems with multiple serial ports, use the `-p` command line
argument to specify the serial port to use.  For example:

    python -m signal_generator.bin.upload -p COM3 uno

--------------------------------------------------

## Usage ##

After uploading the firmware to the board, the `signal_generator.Proxy` class can be
used to interact with the Arduino device.

See the session log below for example usage.

### Example interactive session ###

    >>> from serial import Serial
    >>> from signal_generator import Proxy

Connect to serial device.

    >>> serial_device = Serial('/dev/ttyUSB0', baudrate=115200)

Initialize a device proxy using existing serial connection.

    >>> proxy = Proxy(serial_device)

Query descriptive properties of device.

    >>> proxy.properties()
    base_node_software_version                               0.9.post8.dev141722557
    name                                                                  signal_generator
    manufacturer                                                        Wheeler Lab
    url                           http://github.com/wheeler-microfluidics/signal...
    software_version                                                            0.1
    dtype: object

--------------------------------------------------

## Firmware development ##

The Arduino firmware/sketch is located in the
`signal_generator/Arduino/signal_generator` directory.  The key functionality
is defined in the `signal_generator::Node` class in the file `Node.h`.

Running the following command will build the firmware using [SCons][2] for
Arduino Uno and will package the resulting firmware in a Python package, ready
for distribution.

    paver sdist

### Adding new remote procedure call (RPC) methods ###

New methods may be added to the RPC API by adding new methods to the
`signal_generator::Node` class in the file `Node.h`.

# Author #

Copyright 2015 Christian Fobel <christian@fobel.net>,
Ryan Fobel <ryan@fobel.net>


[1]: https://www.arduino.cc/en/Reference/HomePage
[2]: http://www.scons.org/
[3]: http://microfluidics.utoronto.ca/trac/dropbot/wiki/SignalGeneratorBoard
[4]: http://microfluidics.utoronto.ca/git/kicad___signal_generator_board.git
[5]: http://microfluidics.utoronto.ca/dropbot/
[6]: http://microfluidics.utoronto.ca
