# Setup
## Connecting Raspberry PI
The raspberry pi should be connected to the TELEM2 port on the flight controller, this requires a special cable that has a 6 pin GH connector on one side and can connect to pins on the raspberry pi


PX4 TELEM2 Pin -> RPi GPIO Pin
UART5_TX (2)   -> RXD (GPIO 15 - pin 10)
UART5_RX (3)   -> TXD (GPIO 14 - pin 8)
GND (6)        -> Ground (pin 6)


Additional information can be found on the [PX4](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html "PX4") site or from [ardupilot](https://ardupilot.org/dev/docs/raspberry-pi-via-mavlink.html) 

These both walk through the steps required to enable the serial port and make sure permissions are setup correctly. It can be useful to run mavproxy on the pi to ensure that the cable and serial port are working as expected.

## Configuring Drone Parameters
Once the raspberry pi is connected to the drone, it is important to set the parameters so that the pi can talk to the flight controller. The following parameters must me set using mission planner or mavproxy:

- SR2_POSITION = 1
Tell the flight controller to send us its position every 1hz

- SERIAL2_BAUD = 921
Set Serial 2 Baudrate to 921600

- SERIAL2_PROTOCOL = 2
Set Serial 2 to use Mavlink 2
    
## Setting up mission planner & mavproxy
With the drone powered up the next step is setting up mission planner and mavproxy

Before connecting the drone, we need to setup mavlink mirroring so that mavproxy can see our companion computer, instructions can be found [here](https://ardupilot.org/planner/docs/common-mp-tools.html#mavlink).

<!--TODO: Insert image here-->

Once mirroring is setup, the next step is to connect mavproxy to our mirrored link. This can be done by launching `mavproxy` from the command line, and then clicking link > add and then setting the connection type to TCP and clicking add link. After doing this we should be able to see console output from the drone as well as values from the telemetry.


## Graphing Values from Mavproxy
Once mavproxy has been connected, the next step is to graph the values. This is done using mavproxy.

[Mavproxy Graphing](https://ardupilot.org/mavproxy/docs/modules/graph.html)

1. `module load graph`
2. `graph NAMED_VALUE_FLOAT[CH4].value NAMED_VALUE_FLOAT[C2H6].value`


# Other Notes
## SSH
Using SSH is the easiest way to connect to the pi

``` shell
ssh metec@metec-pi.local
```

## Build Script
This repo contains `build.sh` which is a script that builds and transfers the binary to a pi

## Program Configuration
This program is configured through the command line and environment variables, the options can be changed both when running the program or through a .env file.

### Options

SENSOR_x_PORT: Port for sensor A, I recommend setting this by-id
SENSOR_x_BAUD: Baud rate for the sensor, should be 9600

MAVLINK_PORT: The port to use for mavlink, this should correspond to serial0. NOTE: this can change from pi to pi so make sure it is set correctly.
MAVLINK_BAUD: The baudrate used for mavlink, this must match the config setup on the drone for SERIAL2
MAVLINK_SYSTEM_ID: Mavlink System ID
MAVLINK_COMPONENT_ID: Mavlink Component ID

OUTPUT_DIRECTORY: The program will write logs to the directory specified here

RUST_LOG: Logging level used by [rust env logger](https://docs.rs/env_logger/latest/env_logger/) 
