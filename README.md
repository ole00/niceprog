# niceprog
SPI flash programmer for ICE40 FPGAs and other devices
![Board image](https://github.com/ole00/niceprog/raw/master/img/niceprog_main.jpg "niceprog")

Uses ESP32-S2 and S3 MCUs, spcifically Wemos S2 Mini and Wemos S3 Mini board.
It can be adapted for other ESP32 boards.

The MCU software uses Arduino SPIFlash library by Prajwal Bhattaram (the required version is included in src directory).

Features:
---------------------

* communicates via USB CDC serial line
* uses RLE compression for faster upload of bitstreams
* uses CRC checks for data transfers
* supports up to 4 multi boot partitions for ICE40 FPGAs (cold boot)
* auto creates multi boot partition table for ICE40 FPGAs
* automatic erase of written flash blocks (no need to erase the chip before writing)
* read, write, verify, identify flash chip commands
* execute ICE40 FPGA stream without writing it to flash
* supports external pass-through UART so that niceprog serves as an USB-to-UART interface.
* automatic switching between external pass-through UART and flash operations.
* PCB for convenient programming of Olimex ICE40HX8K-EVB board (see design and gerbers folders)

Performance:
* writes and verifies simple IC40HX8k streams (~132 kbytes) in less than 3 seconds.
* executes a simple IC40HX8k stream (~132 kbytes) in about a second.

How it works:
---------------------
Niceprog has 3 parts, all are located in this repo:
* PC app - sources are in src_pc directory. Compile it in Linux by supplied shell scripts (compile.sh etc.).
  See github releases to get a precompiled PC App binary for Mac OS and Windows OS.
  
  Basic usage:
  <pre>
  niceprog.exe -d COM4 i
  </pre>
  Runs niceprog PC app (Windows OS version) to identify a flash chip (command 'i') and uses COM4 to communicate
  with S2 mini connected to PC via USB cable. This is the most basic command that lets you to verify your wiring connection
  between the MCU and the Flash chip IC (or FPGA board) is functional.

  <pre>
  ./niceprog -d /dev/ttyACM0 x -f fpga_stream.bin
  </pre>
  Runs niceprog PC app (Linux version) to execute an FPGA stream file (command 'x') and uses ttyACM0 device
  to communicate with S2 mini connected to PC via USB cable.

  <pre>
  ./niceprog wv -f fpga_stream.bin
  </pre>
  Runs niceprog PC app (Linux version) to write and verify an FPGA stream file (command 'w' and command 'v') to the flash chip.
  This time it auto detects the serial device to communicate with S2 mini connected to PC via USB cable.
  
  Run the PC app without parameters to view options and other usage examples.
* Arduino app - sources in src directory and niceprog.ino sketch file. Use Arduino IDE 2.3.X and open niceprog.ino file.
  No other file needs to be opened in the IDE. **Ensure the "esp32 by espressif"  board is installed via Board's manager.**
  I used version 2.0.11, but other (more recent) versions might work as well.  Also ensure to select "ESP32S2 dev module"
  and **enable "USB CDC on boot" option** in Tools top menu. See bellow for recommended Arduino IDE parameters.
* Optional PCB that breaks out the connection to a programming header. The header works with Olimex ICE40HX8K-EVB.
  See design and gerbers folders for more information.

  You can also connect your FPGA board directly to S2 Mini or S3 Mini by Dupont wires if you prefer to.
  The S3 mini is connected via the same pins as S2 mini, the correct IO mapping is done in Arduino sketch.
  The direct wiring diagram is as follows:
<p align="center"><img src="https://github.com/ole00/niceprog/raw/master/img/niceprog_wiring.jpg" width="70%"/></p>
  

Uploading sketch via Arduino IDE:
---------------------
Ensure the follwing parameters are set when flashing the Mini S2 or S3 board:

<p align="center"><img src="https://github.com/ole00/niceprog/raw/master/img/arduino_ide_configuration.jpg" width="60%"/></p>


PCB
------
The PCB is a convenient way how to connect Wemos S2 mini (or S3 mini) to an FPGA board. It has an optional 
RGB LED and SPI Flash headers to let you program SPI Flash via programming adaptor you can find cheap on ebay or aliexpress.
You can also spare some of the sockets and solder Wemos S2 mini directly to the PCB if you do not plan to use 
the MCU board for other purposes. The PCB has a jumper J2 that selects between 3.3V and 5V applied
to Pin 1 of the J4 connector. When the jumper is removed the pin provides no voltage on that pin (it is left floating).

The PCB can be produced by online fabrication service (jlcpcb, pcbway etc.). Use the zip archive located
in 'gerbers' directory to have your Niceprog PCB produced.

ICE40HX8K-EVB mod
------
Olimex ICE40HX8K-EVB board can be powered by Niceprog completely by a simple hardware mod
done on the FPGA board. See the image bellow. 

<p align="center"><img src="https://github.com/ole00/niceprog/raw/master/img/olimex_evb_power_mod.jpg" width="40%"/></p>


Such mod allows you to power the whole FPGA dev board from USB (if your peripherals connected to the FPGA board 
do not draw too much current) and to also program it. If you use the mod, make sure to disconnect power supply
barrel jack from the FPGA dev board when Nice prog is connected.

