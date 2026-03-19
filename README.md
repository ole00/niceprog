# niceprog
SPI flash programmer for ICE40 FPGAs and other devices

Uses ESP32-S2 MCU, spcifically Wemos S2 Mini board.
It can be adapted for other ESP32 boards.

The MCU software uses Arduino SPIFlash library by Prajwal Bhattaram (the required version is included in src directory).

Features:
* communicates via USB CDC serial line
* uses RLE compression for faster upload of bitstreams
* uses CRC checks for data transfers
* supports up to 4 multi boot partitions for ICE40 FPGAs (cold boot)
* auto creates multi boot partition table for ICE40 FPGAs
* automatic erase of written flash blocks (no need to erase the chip before writing)
* read, write, verify, identify flash chip commands
* supports external pass-through UART so that niceprog serves as USB-to-UART interface.
* automatic switching between external pass-through UART and flash operations.

Performance:
* writes and verifies simple IC40HX8k streams (~132 kbytes) in less than 3 seconds.

Niceprog has 2 parts, both are located in this repo:
* PC app - sources in src_pc directory. Compile it in Linux by supplied shell scripts. Run the PC app without parameters to view options
  and basic usage examples.
* Arduino app - sources in src directory and niceprog.ino sketch file. Use Arduino IDE 2.3.X and open niceprog.ino file.
  No other file needs to be opened in the IDE. Ensure the "esp32 by espressif"  board is installed via Board's manager.
  I used version 2.0.11, but other more recent versions might work as well.  Also ensure to select "ESP32S2 dev module"
  and enable "USB CDC on boot" option in Tools top menu. Select "PSRAM enabled" and Upload mode "Internal USB" for correct operation.

* more info to come...
