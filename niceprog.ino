
/*
 *  niceprog : a sketch for ESP32 Arduino based Lattice iCE programmers
 *
 *  olin : 2026
 *  inspired by iceprog tool
 *  
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <SPI.h>

#define VERSION "1.2"

//=== USB Check ==============================
#if ARDUINO_USB_CDC_ON_BOOT == 0
#error USB CDC On Boot is Disabled. Enable it in Tools menu.
#endif

//=== BOARD Config =============================

//--- ESP32-S2 : Wemos Mini S2 ----------------
#ifdef CONFIG_IDF_TARGET_ESP32S2

#define BOARD_MIN_BUF (160 * 1024)

// 1: use a static array 0: use malloc (for PSRAM)
#define STATIC_MALLOC 0

// ESP32-S2 has limited internal ram
#ifdef BOARD_HAS_PSRAM
    #define BOARD_MAX_CHUNK (16 *1024)
#else
    #define BOARD_MAX_CHUNK (8 *1024)
#endif


// ESP32 onboard LED
#define PIN_LED 15

// niceprog RGB LED
#define PIN_LED_B 5
#define PIN_LED_R 6
#define PIN_LED_G 7

// CDONE is raised high when FPGA finishes loading the design from flash
#define PIN_CDONE 10

// RESET keps the FPGA IC in reset state so that the flash IC can be reprogrammed
#define PIN_RESET 8

// can be used to drive MOSfet to power your FPGA board
#define PIN_UEXT_POWER 9

// SPI flash pins
#define PIN_CS  13
#define PIN_SCK 36
#define PIN_MISO 37
#define PIN_MOSI 35

// for external serial port connected to either FPGA or other device
#define PIN_SERIAL_RX 2
#define PIN_SERIAL_TX 1

#elif CONFIG_IDF_TARGET_ESP32S3
//--- ESP32-S3 : Wemos Mini S3 ----------------

// 1: use a static array 0: use malloc (for PSRAM)
#define STATIC_MALLOC 0

#define BOARD_MIN_BUF (160 * 1024)
#define BOARD_MAX_CHUNK (16 *1024)

// ESP32 onboard LED
#define PIN_LED 47

// niceprog RGB LED
#define PIN_LED_B 4
#define PIN_LED_R 6
#define PIN_LED_G 12

// CDONE is raised high when FPGA finishes loading the design from flash
#define PIN_CDONE 8

// RESET keps the FPGA IC in reset state so that the flash IC can be reprogrammed
#define PIN_RESET 7

// can be used to drive MOSfet to power your FPGA board
#define PIN_UEXT_POWER 13

// SPI flash pins
#define PIN_CS  9
#define PIN_SCK 38
#define PIN_MISO 44
#define PIN_MOSI 36

// for external serial port connected to either FPGA or other device
#define PIN_SERIAL_RX 3
#define PIN_SERIAL_TX 1

#else
    #error Unsupported board.
#endif

// ====================================================================


#include "src/SPIFlash.h"
#include "src/crc16.h"


#define DEBUG_SERIAL1  0

#if DEBUG_SERIAL1
#define DEBUG(X) Serial1.printf X
#else
#define DEBUG(X)
#endif

//commands list
#define CMD_NONE 0
#define CMD_PROMPT '*'
#define CMD_UPLOAD 'u'
#define CMD_CHUNK  'c'
#define CMD_READ_FLASH 'r'
#define CMD_DOWNLOAD 'd'
#define CMD_FULL_ERASE 'e'
#define CMD_READ_ID   'i'
#define CMD_WRITE 'w'
#define CMD_VERIFY 'v'
#define CMD_BOOT_SECTOR_CHECK 'b'
#define CMD_BOOT_SECTOR_WRITE 'B'
#define CMD_EXECUTE 'x'

// Upload buffer total size - allocation of that size is tried first
// and if it fails, then the MIN_BUF size is tried to allocate.
#define MAX_BUF (512 * 1024)
// Minimum size so that the ICE40HX8 can execute a bit stream.
// If you don't need to execute bit stream, then this size can
// be much smaller, down to 256 bytes (MAX_CHUNK must be also 256 bytes in such case).
// At 256 bytes, all operations wil lbe very slow, but functional.
#define MIN_BUF BOARD_MIN_BUF

// The chunk size during upload and download.
// Do not make less than 256 bytes.
// smaller chunk sizes dramatically reduce speed of read/write operations
#define MAX_CHUNK BOARD_MAX_CHUNK

// Considerations for selecting the right MIN_BUF and MAX_CHUNK values:
// * ensure both fill the available RAM of your MCE as much as possible (that still ensures reliable operation)
// * large value of MAX_CHUNK is more important than large value of MIN_BUF, therefore - if for example your MCU
//   has total of 64 kbytes, then it is better to have MIN_BUF=32kb and MAX_CHUNK=16kb
//   than to have MIN_BUF=48kb and MAX_CHUNK=1kb. 
// * Sizes of MAX_CHUNK larger than 16 kBytes bring very small speed improvements.
// * The smallest possible configuration is MIN_BUF=256 bytes and MAX_CHUNK=256 bytes
//   possibly fitting the MCU with only 2kbytes or RAM.

// in micro seconds
#define MAX_FLASH_IDLE_TIME 200000

#define MODE_IDLE 0
#define MODE_FLASH 1

#define ERROR_INVALID_PARTITION    0xFFEB
#define ERROR_NO_PARTITION         0xFFEC
#define ERROR_FLASH_ACCESS         0xFFFD
#define ERROR_PARTITION_ALIGNMENT  0xFFFE
#define ERROR_PARTITION_SMALL      0xFFFF


// instatiate the SPIFlash library object and pass the SPI pins
SPIFlash flash(PIN_CS);

uint32_t maxBuf;
uint16_t maxBufChunks;
#if STATIC_MALLOC
uint8_t data_buffer[MIN_BUF];
#else
uint8_t* data_buffer;
#endif
char line[32];
short lineIndex;
char endOfLine;
uint32_t uploadSize;
uint32_t uploadPos = 0; //absolute offset into the data_buffer during several uploads
uint32_t uploadChunkPos = 0; // relative offset into the buffer during upload of a single chunk
uint16_t chunkCrc;
uint16_t chunkLen; // may be smaller than MAX_LEN, in such case it is RLE encoded
uint32_t chipId;
uint8_t rle_buffer[MAX_CHUNK];
uint8_t uploading = 0;
uint8_t mode = MODE_IDLE;
unsigned long flashTime;

char handleTerminalCommands();

void setupMode() {
    if (MODE_IDLE == mode) {
        SPI.end(); //release SPI pins
        digitalWrite(PIN_CS, HIGH); //SPI CS is deasserted
        digitalWrite(PIN_RESET, HIGH); // start FPGA loading from flash
        
        digitalWrite(PIN_LED_G, 0);
        digitalWrite(PIN_LED_B, 1);
        digitalWrite(PIN_LED_R, 1);

    } else {
        pinMode(PIN_CS, OUTPUT);
        digitalWrite(PIN_CS, HIGH); //SPI CS is deasserted
        digitalWrite(PIN_RESET, LOW); // keep FPGA in reset

    	SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
        SPI.setDataMode(0);
        SPI.setBitOrder(MSBFIRST);
        SPI.setFrequency(10000000);

        // basic detection of connected flash chip
        flash.powerUp();
        flash.begin();
        chipId = flash.getJEDECID();
        flash.powerDown();

        digitalWrite(PIN_LED_B, 0);

        // GREEN LED - flash chip detected, RED LED - flash chip not detected
        if (chipId == 0xFFFFFF || chipId == 0x000000) {
            digitalWrite(PIN_LED_R, 1);
            digitalWrite(PIN_LED_G, 0);
        } else {
            digitalWrite(PIN_LED_R, 0);
            digitalWrite(PIN_LED_G, 1);
        }
        flashTime = micros();
    }
}

void setup() {
    bool result;
    chipId = 0;
    pinMode (0, INPUT_PULLUP);
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    pinMode(PIN_RESET, OUTPUT);
    // keep FPGA in reset state
    digitalWrite(PIN_RESET, LOW);


    pinMode(PIN_CDONE, INPUT_PULLUP);

    pinMode(PIN_LED, OUTPUT);

    // show the program is starting by litting the LED
    digitalWrite(PIN_LED,1);

    pinMode(PIN_UEXT_POWER, OUTPUT);

    pinMode(PIN_LED_R,OUTPUT);
    pinMode(PIN_LED_G,OUTPUT);
    pinMode(PIN_LED_B,OUTPUT);

    Serial.begin(115200);

    // setup the external serial port
    Serial1.begin(115200,SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX, false, 100);

    mode = MODE_FLASH;
    setupMode();

    delay(500);

    mode = MODE_IDLE;
    setupMode();



    // turn off ESP32 module LED
    digitalWrite(PIN_LED,0);

    readGarbage(); // read bytes from serial port that may still be in the buffer

#if STATIC_MALLOC
    maxBuf = MIN_BUF;
#else
    // allocate download buffer on heap (presumably on PSRAM if it is available)
    data_buffer = (uint8_t*) malloc(MAX_BUF);
    if (NULL == data_buffer) {
        // no PSRAM available? try to allocate the minimum buffer
        data_buffer = (uint8_t*) malloc(MIN_BUF);
        if (NULL == data_buffer) {
            DEBUG(("can't malloc buffer\r\n"));
            // mark an erro by flashing the onboard LED
            while(1) {
                delay(100);
                digitalWrite(PIN_LED,0);
                delay(100);
                digitalWrite(PIN_LED,1);
            }
        } else {
            maxBuf = MIN_BUF;
        }
    } else {
        maxBuf = MAX_BUF;
    }
#endif
    maxBufChunks = maxBuf / MAX_CHUNK;
    DEBUG(("niceprog ready\r\n"));
}

// read from serial line and discard the data
void readGarbage() {
  while (Serial.available() > 0) {
    Serial.read();
  }  
}

// Converts textual hex value 0-9, A-F to a number.
unsigned char toHex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

// Parses hexdecimal integer number typed as 2 digit.
// Returns the number value.
unsigned short parse2hex(char i) {
    if (line[i] == '\r' || line[i] ==  0 || line[i] == ' ') {
        return -1; 
    }

    unsigned short v = toHex(line[i++]) << 4;
    return v + toHex(line[i]); 
}

// Parses hexdecimal integer number typed as 4 digit.
// Returns the number value.
unsigned short parse4hex(char i) {
    unsigned short v;
    if (line[i] == '\r' || line[i] ==  0 || line[i] == ' ') {
        return -1; 
    }

    v = toHex(line[i++]);
    v <<= 4;
    v |= toHex(line[i++]);
    v <<= 4;
    v |= toHex(line[i++]);
    v <<= 4;
    v |= toHex(line[i]);
    return v ;
}

// find a span of characters within a byte buffer for RLE encoding purpose
static unsigned char findRleSpan(unsigned char* buf, int len) {
    int i;
    unsigned char c = buf[0];
    if (len == 1) {
        return 1;
    }
    // find a span of distinct characters
    if (c != buf[1]) {
        for (i = 1; i < len && i < 128 ;i++) {
            if (buf[i] == c) {
                return i;
            }
            c = buf[i];
        }
        return len < 127 ? len : 127;
    } 
    // find a span of repeating character
    else {
        for (i = 1; i < len && i < 128 ;i++) {
            if (buf[i] != c) {
                return 0x80 | i;
            }
        }
        return len < 127 ? (0x80 | len) : (0x80 | 127);
    }
}

// tries to RLE encode the source buffer and place it into destination buffer.
// returns the size of RLE encoded buffer or -1 if RLE is bigger than source buffer length.
static int encodeRle(unsigned char*  dst, unsigned char* src, int len) {
    int srcPos = 0;
    int dstPos = 0;
    int totalLen = len;

    while (srcPos < totalLen && dstPos < totalLen) {
        int spanLen;
        unsigned char span = findRleSpan(src + srcPos, len);
        spanLen = span & 0x7F;
        dst[dstPos++] = span;
        // check destination buffer boundary
        if (dstPos + spanLen >= totalLen) {
            return -1; // RLE encoded buffer is bigger than or the same size as the source buffer
        }
        //repeating span
        if (span & 0x80) {
            dst[dstPos++] = src[srcPos]; // store repeating character
        }
        // distinct span
        else {
            memcpy(dst + dstPos, src + srcPos, spanLen); //store a span-length of distinct characters
            dstPos += spanLen;
        }
        srcPos += spanLen;
        len -= spanLen;
    }
    if (dstPos >= totalLen) {
        return -1;
    }
    return dstPos;
}

// decodes source RLE encoded buffer (of encLen size) and puts the decoded result into destination buffer.
// the output size should be MAX_CHUNK size.
static void decodeRle(unsigned char* dst, unsigned char* src, int encLen)
{
    int i;
    int srcPos = 0;
    int dstPos = 0;

    while (srcPos < encLen) {
        unsigned char c = src[srcPos++];
        //repeating span
        if (c & 0x80) {
            int len = c & 0x7F;
            unsigned char data = src[srcPos++];
            for (i = 0; i < len; i++) {
                dst[dstPos++] = data;
            }
        }
        // distinct span
        else {
            memcpy(dst + dstPos, src + srcPos, c);
            dstPos += c;
            srcPos += c;
        }
    }
}

// in idle mode the USB serial is used as a pass trough for the external TTL UART serial
static char handleIdleMode() {
    uint8_t buf[256];

    // read from external TTL UART and write to USB UART
    int len = Serial1.available();
    if (len > 0) {
        len = Serial1.read(buf, len);
        if (len > 0) {
            Serial.write(buf, len);
            Serial.flush();
        }
    }

    // read from USB UART and write to external TTL UART
    len = Serial.available();
    if (len > 0) {
        len = Serial.read(buf, len);
        if (len > 0) {
            DEBUG(("rec: %d\n", len));
            // check for magic - when detected then switch to flash mode
            if (0 == strncmp("#*\t*nice\tprog#\r", (const char*)buf, 15)) {
                mode = MODE_FLASH;
                setupMode();
                readGarbage();
                return CMD_PROMPT;
            } else {
                Serial1.write(buf, len);
                Serial1.flush();
            }
        }
    }
    return CMD_NONE;
}

void loop() {
    char command = CMD_NONE;

    if (MODE_IDLE == mode) {
        command = handleIdleMode();
        if (command == CMD_NONE) {
            return;
        }
    }

    //check whether USB serial has any commands available
    if (command == CMD_NONE) {
        command = handleTerminalCommands();
    }


    if (command != CMD_NONE) {
        DEBUG(("command=%c\r\n", command));
        switch (command) {
            case CMD_PROMPT:
                DEBUG(("prompt"));
                uploading = 0;
                Serial.printf("niceprog v." VERSION "\r\n");
                Serial.printf("maxBuf=%08x\r\n", maxBuf);
                Serial.printf("chunkSize=%08x\r\n", MAX_CHUNK);
                Serial.printf(">\r\n");
            break;

            case CMD_UPLOAD:
                uploadSize = parse4hex(2);
                DEBUG(("u size=%d\r\n", uploadSize));
                uploadSize *= MAX_CHUNK;
                uploadPos = 0;
                uploadChunkPos = 0;
                Serial.printf(">\r\n"); Serial.flush();
                DEBUG(("total size=%d line=%s\r\n", uploadSize, line));
            break;

            case CMD_CHUNK:
                chunkCrc = parse4hex(2);
                chunkLen = parse4hex(6);
                uploading = 1;
                uploadChunkPos = 0;
                Serial.printf(">\r\n"); Serial.flush();
                DEBUG(("chunk crc=%04x line=%s\r\n", chunkCrc, line));
            break;

            case CMD_READ_FLASH: {
                uint16_t startBlock = parse4hex(2);
                uint16_t blockCount = parse4hex(6);
                DEBUG(("read flash block=%d blkcnt=%d\r\n", startBlock, blockCount));
                readBlocks(startBlock, blockCount, 0);

                Serial.printf("OK\r\n>\r\n"); Serial.flush();
                DEBUG(("read flash done\r\n"));
            }
            break;

            case CMD_DOWNLOAD: {
                uint32_t dlOffset = parse4hex(2);
                dlOffset *= MAX_CHUNK;
                uint32_t total = MAX_CHUNK;
                crc_t crc = crc_update(0, data_buffer + dlOffset, MAX_CHUNK);
                Serial.printf("OK:%04x\r\n>\r\n", crc); Serial.flush();
                usleep(10*1000);
                DEBUG(("dl...crc=%04x\r\n", crc));
                //DEBUG(("%06x : %02x %02x %02x %02x \r\n", dlOffset, data_buffer[dlOffset], data_buffer[dlOffset+1], data_buffer[dlOffset+2], data_buffer[dlOffset+3]));
                while (total) {
                    int w = Serial.write(data_buffer + dlOffset, total > 1024 ? 1024 : total);
                    total -= w;
                    dlOffset += w;
                    //DEBUG(("  wr:%d / %d\r\n", w, total));
                }
                Serial.printf(">\r\n"); Serial.flush();
                DEBUG(("dl finished\r\n"));
            }
            break;

            case CMD_FULL_ERASE: {
                bool result;
                DEBUG(("Full erase\r\n"));
                result = eraseAll();
                Serial.printf("%s\r\n>\r\n", result ? "OK" : "ER:"); Serial.flush();
                DEBUG(("erase result: %s\r\n", result ? "OK" : "failed:"));
            } break;

            case CMD_READ_ID:
                DEBUG(("read ID\r\n"));
                Serial.printf("JDEC Id=%08x\r\n>\r\n", chipId);
            break;

            case CMD_WRITE: {
                uint32_t wrBlock = parse4hex(2);
                bool result = writeAll(wrBlock);
                Serial.printf("%s\r\n>\r\n", result ? "OK" : "ER:write failed"); Serial.flush();
                DEBUG(("write result: %s\r\n", result ? "OK" : "failed:"));
            }
            break;

            case CMD_VERIFY: {
                uint32_t verifyBlock = parse4hex(2);
                uint32_t errorCount = 0;
                bool result = verifyAll(verifyBlock, &errorCount);
                if (result) {
                    Serial.printf("OK\r\n>\r\n");
                } else {
                    Serial.printf("ER:verify failed. errors=%d\r\n>\r\n", errorCount);
                }
                Serial.flush();
                DEBUG(("verify result: %s\r\n", result ? "OK" : "failed:"));
            }
            break;

            case CMD_BOOT_SECTOR_CHECK:
            case CMD_BOOT_SECTOR_WRITE:
            {
                uint8_t partitionIndex = parse2hex(2);
                uint16_t fileBlockCount = parse4hex(4);
                uint16_t result = checkOrCreateBootSectorIce40(partitionIndex, fileBlockCount, command == CMD_BOOT_SECTOR_CHECK);
                Serial.printf("OK:%04x\r\n>\r\n", result);
                Serial.flush();
                DEBUG(("boot sector result: %04x\r\n", result));
            } break;

            case CMD_EXECUTE:
            {
                uint32_t streamSize = parse4hex(2);
                streamSize <<= 16;
                streamSize |= parse4hex(6);
                DEBUG(("Execute %d bytes\r\n", streamSize));
                bool result = executeStream(streamSize);
                Serial.printf("%s:\r\n>\r\n", result ? "OK" : "ER");
                Serial.flush();
                DEBUG(("Execute result=%d\r\n", result));
            } break;
        }
        flashTime = micros();
    } else {
        // when no flash command is passed within few a split of a seconds then enter idle mode
        if (micros() - flashTime >= MAX_FLASH_IDLE_TIME) {
            mode = MODE_IDLE;
            setupMode();
        }
    }
}

// Reads from USB serial and tries to decode a command from PC app.
// The PC app commands are text based.
// It can also read uploaded binary data when data upload is
// issues.
char handleTerminalCommands() {
    char c;
    int len;

    // in binary upload mode
    if (uploading) {
        uint8_t* dstBuf = (chunkLen == MAX_CHUNK) ? (data_buffer + uploadPos) : rle_buffer ;
        uint8_t* crcBuf = dstBuf; 
        while (uploading) {
            len = Serial.available();
            if (len) {
                int r;
                //ensure we don't read past the chunk size
                if (chunkLen - uploadChunkPos < len) {
                    len = chunkLen - uploadChunkPos;
                }
                r  = Serial.readBytes(dstBuf + uploadChunkPos, len);
                uploadChunkPos += r;

                // upload has finished
                if (uploadChunkPos == chunkLen) {
                    uploadChunkPos = 0;
                    uploading = 0;
                    DEBUG(("chunk uploaded pos=%d\r\n", uploadPos));
                    crc_t crc = crc_update(0, crcBuf, chunkLen);
                    if (crc == chunkCrc) {
                        DEBUG(("crc OK\r\n"));
                    } else {
                        DEBUG(("crc failed\r\n"));
                        Serial.printf("ER: crc failed at block starting from 0x%06x\r\n",data_buffer + uploadPos - MAX_CHUNK);
                    }
                    //decode RLE compressed data
                    if (chunkLen < MAX_CHUNK) {
                        decodeRle(data_buffer + uploadPos, rle_buffer, chunkLen);
                    }
                    DEBUG(("data: %02x %02x %02x %02x\r\n", data_buffer[uploadPos], data_buffer[uploadPos+1], data_buffer[uploadPos+2], data_buffer[uploadPos+3] ));
                    uploadPos += MAX_CHUNK; //move the total upload position within the upload buffer
                }
            }
        }
    }

    // in text/command mode 
    while (Serial.available() > 0) {
        c = Serial.read();

        line[lineIndex] = c;
        if (c == '\n' || c == '\r') {
            endOfLine = 1;
        }
        //DEBUG(("line: %s\r\n", line));

        if (lineIndex >= sizeof(line)- 2) {
            lineIndex = 0;
            readGarbage();
            Serial.println();
            Serial.println(F("Error: line too long."));
        } else {
           lineIndex++;
        }
    }
    // end of line was found -> decode command
    if (endOfLine) {
        c = CMD_NONE;
        DEBUG(("line index=%d %c \r\n", lineIndex, line[0]));

        //single letter command entered
        if (lineIndex >= 3 && line[0] == '#') {
            c = line[1];  
            line[lineIndex] = 0;
            lineIndex = 0;
        }
        endOfLine = 0;
        return c;
    }
    return CMD_NONE; 
}

static char isFpgaMagicIce40(uint8_t* b) {
    return (b[0] == 0x7E &&  b[1] == 0xAA && b[2] == 0x99 && b[3] == 0x7e) ? 1 : 0;
}

// get start address of a partition
static uint32_t getPartAddressIce40(uint8_t* b) {
    uint16_t index = 9;
    uint32_t partAddr;

    partAddr = b[index++];
    partAddr <<= 8;
    partAddr |= b[index++];
    partAddr <<= 8;
    partAddr |= b[index];
    return partAddr;
}

// Checks or creates ICE40 multi-boot partition table sector.
// The multi-boot partition table sector is located at flash address 0.
// There are 5 records, each is 32 byte of size.
// Records for partitons 0 to 3 are located at record 1 to 4 respectively.
// We use partitons that start at 32 kbyte boundaries to simplify erasure of the blocks.
// For more information check the 'icemulti' tool. The partition table we use here
// can be generated by 'icemulti' tool as follows:
// icemulti -A 15 -c -v p1.bin p2.bin p3.bin p4.bin -o multi15.bin
// When creating the partititon table the size of each partition is the same and is 
// just big enough to fit the file size passed in 'fileSizeBlocks' variable.
uint16_t checkOrCreateBootSectorIce40(uint8_t partIndex, uint16_t fileSizeBlocks, bool checkOnly) {
    bool r;
    uint16_t result = 0;
    uint8_t* b = rle_buffer;

    flash.powerUp();
    r = flash.readPage(0, b);
    flash.powerDown();
    if (!r) {
        return ERROR_FLASH_ACCESS;
    }
    // detect boot sector - if detected we never overwite it
    if (
        isFpgaMagicIce40(b) &&
        isFpgaMagicIce40(b + 0x20) &&
        isFpgaMagicIce40(b + 0x40) &&
        isFpgaMagicIce40(b + 0x60) &&
        isFpgaMagicIce40(b + 0x80)
    ) {
        // chech partition 
        uint32_t partAddr1, partAddr2;
        // partition records start at record 1, hence (partIndex + 1)
        partAddr1 = getPartAddressIce40(b + ((partIndex + 1) << 5)); // * 32: 32 bytes per partition record
        // check boundary alignment: low 14 bits must be zero to pass this check
        if (partAddr1 & 0x3FFF) {
            return ERROR_PARTITION_ALIGNMENT;
        }

        //Check partition size. Don't care about the size if writing to the last partition.
        if (partIndex < 3) {
            partAddr2 = getPartAddressIce40(b + ((partIndex + 2) << 5)); // * 32
            // Check boundary alignment: low 14 bits must be zero to pass this check.
            // This check is technically not required (if the next partition is never written to),
            // but for the sake of having a consistent multi-boot we enforce this check.
            // Please note that the multi boot sector might have been written by other tool than
            // niceprog, or could have been hand crafted in hexeditor and blasted to the first flash sector.
            if (partAddr2 & 0x3FFF) {
                return ERROR_PARTITION_ALIGNMENT;
            }

            // Sanity check that the next partition starts at higher address than the previous one.
            // Again, this is not technically required, but to avoid further issues let's not support it.
            if (partAddr2 < partAddr1) {
                return ERROR_INVALID_PARTITION;
            }

            if ( ((partAddr2 - partAddr1) >> 13) < fileSizeBlocks ) { // ">> 13" is to get the size in 16 kb blocks
                return ERROR_PARTITION_SMALL;
            }
        } else {
            // sanity check the start address of the last partition is after the previous partition
            partAddr2 = getPartAddressIce40(b + ((partIndex) << 5)); // * 32
            if (partAddr2 >= partAddr1) {
                return ERROR_INVALID_PARTITION;
            }
        }
        // all OK - return the partition start block
        return partAddr1 >> 14; // ">> 14" is to get the size in 16 kb blocks
    } else 
    // no boot sector found - create a new one
    if (!checkOnly) {
        uint8_t i;
        // convert from 16 kb blocks to 32 kb blocks
        uint32_t fileSizeIn32kbBlocks = (fileSizeBlocks + 1) >> 1;
        uint32_t partAddr = 0x8000;
        memset(b , 0x00,  0xA0);
        memset(b + 0xA0 , 0xFF, 0x60);
        // create 5 partition records.
        for (i = 0; i < 5; i++) {
            uint8_t a = i << 5; // *32
            // magic
            b[a++] = 0x7E; b[a++] = 0xAA; b[a++] = 0x99; b[a++] = 0x7E;
            // unknown
            b[a++] = 0x92, b[a++] = 0x00; b[a++] = (i) ? 0x00: 0x10; b[a++] = 0x44;
            // partition start address
            b[a++] = 0x03; b[a++] = (partAddr >> 16) & 0xFF; b[a++] = (partAddr >> 8) & 0xFF; b[a++] = 0;
            // unknown
            b[a++] = 0x82, b[a++] = 0x00; b[a++] = 0x00; b[a++] = 0x01; b[a] = 0x08;
            if (i > 0) {
                partAddr += (fileSizeIn32kbBlocks << 15); //convert to byte address
            }
        }
        flash.powerUp();
        //erase the block and then write it
        r = flash.eraseBlock32K(0);
        flash.powerDown();
        if (!r) {
           return ERROR_FLASH_ACCESS;
        }
        flash.powerUp();
        r = flash.writePage(0, b, false);
        flash.powerDown();
        if (!r) {
           return ERROR_FLASH_ACCESS;
        }
        //return the partition start in 16 kb blocks
        return (2 + ((fileSizeIn32kbBlocks << 1) * partIndex));
    } 

    // no boot sector found when checkOnly was requested
    return ERROR_NO_PARTITION;
}

bool executeStream(uint32_t streamSize) {
    uint8_t done;

    SPI.end(); //release SPI pins - we need to swap MOSI and MISO

    digitalWrite(PIN_LED_R, 1);
    digitalWrite(PIN_LED_G, 1);
    digitalWrite(PIN_LED_B, 0);
    delay(30);

    SPI.begin(PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS); // swap MISO and MOSI pins
    SPI.setDataMode(0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setFrequency(10000000);

    digitalWrite(PIN_CS, LOW); //SPI CS is asserted -> a signal for FPGA to act as a SPI slave
    digitalWrite(PIN_RESET, HIGH); // start FPGA loading from MCU
    delay(1);
    SPI.writeBytes(data_buffer, streamSize);
    delay(1);
    //Pin CDONE should indicate the upload finished
    done = digitalRead(PIN_CDONE);
    DEBUG(("CDONE: %d\r\n", cdone));

    return done == 1;
}

// write the whole uploaded buffer to flash
bool writeAll(uint32_t wrBlock) {
    uint32_t i;
    uint32_t wrAddress = wrBlock * MAX_CHUNK;
    uint32_t eraseAddress = (wrAddress >> 15) << 15; // align to 32k boundary
    uint8_t* buffer = data_buffer;
    bool result = false;

    wrAddress >>= 8; //Pages are 256 bytes long

    DEBUG(("write total=%d firstBlock=%d\r\n", uploadSize, wrBlock));

    flash.powerUp();

    DEBUG(("Erase addr=%06x\r\n", eraseAddress));

    // Special handling for MCU memory configurations where the upload buffer is less than 32 kbytes.
    // In that case we might be erasing portions of flash that were previously written to.
    if ((wrAddress & 0x7F) == 0) { // this check actually does: (wrBlock * MAX_CHUNK) & 0x7FFF;
        //erase the block before writing
        flash.eraseBlock32K(eraseAddress);
    }
    eraseAddress += (32 * 1024); //calculate next erase address;


    //write all bytes up to the uploadSize
    for (i = 0; i < uploadSize;) {
        result = flash.writePage(wrAddress, buffer, false); 
        DEBUG(("write addr=%06x : %02x %02x %02x %02x \r\n", wrAddress << 8, buffer[0], buffer[1], buffer[2], buffer[3]));
        if (!result) {
            goto finish;
        }
        i += PAGESIZE;
        buffer += PAGESIZE;
        wrAddress++;

        // check whether we need to erase another 32kb flash block
        if (eraseAddress == (wrAddress << 8) && i < uploadSize) {
            DEBUG(("Erase addr=%06x\r\n", eraseAddress));
            flash.eraseBlock32K(eraseAddress);
            eraseAddress += (32 * 1024); //calculate next erase address;
        }
    }
    result = true;
finish:
    flash.powerDown();
    return result;
}

// verify the whole uploaded buffer against the flash contents
bool verifyAll(uint32_t rdBlock, uint32_t* errors) {
    uint32_t i, j;
    uint32_t rdAddress = rdBlock * MAX_CHUNK;
    uint8_t* buffer = rle_buffer; //reuse rle_buffer for reading data from flash
    uint32_t errorCount = 0;
    bool result = false;

    rdAddress >>= 8; //Pages are 256 bytes long

    DEBUG(("verify total=%d\r\n", uploadSize));

    flash.powerUp();

    //verify all bytes up to the uploadSize
    for (i = 0; i < uploadSize;) {
        result = flash.readPage(rdAddress, buffer);
        DEBUG(("read addr=%d, i=%d, %02x %02x %02x %02x\r\n", rdAddress, i, buffer[0], buffer[1], buffer[2], buffer[3]));
        if (!result) {
            goto finish;
        }
        // verify byte contents
        for (j = 0; j < PAGESIZE; j++) {
            if (buffer[j] != data_buffer[i++]) {
                errorCount++;
            }
        }
        rdAddress++;
    }
    *errors = errorCount;
    DEBUG(("error count=%d\r\n", errorCount));
    result = (errorCount == 0);
finish:
    flash.powerDown();
    return result;
}

// Use eraseAll only if you need to clear the flash contents.
// If you need to write some data, the write function erases
// the required flash block for you automatically, so there is no need
// to erase the whole chip before writing.
bool eraseAll(void)
{
    uint32_t i;
    bool result;

    // for some reason the bulk erase does not work proeprly in the SPIFlash library
    // but block erase works fine. 

    // lowest byte determines the total flash size
    uint32_t totalSize = chipId & 0xFF;
    totalSize = 1 << totalSize;
    for (i = 0; i < totalSize; ) {
        flash.powerUp();
        result = flash.eraseBlock64K(i);
        i += 64 * 1024;
        flash.powerDown();
        //delay(100);
        DEBUG(("Erase 64 sector: %08x result=%d\r\n", i, result));
    }
    return result;
}

// reads a single flash block of MAX_CHUNK size
static void readBlock(uint16_t block,  uint32_t bufAddr) {
    int i, max;
    uint32_t pageAddr = block;
    pageAddr *= MAX_CHUNK;
    pageAddr >>= 8;

    for (i = 0; i < MAX_CHUNK; i++) {
        flash.readPage(pageAddr, data_buffer + bufAddr);
        DEBUG(("%06x : %02x %02x %02x %02x \r\n", bufAddr, data_buffer[bufAddr], data_buffer[bufAddr+1], data_buffer[bufAddr+2], data_buffer[bufAddr+3]));
        bufAddr += 256;
        i += 256;
        pageAddr ++;
    }
}

// reads multiple flash blocks
void readBlocks(uint16_t block, uint16_t cnt, uint32_t bufAddr) {
    int i;

    flash.powerUp();
    for (i = 0; i < cnt; i++) {
        readBlock(block + i, bufAddr);
        bufAddr += MAX_CHUNK;
    }
    flash.powerDown();
}



