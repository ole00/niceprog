/*
FLASH SPI NOR MEmory via serial port
*/

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include "serial_port.h"
#include "crc16.h"

#define VERSION "v.1.0"

#ifdef GCOM
#define VERSION_EXTENDED VERSION "-" GCOM
#else
#define VERSION_EXTENDED VERSION
#endif

#define MAX_FILE_SIZE (128 * 1024 * 2024)
#define MAX_CHUNK (16*1024)
#define MAX_LINE 256

#define ERROR_INVALID_PARTITION    0xFFEB
#define ERROR_NO_PARTITION         0xFFEC
#define ERROR_FLASH_ACCESS         0xFFFD
#define ERROR_PARTITION_ALIGNMENT  0xFFFE
#define ERROR_PARTITION_SMALL      0xFFFF

char verbose = 0;
char* filename = NULL;
char* deviceName = NULL;

static unsigned char* fileBuffer = NULL;
SerialDeviceHandle serialF = INVALID_HANDLE;

char opRead = 0;
char opWrite = 0;
char opErase = 0;
char opInfo = 0;
char opVerify = 0;

char printSerialWhileWaiting = 0;
int argStartBlock = -1;
int argBlockCount = -1;
int argBootIndex = -1;

char noProgressBar = 0;

static int waitForSerialPrompt(char* buf, int bufSize, int maxDelay);
char sendGenericCommand(const char* command, const char* errorText, int maxDelay, char printResult);

typedef struct {
    uint8_t isFpga;          // indicates whether the file is an FPGA bit stream
    uint8_t hasMultiBoot;    // indicates whether the FPGA bitstream starts with multiboot table
} MultiBootInfo;

MultiBootInfo bootInfo = {0};

static void printHelp() {
    printf("niceprog " VERSION_EXTENDED "  a flash memory programming tool for Arduino based programmer\n");
    printf("more info: https://github.com/ole00/niceprog\n");
    printf("usage: niceprog command(s) [options]\n");
    printf("commands: ierwv\n");
    printf("   i : read device info\n");
    printf("   r : read flash contents from the memory chip, -f option must be set\n");
    printf("   w : write a file to flash chip, -f  option must be set\n");
    printf("   v : verify flash file, -f option must be set\n");
    printf("   e : erase the whole flash chip\n");
        printf("options:\n");
    printf("  -bs value : set starting block (1 block is 16kbytes)\n");
    printf("  -bc value : set block count (1 block is 16kbytes)\n");
    printf("  -boot value : use boot partition for ICE40 cold boot. Use 0 to 3 as the value.\n");
    printf("                Applicable to 'w' and 'v' commands only.\n");
    printf("                The -boot parameter is disregarded if -bs parameter is also set.\n");
    printf("  -v : verbose mode\n");
    printf("  -f file : a binary file to read, write or verify\n");
    printf("  -d serial_device : name of the serial device. Without this option the device is guessed.\n");
    printf("                       serial params are: 115200, 8N1\n");
    printf("  -nobar : do not show progress bar\n");
    printf("examples:\n");
    printf("  niceprog i : prints the flash device id\n");
    printf("  niceprog r -f data.bin : reads the whole flash chip and store it to data.bin file\n");
    printf("  niceprog wv -f data.bin : reads data.bin file and writes it to \n");
    printf("              the flash chip. Does the contents verification at the end.\n");
    printf("              Flash blocks are erased automatically before writing operation.\n");
}

static int8_t verifyArgs() {
    if (!opRead && !opWrite && !opErase && !opInfo && !opVerify) {
        printHelp();
        printf("Error: no command specified.\n");
        return -1;
    }
    if (opRead && opWrite ) {
        printf("Error: invalid command combination. Use either read or write operation in a separate step\n");
        return -1;
    }
    if (0 == filename && (opWrite || opVerify)) {
        printf("Error: missing a filename (param: -f fname)\n");
        return -1;
    }
   return 0;
}

static int8_t checkArgs(int argc, char** argv) {
    int i;
    char* type = 0;
    char* modes = 0;

    for (i = 1; i < argc; i++) {
        char* param = argv[i];
        if (strcmp("-t", param) == 0) {
            i++;
            type = argv[i];
        } else if (strcmp("-v", param) == 0) {
            verbose = 1;
        } else if (strcmp("-f", param) == 0) {
            i++;
            filename = argv[i];
        } else if (strcmp("-d", param) == 0) {
            i++;
            deviceName = argv[i];
        } else if (strcmp("-bs", param) == 0) {
            i++;
            argStartBlock = atoi(argv[i]);
        } else if (strcmp("-bc", param) == 0) {
            i++;
            argBlockCount = atoi(argv[i]);
        } else if (strcmp("-nobar", param) == 0) {
            noProgressBar = 1;
        } else if (strcmp("-boot", param) == 0) {
            i++;
            argBootIndex = atoi(argv[i]);
            if (argBootIndex < 0 || argBootIndex > 3) {
                printf("Error: unsupported boot index %d\n", argBootIndex);
                return -1;
            }
        } 
        else if (param[0] != '-') {
            modes = param;
        }
    }

    i = 0;
    while (modes != 0 && modes[i] != 0) {
        switch (modes[i]) {
        case 'r':
            opRead = 1;
            break;
        case 'w':
            opWrite = 1;
            break;
        case 'v':
            opVerify = 1;
            break;
        case 'e':
            opErase = 1;
            break;
        case 'i':
            opInfo = 1;
            break;
        default:
            printf("Error: unknown operation '%c' \n", modes[i]);
        }
        i++;
    }

    if (verifyArgs()) {
        return -1;
    }

    return 0;
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
unsigned short parse2hex(char* line) {
    int i = 0;
    if (line[i] == '\r' || line[i] ==  0 || line[i] == ' ') {
        return -1; 
    }

    unsigned short v = toHex(line[i++]) << 4;
    return v + toHex(line[i]); 
}

// Parses hexdecimal integer number typed as 4 digit.
// Returns the number value.
unsigned short parse4hex(char* line) {
    int i = 0;
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

static char readFile(int* fileSize) {
    FILE* f;
    int size;

    if (verbose) {
        printf("opening file: '%s'\n", filename);
    }
    if (fileBuffer != NULL) {
        free(fileBuffer);
    }
    fileBuffer = (unsigned char*) malloc(MAX_FILE_SIZE);
    if (NULL == fileBuffer) {
        printf("Error: failed to allocate file buffer\n");
        return -1;
    }
    f = fopen(filename, "rb");
    if (f) {
        size = fread(fileBuffer, 1, MAX_FILE_SIZE, f);
        fclose(f);
    } else {
        printf("Error: failed to open file: %s\n", filename);
        return -1;
    }
    if (fileSize != NULL) {
        //pad the buffer with 0xFF
        memset(fileBuffer + size, 0xFF, MAX_FILE_SIZE - size > MAX_CHUNK ? MAX_CHUNK : MAX_FILE_SIZE - size);
        if (verbose) {
            printf("file size: %d'\n", size);
        }
        // Adjust the size to 256 byte boundary, which is the flash PAGESIZE
        // This is not scrictly required, but it ensures there is no garbage at the end of unaligned files
        *fileSize = ((size + 255) >> 8) << 8;
    }
    return 0;
}

static char writeFile(int fileSize) {
    FILE* f;
    int size;

    if (verbose) {
        printf("opening file: '%s'\n", filename);
    }
    if (fileBuffer == NULL) {
        printf("Error: file buffer not allocated\n");
        return -1;
    }

    f = fopen(filename, "wb");
    if (f) {
        size = fwrite(fileBuffer, 1, fileSize == 0 ? MAX_FILE_SIZE : fileSize, f);
        fclose(f);
    } else {
        printf("Error: failed to open file: %s\n", filename);
        return -1;
    }
    return 0;
}

static char checkForString(char* buf, int start, const char* key) {
    int labelPos = strstr(buf + start, key) -  buf;
    return (labelPos > 0 && labelPos < 500) ? 1 : 0;
}

static int openSerial(void) {
    char buf[512] = {0};
    char devName[256] = {0};
    int total;
    int labelPos;
    int retry = 4;


    //open device name
    if (deviceName == 0) {
        serialDeviceGuessName(&deviceName);
    }
    snprintf(devName, sizeof(devName), "%s", (deviceName == 0) ? DEFAULT_SERIAL_DEVICE_NAME : deviceName);
    serialDeviceCheckName(devName, sizeof(devName));

    if (verbose) {
        printf("opening serial: %s\n", devName);
    }

    while (retry) {
        retry--;
        serialF = serialDeviceOpen(devName);
        if (serialF == INVALID_HANDLE) {
            printf("Error: failed to open serial device: %s\n", devName);
            return -2;
        }
#ifndef _USE_WIN_API_
        //read garbage
        total = waitForSerialPrompt(buf, 512, 4);
#endif

        // prod the programmer to output it's identification
        // also switches the programmer from IDLE mode to FLASH mode
        sprintf(buf, "#*\t*nice\tprog#\r");
        serialDeviceWrite(serialF, buf, 15);

        //read programmer's message
        total = waitForSerialPrompt(buf, 512, 200);
        buf[total] = 0;

        //check we are communicating with Niceprog programmer
        labelPos = strstr(buf, "niceprog v.") -  buf;

        if (labelPos >= 0 && labelPos < 500 && buf[total - 3] == '>') {
            //all OK
            return 0;
        }
        if (verbose) {
            printf("Output from programmer not recognised (%d): %s\n", labelPos, buf);
            printf("--------------\n");
        }
        serialDeviceClose(serialF);
        serialF = INVALID_HANDLE;
    }
    return -4;
}

static void closeSerial(void) {
    if (INVALID_HANDLE == serialF) {
        return;
    }
    serialDeviceClose(serialF);
    serialF = INVALID_HANDLE;
}


static int checkPromptExists(char* buf, int bufSize) {
    int i;
    for (i = 0; i < bufSize - 2 && buf[i] != 0; i++) {
        if (buf[i] == '>' && buf[i+1] == '\r' && buf[i+2] == '\n') {
            return i;
        }
    }
    return -1;
}

static char* stripPrompt(char* buf) {
    int len;
    int i;
    if (buf == 0) {
        return 0;
    }
    len = strlen(buf);
    i  = checkPromptExists(buf, len);
    if (i >= 0) {
        buf[i] = 0;
        len = i;
    }

    //strip rear new line characters
    for (i = len - 1; i >= 0; i--) {
        if (buf[i] != '\r' && buf[i] != '\n') {
            break;
        } else {
            buf[i] = 0;
        }
    }

    //strip frontal new line characters
    for (i = 0; buf[i] != 0; i++) {
        if (buf[0] == '\r' || buf[0] == '\n') {
            buf++;
        }
    }
    return buf;
}

//finds beginnig of the last line
static char* findLastLine(char* buf) {
    int i;
    char* result = buf;

    if (buf == 0) {
        return 0;
    }
    for (i = 0; buf[i] != 0; i++) {
        if (buf[i] == '\r' || buf[i] == '\n') {
            result = buf + i + 1;
        }
    }
    return result;
}

static char* printBuffer(char* bufPrint, int readSize) {
    int i;
    char doPrint = 1;
    for (i = 0; i < readSize;i++) {
        if (*bufPrint == '>') {
            doPrint = 0;
        }
        if (doPrint) {
            printf("%c", *bufPrint);
            if (*bufPrint == '\n' || *bufPrint == '\r') {
                fflush(stdout);
            }
            bufPrint++;
        }
    }
    return bufPrint;
}

static int waitForSerialPrompt(char* buf, int bufSize, int maxDelay) {
    char* bufStart = buf;
    int bufTotal = bufSize;
    int bufPos = 0;
    int readSize;
    char* bufPrint = buf;
    char doPrint = printSerialWhileWaiting;
    
    memset(buf, 0, bufSize);

    while (maxDelay > 0) {
        readSize = serialDeviceRead(serialF, buf, bufSize);
        if (readSize > 0) {
            bufPos += readSize;
            if (checkPromptExists(bufStart, bufTotal) >= 0) {
                maxDelay = -4; //force exit, but read the rest of the line
            } else {
                buf += readSize;
                bufSize -= readSize;
                if (bufSize < 0) {
                    printf("ERROR: serial port read buffer is too small!\nAre you dumping large amount of data?\n");
                    return -1;
                }
            }
            if (printSerialWhileWaiting) {
                bufPrint = printBuffer(bufPrint, readSize);
            }
        }
        if (maxDelay > 0) {
        /* WIN_API handles timeout itself */
#ifndef _USE_WIN_API_
            usleep(10 * 1000);
            maxDelay -= 10;
#else
            maxDelay -= 30;
#endif
            if(maxDelay <= 0 && verbose) {
                printf("waitForSerialPrompt timed out\n");
            }
        }
    }
    return bufPos;
}

static int sendBuffer(unsigned char* buf, int total) {
    int writeSize;
    int retry = 5;

    if (buf == 0) {
        return -1;
    }
    //printf("Send buffer %d\n", total);
    // write the buffer to the serial port's file
    // file is opened non blocking so we have to ensure all contents is written
    while (total > 0) {
        writeSize = serialDeviceWrite(serialF, buf, total);
        if (writeSize < 0) {
            writeSize = 0;
            retry --;
            if (retry == 0) {
                printf("ERROR: written: %i (%s)\n", writeSize, strerror(errno));
                return -4;
            } else {
                usleep(10 * 1000);
            }
        } else if (writeSize > 0) {
            //printf("written: %d / %d\n", writeSize, total);
            retry = 5;
        }
        buf += writeSize;
        total -= writeSize;
    }
    return 0;
}

static int sendLine(char* buf, int bufSize, int maxDelay) {
    int total;
    char* obuf = buf;

    if (serialF == INVALID_HANDLE) {
        if (verbose) {
            printf("Error: serial port is not opened\n");
        }
        return -1;
    }

    total = sendBuffer(buf, strlen(buf));
    //exit on failure
    if (total) {
        return total;
    }

    total = waitForSerialPrompt(obuf, bufSize, (maxDelay < 0) ? 6 : maxDelay);
    if (total < 0) {
        return total;
    }
    obuf[total] = 0;
    obuf = stripPrompt(obuf);
    if (verbose) {
        printf("read: %i '%s'\n", total, obuf);
    }

    return total;
}

static void updateProgressBar(char* label, int current, int total) {
    int done = ((current + 1) * 40) / total;
    if (noProgressBar) {
        return;
    }

    if (current >= total) {
        printf("%s%5d/%5d |########################################|\n", label, total, total);
    } else {
        printf("%s%5d/%5d |", label, current, total);
        printf("%.*s%*s|\r", done, "########################################", 40 - done, "");
        fflush(stdout); //flush the text out so that the animation of the progress bar looks smooth
    }
}

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

// returns the size of RLE encoded buffer or -1 if RLE is bigger than source buffer length
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

// Upload file (or partial file) to Arduino
static char upload(char* uploadBuffer, int uploadSize, int progressStart, int totalFileSize) {
    char fuseSet;
    char buf[MAX_LINE];
    unsigned char rle[MAX_CHUNK];
    unsigned char test[MAX_CHUNK];
    char line[64];
    unsigned int i;
    unsigned char* dataBuff; //buffer to send
    int sendLen;
    int r;

    // Start  upload
    if (verbose) {
        printf("Uploading file... uplSize=%d\n", uploadSize);
    }
    // send number of chunks
    sprintf(buf, "#u%04X\r", (uploadSize  + MAX_CHUNK - 1 ) / MAX_CHUNK);
    sendLine(buf, MAX_LINE, 8000);

    for (i = 0; i < uploadSize;) {
        unsigned char f = 0;
        if (verbose) {
            printf("upload buffer: %02x %02x %02x %02x\n",
                (uint8_t) uploadBuffer[i], (uint8_t)uploadBuffer[i+1], (uint8_t)uploadBuffer[i+2], (uint8_t)uploadBuffer[i+3]);
        }
        int rleSize = encodeRle(rle, uploadBuffer + i, MAX_CHUNK);
        // sanity check - RLE verification
        if (rleSize > 1) {
            int j;
            unsigned char* dst = uploadBuffer + i;
            //printf("RLE: %d\n\n", rleSize);
            decodeRle(test, rle, rleSize);
            for (j = 0; j < MAX_CHUNK; j++) {
                if (test[j] != dst[j]) {
                    printf("!!! ERROR: RLE decode failed at %d\n\n", j);
                    j = MAX_CHUNK;
                }
            }
            dataBuff = rle;
            sendLen = rleSize;
        } else {
            dataBuff = uploadBuffer + i;
            sendLen = MAX_CHUNK;
        }
        //calculate crc
        crc_t crc = crc_update(0, dataBuff, sendLen);
        sprintf(buf, "#c%04X%04X\r", crc, sendLen);

        r = sendLine(buf, MAX_LINE, 8000);
        if (r  < 1) {
            printf("Error: timed out at file offset: %d\n", i);
            return -1;
        }
        //printf("sending chunk...(r=%d) crc=%04x\n", r, crc);
        if (0 != sendBuffer(dataBuff, sendLen)) {
            printf("Error: failed to upload file at offset: %d\n", i);
            return 1;
        }
        i += MAX_CHUNK;

        if (!verbose) {
            updateProgressBar("Upload: ", progressStart + i, totalFileSize);
        }
    }
    //updateProgressBar("", fileSize, fileSize);
    return 0;
}

static char downloadBlock(int block, int memBlockOffset ) {
    char buf[MAX_LINE];
    int r;
    int total = MAX_CHUNK;
    int readSize;
    int bufPos = (block + memBlockOffset) * MAX_CHUNK;;
    char* response;
    crc_t passedCrc;
    crc_t crc;

    //printf("downloading block %d, physical block %d\n", block, block + memBlockOffset);
    sprintf(buf, "#d%04X\r", block);
    r = sendLine(buf, 12, 8000);
    if (r  < 1) {
        printf("Error: download timed out\n");
        return -1;
    }
    response = stripPrompt(buf);
    passedCrc = parse4hex(response + 3);
    //printf("resp: '%s'\n", response); fflush(0);

    while (total > 0) {
        readSize = serialDeviceRead(serialF, fileBuffer + bufPos, total > 1024 ? 1024 : total);
        if (readSize > 0) {
            //printf("Read %d, total=%d\n", readSize, total); fflush(0);
            bufPos += readSize;
            total -= readSize;
        } else {
            usleep(5 * 1000);
        }
    }
    crc = crc_update(0, fileBuffer + bufPos - MAX_CHUNK, MAX_CHUNK);
    //printf("block read finished crc:%04x\n", crc);
    bufPos -= MAX_CHUNK;
    //printf("%06x : %02x %02x %02x %02x \r\n\n", bufPos, fileBuffer[0], fileBuffer[1], fileBuffer[2], fileBuffer[3]);
    buf[0] = 0;
    waitForSerialPrompt(buf, MAX_LINE, 2000);
    if (passedCrc != crc) {
        printf("Error: crc does not match. expected=%04x calculated=%04x\n", passedCrc, crc);
        return -1;
    }
    return 0;
}

//returns 0 on success
char sendGenericCommand(const char* command, const char* errorText, int maxDelay, char printResult) {
    char buf[MAX_LINE];
    int readSize;

    sprintf(buf, "%s", command);
    readSize = sendLine(buf, MAX_LINE, maxDelay);
    if (readSize < 0)  {
        if (verbose) {
            printf("%s\n", errorText);
        }
        return -1;
    } else {
        char* response = stripPrompt(buf);
        char* lastLine = findLastLine(response);
        if (lastLine == 0 || (lastLine[0] == 'E' && lastLine[1] == 'R')) {
            printf("%s\n", response);
            return -1;
        } else if (printResult && printSerialWhileWaiting == 0) {
            printf("%s\n", response);
        }
    }
    return 0;
}

static int getChipTotalBlocks() {
    char buf[MAX_LINE];
    int readSize;
    char* response;
    int i;
    int blocks;

    // read JEDEC info to determine flash size
    sprintf(buf, "#i\r");
    readSize = sendLine(buf, MAX_LINE, 3000);
    if (readSize < 0) {
        printf("Error: failed to identify chip\n");
        return -1;
    }
    response = stripPrompt(buf);
    i = parse2hex(response + 14);
    i = 1 << i;
    blocks = i / MAX_CHUNK;
    if (verbose) {
        printf("Chip size: %d mbytes. Chip block count: %d\n", i / (1024*1024), blocks);
    }
    return blocks;
}

static char isFpgaMagic(uint8_t* b) {
    return (b[0] == 0x7E &&  b[1] == 0xAA && b[2] == 0x99 && b[3] == 0x7e) ? 1 : 0;
}

static int32_t get3int(uint8_t* b) {
    uint32_t b1 = b[1];
    uint32_t b2 = b[2];
    uint32_t b3 = b[3];
    return (b1 << 16) |  (b1 << 8) | b3;
}

static void detectMultiBoot() {
    uint8_t* b = fileBuffer;
    int i;

    memset(&bootInfo, 0, sizeof(MultiBootInfo));

    // check FPGA stream (Lattice)
    // it seems the magic can be on any 4 byte address?
    for (i = 0;  i < 256; i+= 4) {
        if (isFpgaMagic(b + i)) {
            //bootInfo.fileFpgaStreamAddr = i;
            bootInfo.isFpga = 1;
            i = 256;
        }
    }

    // check multi boot
    if ( 
        bootInfo.isFpga == 1 &&
        isFpgaMagic(b + 32) &&
        isFpgaMagic(b + 64) &&
        isFpgaMagic(b + 96) &&
        isFpgaMagic(b + 128)
    ) {
        bootInfo.hasMultiBoot = 1;
    }
}

static char operationWriteOrVerify(char doWrite) {
    char buf[MAX_LINE];
    int readSize;
    int blocks;
    char result = -1;
    int firstBlock = 0;
    int fileSizeInBlocks;
    int fileBufferPos = 0;
    char* response;
    uint32_t partBlock;
    int r;

    if (readFile(&readSize)) {
        return -1;
    }
    if (readSize < 1) {
        return -1;
    }

    if (openSerial() != 0) {
        return -1;
    }

    blocks = getChipTotalBlocks();

    // check whether the file has multiboot installed
    detectMultiBoot();

    fileSizeInBlocks = ((readSize + MAX_CHUNK - 1) / MAX_CHUNK);


    if (verbose) {
        printf("file is ice40 stream: %s\n",  bootInfo.isFpga ? "yes" : "no");
    }

    // Detect multiboot on the flash chip - only when start block is not specified
    // and when the file does not start with multiboot table and when the file is 
    // an FPGA bitstream. 
    if (
        argStartBlock < 0 &&           // start block was not explicitely specified
        bootInfo.isFpga &&             // when using FPGA bitstrem file
        (!bootInfo.hasMultiBoot) &&    // FPGA bitstream file must not have the partition table present
        argBootIndex >= 0               // User asked to use the boot partition index
    ) {
        // detect-and-create: B
        // detect-only: b (when doing verification without writing)
        sprintf(buf, "#%c%02x%04x\r", doWrite ? 'B' : 'b', argBootIndex, fileSizeInBlocks);
        r = sendLine(buf, 12, 2000);
        if (r  < 1) {
            printf("Error: boot detect timed out\n");
            goto finish;
        }
        response = stripPrompt(buf);
        if (verbose) {
            printf("boot detect resp: %s\n", response);
        }
        partBlock = parse4hex(response + 3); // partition starting block in 16 kb blocks
        if (partBlock == ERROR_INVALID_PARTITION) {
            printf("Error: invalid partition table was found.\n");
            goto finish;
        }
        if (partBlock == ERROR_NO_PARTITION) {
            printf("Error: no partition was found.\n");
            goto finish;
        }
        if (partBlock == ERROR_FLASH_ACCESS) {
            printf("Error: failed to access flash.\n");
            goto finish;
        }
        if (partBlock == ERROR_PARTITION_ALIGNMENT) {
            printf("Error: partition table exists, but block alignment is not compatible.\n");
            goto finish;
        }
        if (partBlock == ERROR_PARTITION_SMALL) {
            printf("Error: partition table exists, but partiton is too small.\n");
            goto finish;
        }
        // start writing / verifying at the partition block
        firstBlock = partBlock;
    }

    if (argStartBlock >= 0) {
        if (argStartBlock >= blocks) {
            printf("Error: start block is past the chip size\n");
            goto finish;
        }
        firstBlock = argStartBlock;
    }

    if (argBlockCount > 0) {
        if (argBlockCount > fileSizeInBlocks)  {
            argBlockCount = fileSizeInBlocks;
        }
    } else {
        argBlockCount = fileSizeInBlocks;
    }
    if (firstBlock + argBlockCount > blocks) {
        printf("Error: block count spans past the chip size\n");
        goto finish;
    }
    blocks = argBlockCount;

    if (verbose) {
        printf("first block=%d total blocks=%d\n", firstBlock, blocks);
    }

    while (blocks) {
        // upload up to 1Mbyte
        int b = blocks > 64 ? 64 : blocks;
        int uploadSize = (b == 64) ? 64 * MAX_CHUNK : readSize - fileBufferPos;

        result = upload(fileBuffer + fileBufferPos, uploadSize, fileBufferPos, readSize );
        if (result < 0) {
            return result;
        }

        if (verbose) {
            printf("write/verify at block: %d\n",  firstBlock + (fileBufferPos / MAX_CHUNK));
        }

        // write command
        if (doWrite) {
            sprintf(buf, "#w%04x\r", firstBlock + (fileBufferPos / MAX_CHUNK));
            result = sendGenericCommand(buf, "write failed ?", 8000, 0);
            if (result) {
                goto finish;
            }
        }

        // verify command
        if (opVerify) {
            sprintf(buf, "#v%04x\r", firstBlock + (fileBufferPos / MAX_CHUNK));
            result = sendGenericCommand(buf, "verify failed ?", 8000, 0);
            if (result) {
                goto finish;
            }
        }
        fileBufferPos += (b * MAX_CHUNK);
        blocks -= b;
    }
finish:
    closeSerial();
    return result;
}


static char operationReadInfo(void) {

    char result;

    if (openSerial() != 0) {
        return -1;
    }

    if (verbose) {
        printf("sending 'i' command...\n");
    }
    result = sendGenericCommand("#i\r", "info failed ?", 4000, 1);

    closeSerial();
    return result;
}

static char operationErase(void) {
    char buf[MAX_LINE];
    int readSize;
    char result;

    if (openSerial() != 0) {
        return -1;
    }
    if (verbose) {
        printf("sending 'e' command...\n");
    }
    result = sendGenericCommand("#e\r", "erase failed ?", 16000, 0);

    closeSerial();
    return result;
}

static char operationReadFlash(void) {
    char buf[MAX_LINE];
    int readSize;
    char* response;
    char result = -1;
    int i;
    int firstBlock = 0;
    int blocks;
    int totalBlocks;

    if (openSerial() != 0) {
        return -1;
    }
    if (NULL == fileBuffer) {
        fileBuffer = (unsigned char*) malloc(MAX_FILE_SIZE);
        if (NULL == fileBuffer) {
            printf("Error: file malloc failed\n");
            return -1;
        }
    }

    blocks = getChipTotalBlocks();


    totalBlocks = blocks;
    if (argStartBlock >= 0) {
        if (argStartBlock >= totalBlocks) {
            printf("Error: start block is past the chip size\n");
            goto finish;
        }
        firstBlock = argStartBlock;
        totalBlocks -= argStartBlock;
    }

    if (argBlockCount > 0) {
        if (firstBlock + argBlockCount > blocks) {
            printf("Error: block count spans past the chip size\n");
            goto finish;
        }
        blocks = argBlockCount;
        totalBlocks = blocks;
    }

    while (blocks) {
        // read up to 1Mbyte
        int b = blocks > 64 ? 64 : blocks;
        //READ command
        sprintf(buf, "#r%04x%04x\r", firstBlock, b);
        readSize = sendLine(buf, MAX_LINE, 22000);
        if (readSize < 0)  {
            return -1;
        }
        for (i = 0; i < b; i++) {
            result = downloadBlock(i, totalBlocks-blocks);
            updateProgressBar("Read  : ", (totalBlocks - blocks + i + 1) * MAX_CHUNK, totalBlocks * MAX_CHUNK);
        }
        firstBlock += b;
        blocks -= b;
    }
    writeFile(totalBlocks * MAX_CHUNK);
    result = 0;
finish:
    closeSerial();
    return result;
}




int main(int argc, char** argv) {
    char result = 0;
    int i;

    result = checkArgs(argc, argv);
    if (result) {
        return result;
    }
    if (verbose) {
        printf("niceprog " VERSION " \n");
    }


    if (opErase) {
        result = operationErase();
    }

    if (0 == result) {
        if (opWrite) {
            // writing fuses and optionally verification
            result = operationWriteOrVerify(1);
        } else if (opInfo) {
            result = operationReadInfo();
        } else if (opRead) {
            result = operationReadFlash();
        } else if (opVerify) {
            // verification without writing
            result = operationWriteOrVerify(0);
        }
    }

finish:
    if (verbose) {
        printf("result=%i\n", (char)result);
    }
    return result;
}
