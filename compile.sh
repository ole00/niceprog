
GCOM=`git  rev-parse --short HEAD`


gcc -g2 -O0 -Wno-format-contains-nul -DNO_CLOSE -DGCOM="\"g${GCOM}\"" -o niceprog src_pc/niceprog.c src_pc/crc16.c
