# path to your Win64 cross-compiler
CC=~/opt/mingw64/bin/x86_64-w64-mingw32-gcc

GCOM=`git  rev-parse --short HEAD`

$CC -g3 -O0  -o niceprog.exe src_pc/niceprog.c src_pc/crc16.c -D_USE_WIN_API_ -DNO_CLOSE -DGCOM="\"g${GCOM}\""

