# path to your Osx cross-compiler (https://github.com/tpoechtrager/osxcross)
export PATH=~/dev/osxcross2/bin:$PATH
CC=x86_64-apple-darwin24.4-clang

GCOM=`git  rev-parse --short HEAD`


$CC -g3 -O0 -D_OSX_ -DNO_CLOSE -DGCOM="\"g${GCOM}\"" -o niceprog_osx_x86  src_pc/niceprog.c src_pc/crc16.c
