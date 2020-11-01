# Falk6502
A 6502 emulation WIP.

I build with:

`g++ -lncurses -lrt -lpthread -std=c++20 -Wall -Wformat=2 -O3 ./src/6502.cc -o ./bin/emu.bin`

Arg 1: binary filename to load (if not provided, a temporary demo program will be loaded)
Arg 2: address to try to load the binary into (0~65535)
Arg 3: target Hz  (default 500)
Arg 4: target FPS (default 30)

Very early WIP!

