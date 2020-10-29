# Falk6502
A 6502 emulation WIP.

I build with:

`g++ -lrt -lpthread -std=c++20 -Wall -Wformat=2 -O3 ./src/6502.cc -o ./bin/emu.bin`

First argument is the target clock speed in Hz, the second argument is the target frame rate.

`./bin/emu.bin 233 30` would try to run it at 233Hz at 30 FPS.

Very early WIP!
