#include "DebugOut.h"
#include <pic18f26j50.h>

void dbgPrintByte(unsigned char byte) {
    unsigned char upperNib = ((byte & 0x70) >> 4) | ((byte & 0x80) >> 1);
    unsigned char lowerNib = (byte & 0x07) | ((byte & 0x08) << 3);
    LATC = 0x47; // output 0xF
    LATC = upperNib;
    LATC = lowerNib;
    LATC = 0x47;
    LATC = 0;
}
