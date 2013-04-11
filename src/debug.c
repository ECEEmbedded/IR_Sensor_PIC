#include "debug.h"

void DebugPrint(unsigned char out) {
    //Convert any numbers to just 1 or 0

    LATC = (out & 0x07) | ((out & 0x08) << 3);
}

void DebugPrintByte(unsigned char out) {
    unsigned char tmpH = ((out & 0x70) >> 4) | ((out & 0x80) >> 1);
    unsigned char tmpL = (out & 0x07) | ((out & 0x08) << 3);
    LATC = 0x47;
    LATC = tmpH;
    LATC = 0x47;
    LATC = tmpL;
    LATC = 0x47;
    LATC = 0x00;
}

void DebugPrintString(unsigned char *msg, unsigned char len) {
    char i;
    for (i=0;i<len;++i){
        DebugPrintByte(msg[i]);
    }
}