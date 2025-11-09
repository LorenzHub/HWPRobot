#include "remoteDataProcessing.h"
#include <communication/communication.h>

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>


static bool rdp_command(const uint16_t maxLength, const bool execute, const char* command, va_list argp) {
    uint16_t len = maxLength + 1;

    char *buff = (char*)malloc(len + 1);
    if (buff == 0)
        return false;
    buff[0] = execute ? 0x02 : 0x01;

    int size = vsnprintf(&buff[1], len, command, argp);

    communication_writePacket(CH_OUT_RDP, (uint8_t*)buff, (uint16_t)size >= len ? len : ((uint16_t)size+1));
    free(buff);

    return true;
}


bool remoteDataProcessing_command(const bool execute, const char* command, ...) {
    va_list argp;
    va_start(argp, command);

    bool result = rdp_command(512, execute, command, argp);

    va_end(argp);

    return result;
}


bool remoteDataProcessing_command_maxLength(const uint16_t maxLength, const bool execute, const char* command, ...) {
    va_list argp;
    va_start(argp, command);

    bool result = rdp_command(maxLength, execute, command, argp);

    va_end(argp);

    return result;
}


void remoteDataProcessing_clear(void) {
    char buff = 0x00;
    communication_writePacket(CH_OUT_RDP, (uint8_t*)&buff, 1);
}
