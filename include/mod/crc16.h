#ifndef _CRC16_
#define _CRC16_

#include <stdint.h>
#include <string.h>

inline uint16_t fletcher16(unsigned char* buf, size_t buflen)
{
    // printf("Computing Checksum with function!\n");
    //Compute Fletcher16 CRC
    uint8_t chksm0 = 0, chksm1 = 0;
    for(unsigned i = 0; i < buflen; ++i)
    {
        chksm0 += buf[i];
        chksm1 += chksm0;
        // printf("chksm0: %02x, chksm1: %02x \n", chksm0, chksm1);
    }
    
    return (uint16_t) ((chksm1 << 8) | chksm0);
}

#endif //_CRC16_ 