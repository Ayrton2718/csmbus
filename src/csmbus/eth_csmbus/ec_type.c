#include "es_type.h"

ESType_ackChecksum_t ESType_ackChecksumCalculator(const uint8_t* data, size_t data_len)
{
    ESType_ackChecksum_t checksum = 0;
    for(size_t i = 0; i < data_len % 4; i++)
    {
        checksum += data[i];
    }
    const uint32_t* data32 = (const uint32_t*)(&data[data_len % 4]);
    for(size_t i = 0; i < data_len / 4; i++)
    {
        checksum += data32[i];
    }
    return checksum;
}