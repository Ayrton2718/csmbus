/*
 * ec_type.c
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "ec_type.h"

ECType_ackChecksum_t ECType_ackChecksumCalculator(const uint8_t* data, size_t data_len)
{
    ECType_ackChecksum_t checksum = 0;
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