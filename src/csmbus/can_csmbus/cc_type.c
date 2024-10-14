#include "cs_type.h"

uint16_t CSId_convertId2Num(CSId_t id)
{
    if(CSId_1 <= id && id <= CSId_6)
    {
        return id - CSId_1;
    }else if(CSId_7 <= id && id <= CSId_12){
        return (id - CSId_7) + 6;
    }else{
        return (CSId_UNKNOWN - CSId_7) + 6;
    }
}

CSId_t CSId_convertNum2Id(uint8_t num)
{
    if(num <= 5)
    {
        return num + CSId_1;
    }else if(6 <= num && num <= 11){
        return CSId_7 + (num - 6);
    }else{
        return CSId_UNKNOWN;
    }
}
