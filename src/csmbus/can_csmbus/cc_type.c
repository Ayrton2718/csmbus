/*
 * cc_type.c
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#include "cc_type.h"

uint16_t CCId_convertId2Num(CCId_t id)
{
    if(CCId_1 <= id && id <= CCId_6)
    {
        return id - CCId_1;
    }else if(CCId_7 <= id && id <= CCId_12){
        return (id - CCId_7) + 6;
    }else{
        return (CCId_UNKNOWN - CCId_7) + 6;
    }
}

CCId_t CCId_convertNum2Id(uint8_t num)
{
    if(num <= 5)
    {
        return num + CCId_1;
    }else if(6 <= num && num <= 11){
        return CCId_7 + (num - 6);
    }else{
        return CCId_UNKNOWN;
    }
}
