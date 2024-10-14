/*
 * es_type.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_SMBUS_ES_CTRL_TYPE_H_
#define SRC_ETH_SMBUS_ES_CTRL_TYPE_H_

#include "es_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint32_t        checksum;
}__attribute__((__packed__))  ESCtrl_ackPacket_t;

typedef struct{
    uint8_t         is_active;
}__attribute__((__packed__)) ESCtrl_s2mPingPacket_t;

typedef struct{
    uint8_t         is_safety_on;
}__attribute__((__packed__)) ESCtrl_m2sPingPacket_t;

typedef struct{
    uint32_t        host_seed;
}__attribute__((__packed__)) ESCtrl_m2sResetPacket_t;

typedef enum{
    ESCtrl_s2mRegType_PING = 0
} ESCtrl_s2mRegType_t;


typedef enum{
    ESCtrl_m2sRegType_PING = 0,
    ESCtrl_m2sRegType_RESET = 1,
} ESCtrl_m2sRegType_t;

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_SMBUS_ES_CTRL_TYPE_H_ */
