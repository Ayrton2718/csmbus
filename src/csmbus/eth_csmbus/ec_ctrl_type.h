/*
 * ec_ctrl_type.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_CTRL_TYPE_H
#define SRC_ETH_CSMBUS_EC_CTRL_TYPE_H

#include "ec_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    uint32_t        checksum;
}__attribute__((__packed__))  ECCtrl_ackPacket_t;

typedef struct{
    uint8_t         is_active;
}__attribute__((__packed__)) ECCtrl_s2mPingPacket_t;

typedef struct{
    uint8_t         is_safety_on;
}__attribute__((__packed__)) ECCtrl_m2sPingPacket_t;

typedef struct{
    uint32_t        host_seed;
}__attribute__((__packed__)) ECCtrl_m2sResetPacket_t;

typedef enum{
    ECCtrl_s2mRegType_PING = 0
} ECCtrl_s2mRegType_t;


typedef enum{
    ECCtrl_m2sRegType_PING = 0,
    ECCtrl_m2sRegType_RESET = 1,
} ECCtrl_m2sRegType_t;

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_CTRL_TYPE_H */
