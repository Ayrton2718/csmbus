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

// Ctrl通信のEtherパケットのヘッダ

// ACKパケット(Gateway to PC)
typedef struct{
    uint32_t        checksum;
}__attribute__((__packed__))  ECCtrl_ackPacket_t;

// S2M(Gateway to PC)の通信確認用のパケット
// 一定間隔で送信し続けている
typedef struct{
    uint8_t         is_active;
}__attribute__((__packed__)) ECCtrl_s2mPingPacket_t;

// M2S(PC to Gateway)の通信確認用のパケット
// 一定間隔で送信し続けている
typedef struct{
    uint8_t         is_safety_on;
}__attribute__((__packed__)) ECCtrl_m2sPingPacket_t;

// M2S(PC to Gateway)のリセット用のパケット
typedef struct{
    uint32_t        host_seed;
}__attribute__((__packed__)) ECCtrl_m2sResetPacket_t;


// Ctrl通信のレジスタ番号割当
typedef enum{
    ECCtrl_s2mRegType_PING = ECReg_0,
} ECCtrl_s2mRegType_t;

typedef enum{
    ECCtrl_m2sRegType_PING = ECReg_0,
    ECCtrl_m2sRegType_RESET = ECReg_1,
} ECCtrl_m2sRegType_t;

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_CTRL_TYPE_H */
