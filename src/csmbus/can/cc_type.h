/*
 * cc_type.h
 *
 *  Created on: Oct 27, 2023
 *      Author: sen
 */

#ifndef SRC_CAN_CSMBUS_CC_TYPE_H
#define SRC_CAN_CSMBUS_CC_TYPE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t 	CCType_bool_t;

#define CCTYPE_FALSE 	(0)
#define CCTYPE_TRUE 	(1)
#define CCTYPE_BOOL_NOT(x) ((~x) & 0x01)

// CANのタイムアウト時間（Gateway->can_device間）
// この時間を超えると、can_deviceは任意の非常停止処理を行う
#define CCTYPE_SAFETY_TIMEOUT 	(500) // ms

// CAN IDがM2S（master to slave）か、S2M（slave to master）か、BRC（broadcast）かを判定する
#define CCTYPE_IS_M2S_PACKET(can_id) (((uint16_t)can_id & 0b10000000000) == 0)
#define CCTYPE_IS_S2M_PACKET(can_id) (((uint16_t)can_id & 0b10000000000) != 0)
#define CCTYPE_IS_BRC_PACKET(can_id) (((uint16_t)can_id & 0b11111000000) == 0x000)

// CAN IDをM2S（master to slave）か、S2M（slave to master）か、BRC（broadcast）かに変換する
#define CCTYPE_MAKE_M2S_CAN_ID(id, reg) ((((uint16_t)id & 0b1111) << 6) | ((uint16_t)reg & 0b111111))
#define CCTYPE_MAKE_S2M_CAN_ID(id, reg) (0b10000000000 | CCTYPE_MAKE_M2S_CAN_ID(id, reg))

// CAN IDからパケットIDとレジスタフィールドを取得する
#define CCTYPE_GET_PACKET_ID(can_id)    ((CCId_t)(((uint16_t)can_id & 0b01111000000) >> 6))
#define CCTYPE_GET_PACKET_REG(can_id)   ((uint16_t)(((uint16_t)can_id & 0b00000111111)))

// レジスタフィールドがユーザーレジスタか、システムレジスタか、書き込みレジスタか、ACKレジスタかを判定する
#define CCTYPE_IS_USER_REG(reg)         (((uint16_t)reg & 0b011000) != 0b011000)
#define CCTYPE_IS_SYS_REG(reg)          (((uint16_t)reg & 0b011000) == 0b011000)
#define CCTYPE_IS_WRITE_REG(reg)        (((uint16_t)reg & 0b100000) == 0)
#define CCTYPE_IS_ACK_REG(reg)          (((uint16_t)reg & 0b100000) != 0)

// レジスタフィールドからレジスタ番号を取得する
#define CCTYPE_GET_BRC_REG(reg)         ((CCType_brcReg_t)((uint16_t)reg & 0b011111))
#define CCTYPE_GET_USER_REG(reg)        ((CCReg_t)((uint16_t)reg & 0b011111))
#define CCTYPE_GET_SYS_REG(reg)         ((CCReg_t)((uint16_t)reg & 0b000111))

// レジスタ番号からレジスタフィールドを作成する
#define CCTYPE_MAKE_USER_REG(reg)       ((uint16_t)(reg & 0b011111))
#define CCTYPE_MAKE_SYS_REG(reg)        ((uint16_t)(reg | 0b011000))
#define CCTYPE_MAKE_WRITE_REG(reg)      ((uint16_t)(reg & 0b011111))
#define CCTYPE_MAKE_ACK_REG(reg)        ((uint16_t)(reg | 0b100000))

// ack packetの構造体
typedef struct{
    uint8_t checksum;
} CCType_ack_t;

// CANのDevice IDのリスト
// RobomasとCAN IDが被らないようにするために、連番でない
typedef enum
{
    CCId_BRC = 0,
    CCId_1 = 1, CCId_2, CCId_3, CCId_4, CCId_5,  CCId_6, 
    CCId_7 = 9, CCId_8, CCId_9, CCId_10, CCId_11, CCId_12,
    CCId_UNKNOWN = 15
} CCId_t;

// CANのレジスタ番号のリスト
// レジスタ番号により、パケットの種類を識別する
typedef enum{
    CCReg_0 = 0, CCReg_1, CCReg_2, CCReg_3, CCReg_4, CCReg_5, CCReg_6, CCReg_7, CCReg_8, CCReg_9, CCReg_10,
    CCReg_11, CCReg_12, CCReg_13, CCReg_14, CCReg_15, CCReg_16, CCReg_17, CCReg_18, CCReg_19, CCReg_20,
    CCReg_21, CCReg_22, CCReg_23, 
} CCReg_t;


// broadcast用のレジスタのリスト
typedef enum
{
	CCType_brcReg_Safety    = 0b000001,
	CCType_brcReg_Unsafe    = 0b000010,
	CCType_brcReg_Reset     = 0b000011
} CCType_brcReg_t;

// S2M(Slave to Master)の制御通信用のレジスタのリスト
typedef enum{
    CCReg_s2mSystem_APPID = 0, // アプリケーションID
    CCReg_s2mSystem_ERR = 1,   // エラー情報
} CCReg_s2mSystem_t;

// M2S(Master to Slave)の制御通信用のレジスタのリスト
typedef enum{
    CCReg_m2sSystem_REQ_APPID = 0,  // アプリケーションIDの要求
} CCReg_m2sSystem_t;


// アプリケーションIDのリスト
// 0~255までの範囲で、アプリケーションIDを設定する
typedef enum
{
    CCType_appid_AMT212B=0,
    CCType_appid_SICK,
    CCType_appid_SWITCH,
    CCType_appid_COLOR,
    CCType_appid_AIR,
    CCType_appid_SERVO,
    CCType_appid_AMT102,
    CCType_appid_UNKNOWN=255
} CCType_appid_t;

// 連番でないCAN ID（CCId_t）を、連番のCAN ID（uint16_t）に変換する
uint16_t CCId_convertId2Num(CCId_t id);
CCId_t CCId_convertNum2Id(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* SRC_CAN_CSMBUS_CC_TYPE_H */
