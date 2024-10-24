/*
 * ec_type.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_CSMBUS_EC_TYPE_H
#define SRC_ETH_CSMBUS_EC_TYPE_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ECTYPE_TRUE 	(1)
#define ECTYPE_FALSE	(0)
typedef uint8_t 	ECType_bool_t;

// Gatewayの開始IPアドレス
// 20から始まり、20+ID番号がGatewayのIPアドレスとなる
#define ECTYPE_GW_IP1	(192)
#define ECTYPE_GW_IP2	(168)
#define ECTYPE_GW_IP3	(0)
#define ECTYPE_GW_IP4	(20)

// PCのIPアドレス
#define ECTYPE_MASTER_IP1	(192)
#define ECTYPE_MASTER_IP2	(168)
#define ECTYPE_MASTER_IP3	(0)
#define ECTYPE_MASTER_IP4	(5)

// ポートのアプリケーションID
// Gatewayの任意のCANポートで、どのアプリケーションが動いているかを識別する
typedef enum{
    ECEther_appid_CANSMBUS = 0,
    ECEther_appid_ROBOMAS = 1,
    ECEther_appid_ODRIVE = 2,
    ECEther_appid_NONE = 18,
} ECEther_appid_t;

/*
    * ポート番号の設定
    * 21210: Backdoor M2S
    * 6160: Backdoor S2M
    * 21211: Ctrl M2S
    * 6161: Ctrl S2M
    * 21220~21230: App M2S
    * 6162~6172: App S2M
    * 
    *
Check the linux port setting like this:
$ sudo sysctl -p
net.ipv4.ip_local_reserved_ports = 6171-6191

How to setup
$ sudo cp /etc/sysctl.conf /etc/sysctl.conf.org
$ sudo bash -c "echo 'net.ipv4.ip_local_reserved_ports=6171-6191' >> /etc/sysctl.conf"
$ sudo sysctl -p

lsof port scan
$ lsof -i -P
*/
#define ECTYPE_BACKDOOR_M2S_PORT 	    (21210)
#define ECTYPE_BACKDOOR_S2M_PORT 	    (6160)

#define ECTYPE_CTRL_M2S_PORT 	    (21211)
#define ECTYPE_CTRL_S2M_PORT 	    (6161)

#define ECTYPE_APP_M2S_PORT(port, appid)  	(21220 + ((uint8_t)appid * 2) + port)
#define ECTYPE_APP_S2M_PORT(appid)  		(6162 + (uint8_t)appid)

// 一つのポートに対して、最大4つのアプリケーションを設定できる
#define ECTYPE_APP_MAX_COUNT    (4)

// EtherのUDPパケットの最大サイズ
#define ECTYPE_PACKET_MAX_SIZE (512)

#define EC_ID_MAX_COUNT     (16) // GatewayのIDの最大数
#define EC_REG_MAX_COUNT    (16) // Gatewayのレジスタ番号の最大数

// Etherパケットのヘッダ
typedef struct{
    uint8_t     seq :   7;
    uint8_t     ack :   1;
    uint8_t     reg_type;
}__attribute__((__packed__)) ECEther_header_t;

// EtherパケットのACKパケット
typedef struct{
    ECEther_header_t    header;
    uint32_t            checksum;
}__attribute__((__packed__)) ECApp_ackPacket_t;

// GatewayのIDのリスト
typedef enum{
    ECId_1 = 0,
    ECId_2,
    ECId_3,
    ECId_4,
    ECId_5,
    ECId_6,
    ECId_7,
    ECId_8,
    ECId_9,
    ECId_10,
    ECId_11,
    ECId_12,
    ECId_13,
    ECId_14,
    ECId_15,
    ECId_16,
    ECId_UNKNOWN = 255
} ECId_t;

// Gatewayのポートのリスト
typedef enum{
    ECPort_1 = 0,
    ECPort_2
} ECPort_t;

// Gatewayのレジスタのリスト
// レジスタ番号により、パケットの種類を識別する
typedef enum{
    ECReg_0 = 0, ECReg_1, ECReg_2, ECReg_3, ECReg_4, ECReg_5, ECReg_6, ECReg_7, ECReg_8, ECReg_9, ECReg_10,
    ECReg_11, ECReg_12, ECReg_13, ECReg_14, ECReg_15 
} ECReg_t;


// 1バイトのレジスタフィールドにポート番号とレジスタ番号を格納しているので、それを分解する
#define ECTYPE_REG_2_REGTYPE(port, reg) (((reg) & 0x0F) | ((port) << 4))
#define ECTYPE_REGTYPE_2_CAN_PORT(regtype) (((regtype) >> 4) & 0x0F)
#define ECTYPE_REGTYPE_2_REG(regtype) ((regtype) & 0x0F)

typedef uint32_t ECType_ackChecksum_t;

// ACKパケットのチェックサムを計算する
ECType_ackChecksum_t ECType_ackChecksumCalculator(const uint8_t* data, size_t data_len);


#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_CSMBUS_EC_TYPE_H */
