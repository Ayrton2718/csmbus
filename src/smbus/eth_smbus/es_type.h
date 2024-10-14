/*
 * es_type.h
 *
 *  Created on: Oct 12, 2023
 *      Author: sen
 */

#ifndef SRC_ETH_SMBUS_ES_TYPE_H_
#define SRC_ETH_SMBUS_ES_TYPE_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESTYPE_TRUE 	(1)
#define ESTYPE_FALSE	(0)
typedef uint8_t 	ESType_bool_t;

#define ESTYPE_GW_IP1	(192)
#define ESTYPE_GW_IP2	(168)
#define ESTYPE_GW_IP3	(0)
#define ESTYPE_GW_IP4	(20)

#define ESTYPE_MASTER_IP1	(192)
#define ESTYPE_MASTER_IP2	(168)
#define ESTYPE_MASTER_IP3	(0)
#define ESTYPE_MASTER_IP4	(5)


typedef enum{
    ESEther_appid_CANSMBUS = 0,
    ESEther_appid_ROBOMAS = 1,
    ESEther_appid_ODRIVE = 2,
    ESEther_appid_NONE = 18,
} ESEther_appid_t;

/*
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
#define ESTYPE_BACKDOOR_M2S_PORT 	    (21210)
#define ESTYPE_BACKDOOR_S2M_PORT 	    (6160)

#define ESTYPE_CTRL_M2S_PORT 	    (21211)
#define ESTYPE_CTRL_S2M_PORT 	    (6161)

#define ESTYPE_APP_M2S_PORT(port, appid)  	(21220 + ((uint8_t)appid * 2) + port)
#define ESTYPE_APP_S2M_PORT(appid)  		(6162 + (uint8_t)appid)

#define ES_APP_MAX_COUNT    (4)
#define ESTYPE_PACKET_MAX_SIZE (512)

typedef struct{
    uint8_t     seq :   7;
    uint8_t     ack :   1;
    uint8_t     reg_type;
}__attribute__((__packed__)) ESEther_header_t;

typedef struct{
    ESEther_header_t    header;
    uint32_t            checksum;
}__attribute__((__packed__)) ESApp_ackPacket_t;

#define ES_ID_MAX_COUNT     (16)
#define ES_REG_MAX_COUNT    (16)

typedef enum{
    ESId_1 = 0,
    ESId_2,
    ESId_3,
    ESId_4,
    ESId_5,
    ESId_6,
    ESId_7,
    ESId_8,
    ESId_9,
    ESId_10,
    ESId_11,
    ESId_12,
    ESId_13,
    ESId_14,
    ESId_15,
    ESId_16,
    ESId_UNKNOWN = 255
} ESId_t;

typedef enum{
    ESPort_1 = 0,
    ESPort_2
} ESPort_t;

typedef enum{
    ESReg_0 = 0, ESReg_1, ESReg_2, ESReg_3, ESReg_4, ESReg_5, ESReg_6, ESReg_7, ESReg_8, ESReg_9, ESReg_10,
    ESReg_11, ESReg_12, ESReg_13, ESReg_14, ESReg_15 
} ESReg_t;


#define ESTYPE_REG_2_REGTYPE(port, reg) (((reg) & 0x0F) | ((port) << 4))
#define ESTYPE_REGTYPE_2_CAN_PORT(regtype) (((regtype) >> 4) & 0x0F)
#define ESTYPE_REGTYPE_2_REG(regtype) ((regtype) & 0x0F)

typedef uint32_t ESType_ackChecksum_t;

ESType_ackChecksum_t ESType_ackChecksumCalculator(const uint8_t* data, size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* SRC_ETH_SMBUS_ES_TYPE_H_ */
