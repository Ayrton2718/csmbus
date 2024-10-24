// Gatewayの接続確認やリセット、安全停止の制御を行う関数

#ifndef ETH_CSMBUS_EC_CTRL_H
#define ETH_CSMBUS_EC_CTRL_H

#include "ec_type.h"
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

void ECCtrl_init(void);

ECType_bool_t ECCtrl_isConnected(ECId_t gw_id);

void ECCtrl_reset(ECId_t gw_id);

void ECCtrl_safetyOff(void);
void ECCtrl_safetyOn(void);

in_addr_t ECCtrl_id_to_ip(ECId_t id);
ECId_t ECCtrl_ip_to_id(in_addr_t ip);

#ifdef __cplusplus
}
#endif

#endif /*ETH_CSMBUS_EC_CTRL_H*/