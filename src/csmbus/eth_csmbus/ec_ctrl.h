#ifndef ETH_SMBUS_ES_IO_H
#define ETH_SMBUS_ES_IO_H

#include "es_type.h"
#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

void ESCtrl_init(void);

ESType_bool_t ESCtrl_isConnected(ESId_t gw_id);

void ESCtrl_reset(ESId_t gw_id);

void ESCtrl_safetyOff(void);
void ESCtrl_safetyOn(void);


in_addr_t ESCtrl_id_to_ip(ESId_t id);
ESId_t ESCtrl_ip_to_id(in_addr_t ip);

#ifdef __cplusplus
}
#endif

#endif /*ETH_SMBUS_ES_IO_H*/