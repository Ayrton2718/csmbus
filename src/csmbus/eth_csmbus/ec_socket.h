#ifndef ETH_SMBUS_ES_SOCKET_H
#define ETH_SMBUS_ES_SOCKET_H

#include "es_type.h"
#include "es_ctrl.h"

#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* ESSocket_t;

ESSocket_t ESSocket_connect(ESEther_appid_t appid);

typedef struct
{
    ESId_t      id;
    ESPort_t    port;
    ESReg_t     reg;
} ESSocket_addr_t;

ESType_bool_t ESSocket_recv(ESSocket_t sock_obj, ESSocket_addr_t* addr, void* data, size_t* data_len);
void ESSocket_send(ESSocket_t sock_obj, ESSocket_addr_t addr, const void* data, size_t data_len);
void ESSocket_sendAck(ESSocket_t sock_obj, ESSocket_addr_t addr, const void* data, size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /*ETH_SMBUS_ES_SOCKET_H*/