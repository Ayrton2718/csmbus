// CSMBusのUDPパケットを作成&送受信するための関数

#ifndef ETH_CSMBUS_EC_SOCKET_H
#define ETH_CSMBUS_EC_SOCKET_H

#include "ec_type.h"
#include "ec_ctrl.h"

#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void* ECSocket_t;

ECSocket_t ECSocket_connect(ECEther_appid_t appid);

typedef struct
{
    ECId_t      id;
    ECPort_t    port;
    ECReg_t     reg;
} ECSocket_addr_t;

ECType_bool_t ECSocket_recv(ECSocket_t sock_obj, ECSocket_addr_t* addr, void* data, size_t* data_len);
void ECSocket_send(ECSocket_t sock_obj, ECSocket_addr_t addr, const void* data, size_t data_len);
void ECSocket_sendAck(ECSocket_t sock_obj, ECSocket_addr_t addr, const void* data, size_t data_len);

#ifdef __cplusplus
}
#endif

#endif /*ETH_CSMBUS_EC_SOCKET_H*/