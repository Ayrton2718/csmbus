#include "ec_socket.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#include "../logger/logger.hpp"

using namespace csmbus;

#define EC_ETHER_ACK_LIST_COUNT (16)

typedef void (*ECSocket_ackCallback_t)(const ECSocket_addr_t* addr, uint8_t seq, ECType_ackChecksum_t checksum);

typedef struct
{
    ECType_bool_t is_using;
    uint8_t seq;
    ECSocket_addr_t addr;
    ECType_ackChecksum_t checksum;
    pthread_cond_t cond;
} ECSocket_ackCond_t;

typedef struct
{
    ECEther_appid_t appid;
    
    int send_sock;
    pthread_mutex_t send_locker;
    uint8_t seq;

    int recv_sock;

    ECSocket_ackCond_t ack_buff[EC_ETHER_ACK_LIST_COUNT];
} ECSocket_info_t;


ECSocket_t ECSocket_connect(ECEther_appid_t appid)
{
    ECSocket_info_t* _obj = (ECSocket_info_t*)malloc(sizeof(ECSocket_info_t));
    _obj->appid = appid;

    _obj->send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(_obj->send_sock < 0)
    {
        logger::err_out("eth_smbus", "Create socket failed by \"%s, %d\"", strerror(errno), _obj->send_sock);
    }
    _obj->seq = 0;

    _obj->recv_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(_obj->recv_sock < 0)
    {
        logger::err_out("eth_smbus", "Create socket failed by \"%s, %d\"", strerror(errno), _obj->recv_sock);
    }
    struct sockaddr_in ctrl_recv_addr;
    ctrl_recv_addr.sin_family = AF_INET;
    ctrl_recv_addr.sin_port = htons(ECTYPE_APP_S2M_PORT(appid));
    ctrl_recv_addr.sin_addr.s_addr = INADDR_ANY;
    int res = bind(_obj->recv_sock, (struct sockaddr*)&ctrl_recv_addr, sizeof(ctrl_recv_addr));
    if(res < 0)
    {
        logger::err_out("eth_smbus", "Bind failed by \"%s, %d\"", strerror(errno), res);
    }

    for(size_t i = 0; i < EC_ETHER_ACK_LIST_COUNT; i++)
    {
        ECSocket_ackCond_t* ack_cond = &_obj->ack_buff[i];
        ack_cond->is_using = 0;
        ack_cond->addr.id = ECId_UNKNOWN;
        ack_cond->addr.port = ECPort_1;
        ack_cond->addr.reg = ECReg_0;
        ack_cond->seq = 0;
        ack_cond->checksum = 0;
        if(pthread_cond_init(&ack_cond->cond, NULL) != 0)
        {
            logger::err_out("eth_smbus", "cond_init failed by \"%s\"", strerror(errno));
        }
    }

    pthread_mutex_init(&_obj->send_locker, NULL);

    return _obj;
}


ECType_bool_t ECSocket_recv(ECSocket_t sock_obj, ECSocket_addr_t* addr, void* data, size_t* data_len)
{
    ECSocket_info_t* _obj = (ECSocket_info_t*)sock_obj;

    struct sockaddr_in client_address;
    socklen_t client_address_len = sizeof(client_address);

    char buff[ECTYPE_PACKET_MAX_SIZE];
    int len = recvfrom(_obj->recv_sock, buff, sizeof(buff), 0, (struct sockaddr*)&client_address, &client_address_len);
    if(len < 0)
    {
        logger::err_out("eth_smbus", "recvfrom failed by \"%s, %d\".", strerror(errno), len);
        return ECTYPE_FALSE;
    }else if(ECTYPE_PACKET_MAX_SIZE < len){
        logger::err_out("eth_smbus", "Packet over size(%d byte).", len);
        return ECTYPE_FALSE;
    }

    if((int)sizeof(ECEther_header_t) < len)
    {
        ECEther_header_t* header = (ECEther_header_t*)buff;
        addr->id = ECCtrl_ip_to_id(client_address.sin_addr.s_addr);
        addr->port = (ECPort_t)ECTYPE_REGTYPE_2_CAN_PORT(header->reg_type);
        addr->reg = (ECReg_t)ECTYPE_REGTYPE_2_REG(header->reg_type);

        if(header->ack == 0)
        {
            *data_len = len - sizeof(ECEther_header_t);
            memcpy(data, &buff[sizeof(ECEther_header_t)], (*data_len));
            // TAGGER_INFO("eth_smbus", "input", "Recv from %d, %d, %d, %d[byte]", addr->id, addr->port, addr->reg, *data_len);
            return ECTYPE_TRUE;
        }
        else if(header->ack == 1 && len == sizeof(ECApp_ackPacket_t))
        {
            ECApp_ackPacket_t* ack_packet = (ECApp_ackPacket_t*)buff;
            pthread_mutex_lock(&_obj->send_locker);
            for(size_t i = 0; i < EC_ETHER_ACK_LIST_COUNT; i++)
            {
                ECSocket_ackCond_t* ack_cond = &_obj->ack_buff[i];
                if(ack_cond->is_using)
                {
                    if(ack_cond->seq == header->seq && (memcmp(&ack_cond->addr, addr, sizeof(ECSocket_addr_t)) == 0))
                    {
                        if(ack_packet->checksum == ack_cond->checksum)
                        {
                            pthread_cond_signal(&ack_cond->cond);
                        }else{
                            logger::err_out("eth_smbus", "Wrong checksum received! (%d, %d)", ack_packet->checksum, ack_cond->checksum);
                        }
                    }
                }
            }
            pthread_mutex_unlock(&_obj->send_locker);
            // TAGGER_INFO("eth_smbus", "input", "RecvAck from %d, %d, %d, (%d)", addr->id, addr->port, addr->reg, ack_packet->checksum);
            return ECTYPE_FALSE;
        }
    }
    else
    {
        logger::err_out("eth_smbus", "Too small packet(%d byte).", len);
        return ECTYPE_FALSE;
    }
    return ECTYPE_FALSE;
}


void ECSocket_send(ECSocket_t sock_obj, ECSocket_addr_t addr, const void* data, size_t data_len)
{
    ECSocket_info_t* _obj = (ECSocket_info_t*)sock_obj;

    if(addr.id == ECId_UNKNOWN){
        logger::err_out("eth_smbus", "Invalid id.");
        return;
    }

    char buff[ECTYPE_PACKET_MAX_SIZE];
    if(ECTYPE_PACKET_MAX_SIZE < data_len){
        logger::err_out("eth_smbus", "Packet over size(%d byte).", data_len);
        return;
    }

    size_t send_len = sizeof(ECEther_header_t) + data_len;
    ECEther_header_t* header = (ECEther_header_t*)buff;
    header->ack = 0;
    header->reg_type = ECTYPE_REG_2_REGTYPE(addr.port, addr.reg);
    memcpy(&buff[sizeof(ECEther_header_t)], data, sizeof(char) * data_len);

    struct sockaddr_in send_addr;

    pthread_mutex_lock(&_obj->send_locker);
    send_addr.sin_family = AF_INET;
    header->seq = _obj->seq++;
    send_addr.sin_port = htons(ECTYPE_APP_M2S_PORT(addr.port, _obj->appid));
    send_addr.sin_addr.s_addr = ECCtrl_id_to_ip(addr.id);

    int res = sendto(_obj->send_sock, buff, send_len, 0, (struct sockaddr*)&send_addr, sizeof(struct sockaddr_in));
    if((int)send_len != res)
    {
        logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
    }
    rclcpp::sleep_for(std::chrono::microseconds(1));
    pthread_mutex_unlock(&_obj->send_locker);

    // TAGGER_INFO("eth_smbus", "output", "Send to %d, %d, %d, %d[byte]", addr.id, addr.port, addr.reg, data_len);
}


void ECSocket_sendAck(ECSocket_t sock_obj, ECSocket_addr_t addr, const void* data, size_t data_len)
{
    ECSocket_info_t* _obj = (ECSocket_info_t*)sock_obj;

    if(addr.id == ECId_UNKNOWN){
        logger::err_out("eth_smbus", "Invalid id.");
        return;
    }
    
    char buff[ECTYPE_PACKET_MAX_SIZE];
    if(ECTYPE_PACKET_MAX_SIZE < data_len)
    {
        logger::err_out("eth_smbus", "Packet over size(%d byte).", data_len);
        return;
    }

    size_t send_len = sizeof(ECEther_header_t) + data_len;
    ECEther_header_t* header = (ECEther_header_t*)buff;
    header->ack = 1;
    header->reg_type = ECTYPE_REG_2_REGTYPE(addr.port, addr.reg);
    memcpy(&buff[sizeof(ECEther_header_t)], data, sizeof(char) * data_len);

    struct sockaddr_in send_addr;
    ECType_bool_t is_hit = ECTYPE_FALSE;

    pthread_mutex_lock(&_obj->send_locker);
    header->seq = _obj->seq++;
    send_addr.sin_family = AF_INET;
    send_addr.sin_port = htons(ECTYPE_APP_M2S_PORT(addr.port, _obj->appid));
    send_addr.sin_addr.s_addr = ECCtrl_id_to_ip(addr.id);


    ECType_ackChecksum_t checksum = ECType_ackChecksumCalculator((const uint8_t*)data, data_len);
    int list_i = -1;
    for(size_t i = 0; i < EC_ETHER_ACK_LIST_COUNT; i++)
    {
        ECSocket_ackCond_t* ack_cond = &_obj->ack_buff[i];
        if(ack_cond->is_using == 0)
        {
            ack_cond->is_using = ECTYPE_TRUE;
            ack_cond->seq = header->seq;
            ack_cond->addr = addr;
            ack_cond->checksum = checksum;
            list_i = i;
            break;
        }
    }
    
    if(list_i != -1)
    {
        // TAGGER_INFO("eth_smbus", "output", "SendAck to %d, %d, %d, (%d)", addr.id, addr.port, addr.reg, checksum);
        for(size_t i = 0; i < 5; i++)
        {
            int res = sendto(_obj->send_sock, buff, send_len, 0, (struct sockaddr*)&send_addr, sizeof(struct sockaddr_in));
            if((int)send_len != res)
            {
                logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
            }

            uint64_t ms = 20;
            struct timespec timeout_spec;
            clock_gettime(CLOCK_REALTIME, &timeout_spec);

            timeout_spec.tv_nsec += (ms % (long)1e3) * (long)1e6;
            if((long)1e9 <= timeout_spec.tv_nsec)
            {
                timeout_spec.tv_sec++;
                timeout_spec.tv_nsec -= (long)1e9;
            }

            int result = pthread_cond_timedwait(&_obj->ack_buff[list_i].cond, &_obj->send_locker, &timeout_spec);
            if(result == 0)
            {
                is_hit = ECTYPE_TRUE;
                break;
            }else if(result == ETIMEDOUT || result == EAGAIN){
                logger::err_out("eth_smbus", "Cond wait timeout.");
            }else{
                logger::err_out("eth_smbus", "Cond wait failed.");
            }
        }

        _obj->ack_buff[list_i].is_using = ECTYPE_FALSE;
    }else{
        logger::err_out("eth_smbus", "Cond all using");
    }
    pthread_mutex_unlock(&_obj->send_locker);

    if(is_hit == ECTYPE_FALSE)
    {
        logger::err_out("eth_smbus", "Ack can't receive.");
    }
}