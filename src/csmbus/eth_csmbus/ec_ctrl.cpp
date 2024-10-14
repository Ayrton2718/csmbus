#include "es_ctrl.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "es_ctrl_type.h"

#include "../logger/logger.hpp"
#include <tut_tool/tt_timer.hpp>

using namespace smbus;

typedef struct
{
    volatile uint8_t    flg;
    uint64_t            timeout[2];
    ESEther_appid_t     port1[2];
    ESEther_appid_t     port2[2];
} ESCtrl_gwStatus_t;

typedef struct
{
    pthread_t ctrl_recv_thread;
    tut_tool::RealTimer     tim;
    ESCtrl_gwStatus_t     gw_status[ES_ID_MAX_COUNT];

    uint8_t ctrl_send_seq;
    uint32_t ctrl_host_seed;
    pthread_mutex_t ctrl_send_locker;
    pthread_t ctrl_send_thread;
    int ctrl_send_sock;
    struct sockaddr_in ctrl_send_addrs;
    volatile uint8_t is_safety_on;
} ESCtrl_info_t;

static ESCtrl_info_t g_obj;

static void* ESCtrl_ctrlRecvThread(void* args);
static void* ESCtrl_safetyThread(void* args);


void ESCtrl_init(void)
{
    g_obj.tim.start();

    g_obj.ctrl_send_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(g_obj.ctrl_send_sock < 0)
    {
        logger::err_out("eth_smbus", "Create socket failed by \"%s, %d\"", strerror(errno), g_obj.ctrl_send_sock);
    }


    pthread_mutex_init(&g_obj.ctrl_send_locker, NULL);
    g_obj.is_safety_on = 1;

    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {
        g_obj.gw_status[gw_id].flg = 0;
        g_obj.gw_status[gw_id].timeout[0] = 0;
        g_obj.gw_status[gw_id].timeout[1] = 0;
        g_obj.gw_status[gw_id].port1[0] = ESEther_appid_NONE;
        g_obj.gw_status[gw_id].port1[1] = ESEther_appid_NONE;
        g_obj.gw_status[gw_id].port2[0] = ESEther_appid_NONE;
        g_obj.gw_status[gw_id].port2[1] = ESEther_appid_NONE;
    }

    srand((unsigned int)time(NULL));
    g_obj.ctrl_send_seq = 0;
    g_obj.ctrl_host_seed = (rand() % (UINT32_MAX - 10)) + 1;
    logger::info_out("eth_smbus", "my_host_id : %d", g_obj.ctrl_host_seed);

    g_obj.ctrl_send_addrs.sin_family = AF_INET;
    g_obj.ctrl_send_addrs.sin_port = htons(ESTYPE_CTRL_M2S_PORT);
    
    int res;
    
    res = pthread_create(&g_obj.ctrl_recv_thread, NULL, ESCtrl_ctrlRecvThread, NULL);
    if(res != 0)
    {
        logger::err_out("eth_smbus", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }

    res = pthread_create(&g_obj.ctrl_send_thread, NULL, ESCtrl_safetyThread, NULL);
    if(res != 0)
    {
        logger::err_out("eth_smbus", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }
}


ESType_bool_t ESCtrl_isConnected(ESId_t gw_id)
{
    uint8_t flg = g_obj.gw_status[gw_id].flg;
    if(g_obj.tim.getMs() < g_obj.gw_status[gw_id].timeout[flg])
    {
        return ESTYPE_TRUE;
    }else{
        return ESTYPE_FALSE;
    }
}

void ESCtrl_reset(ESId_t gw_id)
{
    typedef struct{
        ESEther_header_t header;
        ESCtrl_m2sResetPacket_t msg;
    }__attribute__((__packed__)) packet_t;

    packet_t packet;
    packet.header.seq = 0;
    packet.header.reg_type = ESCtrl_m2sRegType_RESET;
    packet.header.ack = 0;
    packet.msg.host_seed = g_obj.ctrl_host_seed;

    pthread_mutex_lock(&g_obj.ctrl_send_locker);
    packet.header.seq = ++g_obj.ctrl_send_seq;
    g_obj.ctrl_send_addrs.sin_addr.s_addr = ESCtrl_id_to_ip((ESId_t)gw_id);
    int res = sendto(g_obj.ctrl_send_sock, &packet, sizeof(packet_t), 0, (struct sockaddr*)&g_obj.ctrl_send_addrs, sizeof(struct sockaddr_in));
    if((int)sizeof(packet_t) != res)
    {
        logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
    }
    rclcpp::sleep_for(std::chrono::microseconds(1));
    pthread_mutex_unlock(&g_obj.ctrl_send_locker);
}


void* ESCtrl_ctrlRecvThread(void* args)
{
    (void)(args);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
    {
        logger::err_out("eth_smbus", "Create socket failed by \"%s, %d\"", strerror(errno), sock);
    }
    struct sockaddr_in ctrl_recv_addr;
    ctrl_recv_addr.sin_family = AF_INET;
    ctrl_recv_addr.sin_port = htons(ESTYPE_CTRL_S2M_PORT);
    ctrl_recv_addr.sin_addr.s_addr = INADDR_ANY;
    int res = bind(sock, (struct sockaddr*)&ctrl_recv_addr, sizeof(ctrl_recv_addr));
    if(res < 0)
    {
        logger::err_out("eth_smbus", "Bind failed by \"%s, %d\"", strerror(errno), res);
    }

    struct sockaddr_in client_address;
    socklen_t client_address_len = sizeof(client_address);

    char buff[1024];
    while(1)
    {
        int len = recvfrom(sock, buff, sizeof(buff), 0, (struct sockaddr*)&client_address, &client_address_len);
        if(len < 0)
        {
            logger::err_out("eth_smbus", "recvfrom failed by \"%s, %d\".", strerror(errno), len);
            continue;
        }else if(ESTYPE_PACKET_MAX_SIZE < len){
            logger::err_out("eth_smbus", "Packet over size(%d byte).", len);
            continue;
        }

        if((int)sizeof(ESEther_header_t) <= len)
        {
            ESId_t id = ESCtrl_ip_to_id(client_address.sin_addr.s_addr);
            ESEther_header_t* header = (ESEther_header_t*)buff;
            switch(header->reg_type)
            {
            case ESCtrl_s2mRegType_PING:{
                if(len == sizeof(ESEther_header_t) + sizeof(ESCtrl_s2mPingPacket_t)){
                    uint8_t flg = !g_obj.gw_status[id].flg;
                    g_obj.gw_status[id].timeout[flg] = g_obj.tim.getMs() + 1000;
                    g_obj.gw_status[id].flg = flg;
                }
                }break;

            default:
                break;
            }
        }
    }
}



void ESCtrl_safetyOff(void)
{
    typedef struct{
        ESEther_header_t header;
        ESCtrl_m2sPingPacket_t msg;
    }__attribute__((__packed__)) packet_t;

    packet_t packet[ES_ID_MAX_COUNT];

    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {   
        packet[gw_id].header.seq = 0;
        packet[gw_id].header.reg_type = ESCtrl_m2sRegType_PING;
        packet[gw_id].header.ack = 0;
        packet[gw_id].msg.is_safety_on = 0;
    }

    pthread_mutex_lock(&g_obj.ctrl_send_locker);
    g_obj.is_safety_on = 0;
    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {
        packet[gw_id].header.seq = ++g_obj.ctrl_send_seq;
        packet[gw_id].msg.is_safety_on = g_obj.is_safety_on;
        g_obj.ctrl_send_addrs.sin_addr.s_addr = ESCtrl_id_to_ip((ESId_t)gw_id);
        int res = sendto(g_obj.ctrl_send_sock, &packet[gw_id], sizeof(packet_t), 0, (struct sockaddr*)&g_obj.ctrl_send_addrs, sizeof(struct sockaddr_in));
        if((int)sizeof(packet_t) != res)
        {
            logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
        }
    }
    rclcpp::sleep_for(std::chrono::microseconds(1));
    pthread_mutex_unlock(&g_obj.ctrl_send_locker);
}

void ESCtrl_safetyOn(void)
{
    typedef struct{
        ESEther_header_t header;
        ESCtrl_m2sPingPacket_t msg;
    }__attribute__((__packed__)) packet_t;

    packet_t packet[ES_ID_MAX_COUNT];

    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {   
        packet[gw_id].header.seq = 0;
        packet[gw_id].header.reg_type = ESCtrl_m2sRegType_PING;
        packet[gw_id].header.ack = 0;
        packet[gw_id].msg.is_safety_on = 0;
    }

    pthread_mutex_lock(&g_obj.ctrl_send_locker);
    g_obj.is_safety_on = 1;
    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {
        packet[gw_id].header.seq = ++g_obj.ctrl_send_seq;
        packet[gw_id].msg.is_safety_on = g_obj.is_safety_on;
        g_obj.ctrl_send_addrs.sin_addr.s_addr = ESCtrl_id_to_ip((ESId_t)gw_id);
        int res = sendto(g_obj.ctrl_send_sock, &packet[gw_id], sizeof(packet_t), 0, (struct sockaddr*)&g_obj.ctrl_send_addrs, sizeof(struct sockaddr_in));
        if((int)sizeof(packet_t) != res)
        {
            logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
        }
    }
    rclcpp::sleep_for(std::chrono::microseconds(1));
    pthread_mutex_unlock(&g_obj.ctrl_send_locker);
}

void* ESCtrl_safetyThread(void* args)
{
    (void)(args);

    typedef struct{
        ESEther_header_t header;
        ESCtrl_m2sPingPacket_t msg;
    }__attribute__((__packed__)) packet_t;

    packet_t packet[ES_ID_MAX_COUNT];

    for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
    {   
        packet[gw_id].header.seq = 0;
        packet[gw_id].header.reg_type = ESCtrl_m2sRegType_PING;
        packet[gw_id].header.ack = 0;
        packet[gw_id].msg.is_safety_on = 1;
    }

    while(1)
    {
        pthread_mutex_lock(&g_obj.ctrl_send_locker);
        for(uint32_t gw_id = ESId_1; gw_id < ES_ID_MAX_COUNT; gw_id++)
        {
            packet[gw_id].header.seq = ++g_obj.ctrl_send_seq;
            packet[gw_id].msg.is_safety_on = g_obj.is_safety_on;
            g_obj.ctrl_send_addrs.sin_addr.s_addr = ESCtrl_id_to_ip((ESId_t)gw_id);
            int res = sendto(g_obj.ctrl_send_sock, &packet[gw_id], sizeof(packet_t), 0, (struct sockaddr*)&g_obj.ctrl_send_addrs, sizeof(struct sockaddr_in));
            if((int)sizeof(packet_t) != res)
            {
                logger::err_out("eth_smbus", "Send failed by \"%s, %d\"", strerror(errno), res);
            }
        }
        rclcpp::sleep_for(std::chrono::microseconds(1));
        pthread_mutex_unlock(&g_obj.ctrl_send_locker);

        tut_tool::RealTimer::sleep_ms(200);
    }
}


in_addr_t ESCtrl_id_to_ip(ESId_t id)
{
    in_addr_t ip_as_integer = 0;
    ip_as_integer |= ((ESTYPE_GW_IP4 + id) << 24);
    ip_as_integer |= (ESTYPE_GW_IP3 << 16);
    ip_as_integer |= (ESTYPE_GW_IP2 << 8);
    ip_as_integer |= (ESTYPE_GW_IP1 << 0);
    return ip_as_integer;
}

ESId_t ESCtrl_ip_to_id(in_addr_t ip)
{
    uint32_t ip4 = (ip & 0xFF000000) >> 24;
    if(ESTYPE_GW_IP4 <= ip4)
    {
        ip4 = ip4 - ESTYPE_GW_IP4;
        if(ip4 <= ES_ID_MAX_COUNT)
        {
            return (ESId_t)ip4;
        }
    }

    return ESId_UNKNOWN;
}