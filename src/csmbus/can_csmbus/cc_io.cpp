#include "cs_io.hpp"

#include "tut_tool/tt_timer.hpp"

#define CS_IO_BUFF_COUNT (16)

namespace smbus::can_smbus::io
{

//type
typedef struct 
{
    uint8_t wp;
    volatile uint8_t rp;
    volatile uint8_t count;
    CanSMBus_packet_t packet[CS_IO_BUFF_COUNT];
} CSIo_ringBuffer_t;

typedef std::map<CSId_t, std::vector<can_callback_t>> devices_t;
typedef std::map<app_addr_t, std::pair<CSIo_ringBuffer_t, devices_t>> port_list_t;

typedef struct
{
    ESSocket_t sock;

    port_list_t port_list;

    pthread_mutex_t send_locker;
    pthread_mutex_t recv_locker;
    pthread_t recv_th;
    pthread_t send_th;
} info_t;

static info_t g_obj;

static void send_raw(ESId_t gw_id, ESPort_t port, CanSMBus_packet_t packet);

static void* recv_thread(void* args);
static void* send_thread(void* args);


void init(std::set<app_addr_t> add_list)
{
    g_obj.sock = ESSocket_connect(ESEther_appid_CANSMBUS);

    for(auto it = add_list.begin(); it != add_list.end(); it++)
    {
        CSIo_ringBuffer_t ring;
        ring.wp = 0;
        ring.rp = 0;
        ring.count = 0;
        memset(&ring.packet, 0x00, sizeof(CanSMBus_packet_t) * CS_IO_BUFF_COUNT);

        devices_t dev;
        g_obj.port_list[*it] = std::make_pair(ring, dev);
    }

    pthread_mutex_init(&g_obj.recv_locker, NULL);
    int res = pthread_create(&g_obj.recv_th, NULL, recv_thread, NULL);
    if(res != 0)
    {
        logger::err_out("can_smbus", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }

    pthread_mutex_init(&g_obj.send_locker, NULL);
    res = pthread_create(&g_obj.send_th, NULL, send_thread, NULL);
    if(res != 0)
    {
        logger::err_out("can_smbus", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }
}

void bind(ESId_t gw_id, ESPort_t port, CSId_t can_id, can_callback_t callback)
{
    auto it = g_obj.port_list.find(std::make_pair(gw_id, port));
    if(it == g_obj.port_list.end())
    {
        logger::err_out("can_smbus", "Undefined port gw_id(%d), port(%d)", gw_id, port);
        return;  
    }
    
    pthread_mutex_lock(&g_obj.recv_locker);
    it->second.second[can_id].push_back(callback);
    pthread_mutex_unlock(&g_obj.recv_locker);
}

void send(ESId_t gw_id, ESPort_t port, CSId_t can_id, uint16_t raw_reg, uint8_t len, const uint8_t* data)
{
    CanSMBus_packet_t packet;
    packet.can_id = CSTYPE_MAKE_M2S_CAN_ID(can_id, raw_reg);
    packet.len = len;
    memcpy(packet.data, data, sizeof(uint8_t) * len);
    send_raw(gw_id, port, packet);
}


static void send_raw(ESId_t gw_id, ESPort_t port, CanSMBus_packet_t packet)
{
    auto it = g_obj.port_list.find(std::make_pair(gw_id, port));
    if(it == g_obj.port_list.end())
    {
        logger::err_out("can_smbus", "Undefined port gw_id(%d), port(%d)", gw_id, port);
        return;  
    }

    pthread_mutex_lock(&g_obj.send_locker);
    CSIo_ringBuffer_t* ring = &it->second.first;
    if(ring->count == CS_IO_BUFF_COUNT)
    {
        ring->rp++;
        ring->count--;
        logger::err_out("can_smbus", "Buffer overflowed gw_id(%d), port(%d)", gw_id, port);
    }
    ring->packet[ring->wp % CS_IO_BUFF_COUNT] = packet;
    ring->wp++;
    ring->count++;
    pthread_mutex_unlock(&g_obj.send_locker);
}

static void* recv_thread(void* args)
{
    (void)(args);

    size_t report_undefined_counter = 0;

    while(1)
    {
        ESSocket_addr_t addr;
        uint8_t buff[ESTYPE_PACKET_MAX_SIZE];
        size_t len;
        if(ESSocket_recv(g_obj.sock, &addr, buff, &len))
        {
            auto it = g_obj.port_list.find(std::make_pair(addr.id, addr.port));
            if(it == g_obj.port_list.end())
            {
                logger::err_out("can_smbus", "Undefined port gw_id(%d), port(%d)", addr.id, addr.port);
                continue;
            }

            pthread_mutex_lock(&g_obj.recv_locker);
            if((addr.reg == ESReg_0) && (len == sizeof(CanSMBus_s2m_t)))
            {
                const CanSMBus_s2m_t* s2m = (const CanSMBus_s2m_t*)buff;

                for(size_t i = 0; (i < s2m->count) && (i < 4); i++)
                {
                    const CanSMBus_packet_t* packet = (const CanSMBus_packet_t*)&s2m->packet[i];

                    if(CSTYPE_IS_S2M_PACKET(packet->can_id))
                    {
                        CSId_t can_id = CSTYPE_GET_PACKET_ID(packet->can_id);
                        uint16_t raw_reg = CSTYPE_GET_PACKET_REG(packet->can_id);

                        auto vec_it = it->second.second.find(can_id);
                        if(vec_it != it->second.second.end())
                        {
                            for(can_callback_t callback : vec_it->second)
                            {
                                callback(raw_reg, packet->len, packet->data);
                            }
                        }
                        else
                        {
                            // 開始して20回までのUndefinedエラーを報告
                            if(report_undefined_counter < 20)
                            {
                                logger::err_out("can_smbus", "Undefined can_id gw_id(%d), port(%d), can_id(%d)", addr.id, addr.port, CSId_convertId2Num(can_id));
                                report_undefined_counter++;
                            }
                        }
                    }
                }
            }
            pthread_mutex_unlock(&g_obj.recv_locker);
        }
    }
}

static void* send_thread(void* args)
{
    (void)(args);

    while(1)
    {
        for (auto& [key, value] : g_obj.port_list)
        {
            CSIo_ringBuffer_t* ring = &value.first;
            if(0 < ring->count)
            {
                CanSMBus_m2s_t m2s;
                m2s.count = 0;

                pthread_mutex_lock(&g_obj.send_locker);
                size_t send_count = ring->count;
                for(size_t i = 0; (i < send_count) && (i < 4); i++)
                {
                    m2s.packet[m2s.count] = ring->packet[ring->rp % CS_IO_BUFF_COUNT];
                    m2s.count++;

                    ring->rp++;
                    ring->count--;
                }
                pthread_mutex_unlock(&g_obj.send_locker);

                if(0 < m2s.count)
                {
                    ESSocket_addr_t addr;
                    addr.id = key.first;
                    addr.port = key.second;
                    addr.reg = ESReg_0;
                    ESSocket_send(g_obj.sock, addr, &m2s, sizeof(CanSMBus_m2s_t));
                }
            }
        }

        tut_tool::RealTimer::sleep_us(500);
    }
}

}