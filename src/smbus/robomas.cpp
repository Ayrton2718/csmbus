#include "robomas.hpp"

#include <pthread.h>
#include "logger/logger.hpp"
#include <tut_tool/tt_timer.hpp>

namespace smbus::robomas
{

typedef struct
{
    tut_tool::RealTimer interval_tim;
    std::atomic<uint64_t> max_interval_us;
    Robomas_sensor_t sensor;
} sensor_info_t;

typedef struct
{
    tut_tool::RealTimer interval_tim;
    std::atomic<uint64_t> max_interval_us;

    Robomas_power_t power[6];
    sensor_info_t   sensor[6];
} bus_info_t;

typedef struct
{
    std::map<app_addr_t, bus_info_t> addr_list;

    ESSocket_t sock;

    pthread_mutex_t locker;
    pthread_t recv_th;
    pthread_t send_th;
} info_t;

static info_t g_obj;

static void* recv_thread(void* args);
static void* send_thread(void* args);

void init(std::set<app_addr_t> addr_list)
{
    g_obj.sock = ESSocket_connect(ESEther_appid_ROBOMAS);

    // リストからキーを取り出して std::map に挿入
    for (const auto& key : addr_list) 
    {
        g_obj.addr_list[key];  // キーを追加し、値はデフォルトコンストラクタで初期化
    }

    for (auto it = g_obj.addr_list.begin(); it != g_obj.addr_list.end(); it++)
    {
        it->second.interval_tim.start();
        it->second.max_interval_us.store(0);
        for(size_t i = 0; i < 6; i++)
        {
            Robomas_power_t& power = it->second.power[i];
            power.mode = Robomas_mode_DISABLE;
            power.cur = 0;
            power.rpm = 0;
            power.ang = 0;

            sensor_info_t& sens_offset = it->second.sensor[i];
            sens_offset.interval_tim.reset();
            sens_offset.max_interval_us.store(0);
            sens_offset.sensor.is_received = 0;
            sens_offset.sensor.is_connected = 0;
            sens_offset.sensor.rpm = 0;
            sens_offset.sensor.cur = 0;
            sens_offset.sensor.set_cur = 0;
            sens_offset.sensor.ang = 0;
        }
    }

    for (auto it = g_obj.addr_list.begin(); it != g_obj.addr_list.end(); ++it) {
        logger::ether_diag_bind(it->first.first, it->first.second, "Robomas", [it](){
                uint64_t interval_us = 0;

                interval_us = it->second.max_interval_us.exchange(0);
                
                logger::diagnostic_t diag;
                if(interval_us != 0 && (interval_us < (10*1000))){
                    diag.status = true;
                }else{
                    diag.status = false;
                    diag.msg = "interval:" + std::to_string((float)interval_us / 1000);
                }
                return diag;
            });
    }

    pthread_mutex_init(&g_obj.locker, NULL);
    int res = pthread_create(&g_obj.recv_th, NULL, recv_thread, NULL);
    if(res != 0)
    {
        logger::err_out("robomas", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }

    res = pthread_create(&g_obj.send_th, NULL, send_thread, NULL);
    if(res != 0)
    {
        logger::err_out("robomas", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }
}

void send_param(ESId_t gw_id, ESPort_t port, id_t number, Robomas_param_t* param)
{
    ESSocket_addr_t addr;
    addr.id = gw_id;
    addr.port = port;
    addr.reg = (ESReg_t)((int)ESReg_8 + (int)number);

    ESSocket_sendAck(g_obj.sock, addr, param, sizeof(Robomas_param_t));
}

void set_power(ESId_t gw_id, ESPort_t port, id_t number, Robomas_power_t* power)
{
    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end())
    {
        pthread_mutex_lock(&g_obj.locker);
        it->second.power[(int)number] = *power;
        pthread_mutex_unlock(&g_obj.locker);
    }
}

Robomas_sensor_t get_sensor(ESId_t gw_id, ESPort_t port, id_t number)
{
    Robomas_sensor_t sens;

    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end())
    {
        pthread_mutex_lock(&g_obj.locker);
        sens = it->second.sensor[(int)number].sensor;
        pthread_mutex_unlock(&g_obj.locker);
    }else{
        sens.is_received = 0;
        sens.is_connected = 0;
        sens.rpm = 0;
        sens.cur = 0;
        sens.set_cur = 0;
        sens.ang = 0;
    }

    return sens;
}

uint64_t get_max_interval(ESId_t gw_id, ESPort_t port, id_t number, bool is_reset)
{
    uint64_t interval_us = 0;

    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end()){
        if(is_reset){
            interval_us = it->second.sensor[(int)number].max_interval_us.exchange(0);
        }else{
            interval_us = it->second.sensor[(int)number].max_interval_us.load();
        }
    }else{
        interval_us = 0;
    }

    return interval_us;
}

static void* recv_thread(void* args)
{
    (void)(args);

    while(1)
    {
        ESSocket_addr_t addr;
        uint8_t buff[ESTYPE_PACKET_MAX_SIZE];
        size_t len;
        if(ESSocket_recv(g_obj.sock, &addr, buff, &len))
        {
            if(ESReg_0 == addr.reg && (len == sizeof(Robomas_sensor_t) * 6))
            {
                Robomas_sensor_t* sens = (Robomas_sensor_t*)buff;
                auto it = g_obj.addr_list.find(std::make_pair(addr.id, addr.port));
                if(it != g_obj.addr_list.end())
                {
                    pthread_mutex_lock(&g_obj.locker);
                    bus_info_t& info = it->second;
                    for(size_t id_i = 0; id_i < 6; id_i++)
                    {
                        info.sensor[id_i].sensor = sens[id_i];

                        if(sens[id_i].is_received){
                            uint64_t interval_us = info.sensor[id_i].interval_tim.getUs();
                            if(info.sensor[id_i].max_interval_us.load() < interval_us){
                                info.sensor[id_i].max_interval_us.store(interval_us);
                            }
                            info.sensor[id_i].interval_tim.reset();
                        }
                    }

                    if(info.max_interval_us.load() < info.interval_tim.getUs()){
                        info.max_interval_us.store(info.interval_tim.getUs());
                    }
                    info.interval_tim.reset();
                    pthread_mutex_unlock(&g_obj.locker);
                }
            }
        }
    }
}


static void* send_thread(void* args)
{
    (void)(args);

    while(1)
    {
        for (auto it = g_obj.addr_list.begin(); it != g_obj.addr_list.end(); ++it) {
            ESSocket_addr_t addr;
            addr.id = it->first.first;
            addr.port = it->first.second;
            addr.reg = ESReg_0;

            Robomas_power_t power_list[6];
            pthread_mutex_lock(&g_obj.locker);
            memcpy(power_list, it->second.power, sizeof(Robomas_power_t)*6);
            pthread_mutex_unlock(&g_obj.locker);

            ESSocket_send(g_obj.sock, addr, power_list, sizeof(Robomas_power_t)*6);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }
}

}