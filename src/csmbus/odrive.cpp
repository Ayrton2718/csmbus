#include "odrive.hpp"

namespace csmbus::odrive
{

typedef struct
{
    RealTimer interval_tim;
    std::atomic<uint64_t> max_interval_us;
    Odrive_sensor_t sensor;
    uint32_t befor_err_code;
} sensor_info_t;

typedef struct
{
    RealTimer interval_tim;
    std::atomic<uint64_t> max_interval_us;

    Odrive_power_t power[4];
    sensor_info_t sensor[4];
} bus_info_t;

typedef struct
{
    std::map<app_addr_t, bus_info_t> addr_list;

    ECSocket_t sock;

    pthread_mutex_t locker;
    pthread_t recv_th;
    pthread_t send_th;
} info_t;

static info_t g_obj;
static void* recv_thread(void* args);
static void* send_thread(void* args);

static std::string report_axis_err(uint32_t err_code);

void init(std::set<app_addr_t> addr_list)
{
    g_obj.sock = ECSocket_connect(ECEther_appid_ODRIVE);


    // リストからキーを取り出して std::map に挿入
    for (const auto& key : addr_list) 
    {
        g_obj.addr_list[key];  // キーを追加し、値はデフォルトコンストラクタで初期化
    }

    for (auto it = g_obj.addr_list.begin(); it != g_obj.addr_list.end(); it++)
    {
        it->second.interval_tim.start();
        it->second.max_interval_us.store(0);
        for(size_t i = 0; i < 4; i++)
        {
            Odrive_power_t& pw_offset = it->second.power[i];
            pw_offset.mode = Odrive_mode_DISABLE;
            pw_offset.torque = 0;
            pw_offset.velocity = 0;

            sensor_info_t& sens_offset = it->second.sensor[i];
            sens_offset.interval_tim.start();
            sens_offset.max_interval_us.store(0);
            sens_offset.sensor.heartbeat_received = 0;
            sens_offset.sensor.error_received = 0;
            sens_offset.sensor.encoder_estimate_received = 0;
            sens_offset.sensor.torques_received = 0;
            sens_offset.sensor.axis_error = 0;
            sens_offset.sensor.axis_state = 0;
            sens_offset.sensor.procedure_result = 0;
            sens_offset.sensor.active_errors = 0;
            sens_offset.sensor.disarm_reason = 0;
            sens_offset.sensor.pos_estimate = 0;
            sens_offset.sensor.vel_estimate = 0;
            sens_offset.sensor.torque_target = 0;
            sens_offset.sensor.torque_estimate = 0;
            sens_offset.befor_err_code = 0;
        }
    }

    for (auto it = g_obj.addr_list.begin(); it != g_obj.addr_list.end(); ++it) {
        logger::ether_diag_bind(it->first.first, it->first.second, "Odrive", [it](){
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
        logger::err_out("odrive", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }

    res = pthread_create(&g_obj.send_th, NULL, send_thread, NULL);
    if(res != 0)
    {
        logger::err_out("odrive", "Pthread_create failed by \"%s, %d\"", strerror(errno), res);
    }
}

void send_param(ECId_t gw_id, ECPort_t port, id_t number, Odrive_param_t* param)
{
    ECSocket_addr_t addr;
    addr.id = gw_id;
    addr.port = port;
    addr.reg = (ECReg_t)((int)ECReg_8 + (int)number);

    ECSocket_sendAck(g_obj.sock, addr, param, sizeof(Odrive_param_t));
}

void set_power(ECId_t gw_id, ECPort_t port, id_t number, Odrive_power_t* power)
{
    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end())
    {
        pthread_mutex_lock(&g_obj.locker);
        it->second.power[(int)number] = *power;
        pthread_mutex_unlock(&g_obj.locker);
    }
}

Odrive_sensor_t get_sensor(ECId_t gw_id, ECPort_t port, id_t number)
{
    Odrive_sensor_t sens;

    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end())
    {
        pthread_mutex_lock(&g_obj.locker);
        sens = it->second.sensor[(int)number].sensor;
        pthread_mutex_unlock(&g_obj.locker);
    }else{
        sens.is_connected = 0;
        sens.heartbeat_received = 0;
        sens.error_received = 0;
        sens.encoder_estimate_received = 0;
        sens.torques_received = 0;
        sens.axis_error = 0;
        sens.axis_state = 0;
        sens.procedure_result = 0;
        sens.active_errors = 0;
        sens.disarm_reason = 0;
        sens.pos_estimate = 0;
        sens.vel_estimate = 0;
        sens.torque_target = 0;
        sens.torque_estimate = 0;
    }

    return sens;
}

uint64_t get_max_interval(ECId_t gw_id, ECPort_t port, id_t number, bool is_reset)
{
    uint64_t interval_us = 0;

    auto it = g_obj.addr_list.find(std::make_pair(gw_id, port));
    if(it != g_obj.addr_list.end())
    {
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
        ECSocket_addr_t addr;
        uint8_t buff[ECTYPE_PACKET_MAX_SIZE];
        size_t len;
        if(ECSocket_recv(g_obj.sock, &addr, buff, &len))
        {
            if(ECReg_0 == addr.reg && (len == sizeof(Odrive_sensor_t) * 4))
            {
                Odrive_sensor_t* sens = (Odrive_sensor_t*)buff;

                auto it = g_obj.addr_list.find(std::make_pair(addr.id, addr.port));
                if(it != g_obj.addr_list.end())
                {
                    pthread_mutex_lock(&g_obj.locker);
                    bus_info_t& info = it->second;
                    for(size_t id_i = 0; id_i < 4; id_i++)
                    {
                        info.sensor[id_i].sensor = sens[id_i];

                        if(sens[id_i].encoder_estimate_received){
                            uint64_t interval_us = info.sensor[id_i].interval_tim.getUs();
                            if(info.sensor[id_i].max_interval_us.load() < interval_us){
                                info.sensor[id_i].max_interval_us.store(interval_us);
                            }
                            info.sensor[id_i].interval_tim.reset();
                        }

                        if(info.sensor[id_i].befor_err_code != sens[id_i].active_errors)
                        {
                            logger::err_out("odrive", "%d, %d, %d, (%d, %d, %d, %d, %d)", addr.id, addr.port, id_i, 
                                sens->axis_error, sens->axis_state, sens->procedure_result, sens->active_errors, sens->disarm_reason);
                            logger::err_out("odrive", "err_list=" + report_axis_err(sens[id_i].active_errors));
                            info.sensor[id_i].befor_err_code = sens[id_i].active_errors;
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
            ECSocket_addr_t addr;
            addr.id = it->first.first;
            addr.port = it->first.second;
            addr.reg = ECReg_0;
            
            Odrive_power_t power_list[4];
            pthread_mutex_lock(&g_obj.locker);
            memcpy(power_list, it->second.power, sizeof(Odrive_power_t)*4);
            pthread_mutex_unlock(&g_obj.locker);

            ECSocket_send(g_obj.sock, addr, power_list, sizeof(Odrive_power_t)*4);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }
}

static std::string report_axis_err(uint32_t err_code)
{
    std::string err_str = "[";
    if (err_code & INITIALIZING)                err_str += "INITIALIZING, ";
    if (err_code & SYSTEM_LEVEL)                err_str += "SYSTEM_LEVEL, ";
    if (err_code & TIMING_ERROR)                err_str += "TIMING_ERROR, ";
    if (err_code & MISSING_ESTIMATE)            err_str += "MISSING_ESTIMATE, ";
    if (err_code & BAD_CONFIG)                  err_str += "BAD_CONFIG, ";
    if (err_code & DRV_FAULT)                   err_str += "DRV_FAULT, ";
    if (err_code & MISSING_INPUT)               err_str += "MISSING_INPUT, ";
    if (err_code & DC_BUS_OVER_VOLTAGE)         err_str += "DC_BUS_OVER_VOLTAGE, ";
    if (err_code & DC_BUS_UNDER_VOLTAGE)        err_str += "DC_BUS_UNDER_VOLTAGE, ";
    if (err_code & DC_BUS_OVER_CURRENT)         err_str += "DC_BUS_OVER_CURRENT, ";
    if (err_code & DC_BUS_OVER_REGEN_CURRENT)   err_str += "DC_BUS_OVER_REGEN_CURRENT, ";
    if (err_code & CURRENT_LIMIT_VIOLATION)     err_str += "CURRENT_LIMIT_VIOLATION, ";
    if (err_code & MOTOR_OVER_TEMP)             err_str += "MOTOR_OVER_TEMP, ";
    if (err_code & INVERTER_OVER_TEMP)          err_str += "INVERTER_OVER_TEMP, ";
    if (err_code & VELOCITY_LIMIT_VIOLATION)    err_str += "VELOCITY_LIMIT_VIOLATION, ";
    if (err_code & POSITION_LIMIT_VIOLATION)    err_str += "POSITION_LIMIT_VIOLATION, ";
    if (err_code & WATCHDOG_TIMER_EXPIRED)      err_str += "WATCHDOG_TIMER_EXPIRED, ";
    if (err_code & ESTOP_REQUESTED)             err_str += "ESTOP_REQUESTED, ";
    if (err_code & SPINOUT_DETECTED)            err_str += "SPINOUT_DETECTED, ";
    if (err_code & BRAKE_RESISTOR_DISARMED)     err_str += "BRAKE_RESISTOR_DISARMED, ";
    if (err_code & THERMISTOR_DISCONNECTED)     err_str += "THERMISTOR_DISCONNECTED, ";
    if (err_code & CALIBRATION_ERROR)           err_str += "CALIBRATION_ERROR, ";

    err_str += "]";
    return err_str;
}

}