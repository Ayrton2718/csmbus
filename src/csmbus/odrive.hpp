#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "eth/eth_csmbus.h"

#include "logger/logger.hpp"

namespace csmbus
{

namespace odrive
{

enum axis_err_t {
    INITIALIZING = 0x1,
    SYSTEM_LEVEL = 0x2,
    TIMING_ERROR = 0x4,
    MISSING_ESTIMATE = 0x8,
    BAD_CONFIG = 0x10,
    DRV_FAULT = 0x20,
    MISSING_INPUT = 0x40,
    DC_BUS_OVER_VOLTAGE = 0x100,
    DC_BUS_UNDER_VOLTAGE = 0x200,
    DC_BUS_OVER_CURRENT = 0x400,
    DC_BUS_OVER_REGEN_CURRENT = 0x800,
    CURRENT_LIMIT_VIOLATION = 0x1000,
    MOTOR_OVER_TEMP = 0x2000,
    INVERTER_OVER_TEMP = 0x4000,
    VELOCITY_LIMIT_VIOLATION = 0x8000,
    POSITION_LIMIT_VIOLATION = 0x10000,
    WATCHDOG_TIMER_EXPIRED = 0x1000000,
    ESTOP_REQUESTED = 0x2000000,
    SPINOUT_DETECTED = 0x4000000,
    BRAKE_RESISTOR_DISARMED = 0x8000000,
    THERMISTOR_DISCONNECTED = 0x10000000,
    CALIBRATION_ERROR = 0x40000000
};

typedef enum{
    Odrive_mode_DISABLE = 0,
    Odrive_mode_TORQUE = 1,
    Odrive_mode_VELOCITY = 2,
} Odrive_mode_t;


typedef struct{
    uint8_t mode;
    float torque;
    float velocity;
}__attribute__((__packed__)) Odrive_power_t;

typedef struct{
    float vel_p;
    float vel_i;
}__attribute__((__packed__)) Odrive_param_t;

typedef struct{
    uint8_t is_connected :1;
    uint8_t heartbeat_received : 1;
    uint8_t error_received : 1;
    uint8_t encoder_estimate_received : 1;
    uint8_t torques_received : 1;
    uint8_t dummy : 3;
    uint32_t axis_error;
    uint8_t axis_state;
    uint8_t procedure_result;
    uint32_t active_errors;
    uint32_t disarm_reason;
    float pos_estimate;
    float vel_estimate;
    float torque_target;
    float torque_estimate;
}__attribute__((__packed__)) Odrive_sensor_t;

/// @brief Odriveのつながっている場所を登録する。
/// @param add_list 
void init(std::set<app_addr_t> add_list);

void send_param(ECId_t gw_id, ECPort_t port, id_t number, Odrive_param_t* param);
void set_power(ECId_t gw_id, ECPort_t port, id_t number, Odrive_power_t* power);
Odrive_sensor_t get_sensor(ECId_t gw_id, ECPort_t port, id_t number);
uint64_t get_max_interval(ECId_t gw_id, ECPort_t port, id_t number, bool is_reset);
}

class Odrive
{
public:
    enum class mode_t
    {
        disable = odrive::Odrive_mode_DISABLE,
        torque = odrive::Odrive_mode_TORQUE,
        velocity = odrive::Odrive_mode_VELOCITY
    };

    Odrive()
    {
        _gw_id = ECId_UNKNOWN;
        _port = ECPort_1;
        _id = id_t::_1;
        _direction = 1;
        _offset_pos = 0;
        _mode = (uint8_t)mode_t::disable;
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id OdriveのID（Odriveの設定で、(3<<3 | ID)のようにしておく）
    /// @param direction 回転方向
    /// @param mode モード
    void init(ECId_t gw_id, ECPort_t port, id_t id, direction_t direction, mode_t mode=mode_t::disable)
    {
        if(id_t::_4 < id)
        {
            logger::err_out("odrive", "Invalid ID(%d)", (uint8_t)id);
            return;
        }

        _gw_id = gw_id;
        _port = port;
        _id = id;

        if(direction == direction_t::forward)
        {
            _direction = 1;
        }else{
            _direction = -1;
        }
        
        _mode = (uint8_t)mode;

        logger::can_diag_bind(gw_id, port, id, "Odrive", [this](){
            logger::diagnostic_t diag;
            uint64_t interval_us = odrive::get_max_interval(_gw_id, _port, _id, true);
            if(this->_mode == odrive::Odrive_mode_t::Odrive_mode_VELOCITY){
                if(interval_us != 0 && (interval_us < (50*1000))){
                diag.status = true;
                }else{
                    diag.status = false;
                    diag.msg = "interval:" + std::to_string((float)interval_us / 1000);
                }
            }else{
                if(interval_us != 0 && (interval_us < (10*1000))){
                diag.status = true;
                }else{
                    diag.status = false;
                    diag.msg = "interval:" + std::to_string((float)interval_us / 1000);
                }
            }
            return diag;
        });
    }

    /// @brief Odriveの司令モードを設定する。Odrive自身のモードと一致している必要がある
    /// @param mode 
    void set_mode(mode_t mode)
    {
        _mode = (uint8_t)mode;
    }

    /// @brief 速度指定モードのPIDゲインを設定する
    /// @param kp デフォルト 0.163?
    /// @param ki デフォルト 0.333?
    void set_vel_gain(float p=0.163, float i=0.333)
    {
        odrive::Odrive_param_t param;
        param.vel_p = p;
        param.vel_i = i;
        odrive::send_param(_gw_id, _port, _id, &param);
    }

    /// @brief トルク（電流）指定で回す
    /// @param cur トルク
    void set_torque(float torque)
    {
        if(this->get_mode() != mode_t::torque)
        {
            this->set_mode(mode_t::torque);
        }

        odrive::Odrive_power_t power;
        power.mode = _mode;
        power.torque = torque * _direction;
        power.velocity = 0;
        odrive::set_power(_gw_id, _port, _id, &power);
        // TAGGER_INFO("csmbus", "odrive", "%d, %d, %d, %f", _gw_id, _port, _id, cur);
    }


    /// @brief 速度指定で回す
    /// @param velocity 速度
    /// @param torque トルク
    void set_velocity(float velocity, float torque = 0)
    {
        if(this->get_mode() != mode_t::velocity)
        {
            this->set_mode(mode_t::velocity);
        }

        odrive::Odrive_power_t power;
        power.mode = _mode;
        power.torque = torque * _direction;
        power.velocity = velocity * _direction;
        odrive::set_power(_gw_id, _port, _id, &power);
        // TAGGER_INFO("csmbus", "odrive", "%d, %d, %d, %f", _gw_id, _port, _id, cur);
    }


    /// @brief 角度が取れる。Odriveのencoder_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_pos(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        sens.pos_estimate -= _offset_pos;
        return sens.pos_estimate * _direction;
    }

    /// @brief 速度が取れる。Odriveのencoder_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_vel(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.vel_estimate  * _direction;
    }

    /// @brief トルク指令値？が取れる。Odriveのtorques_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_setting_torque(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.torque_target * _direction;
    }

    /// @brief トルクが取れる。Odriveのtorques_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_torque(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.torque_estimate * _direction;
    }

    mode_t get_mode(void)
    {
        return (mode_t)_mode;
    }

    void set_offset_pos(double offset=0.0)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        _offset_pos = sens.pos_estimate - (offset * _direction);
    }

    /// @brief ログを出力する。
    /// @param  
    void print_err(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        logger::info_out("odrive", "%d, %x, %x, %x, %x, %x", sens.is_connected, sens.axis_error, sens.axis_state, sens.procedure_result, sens.active_errors, sens.disarm_reason);
    }

    /// @brief Odriveが起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        if(sens.is_connected == 1)
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief Odriveが今つながっているか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=100)
    {
        uint64_t max_interval = odrive::get_max_interval(_gw_id, _port, _id, false);
        if(max_interval < (timeout_ms * 1000))
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief Odriveの起動しているインデックスリストを取得する
    /// @param  
    /// @return 0~3ビットがそれぞれ、id_1~id_4に対応している
    uint8_t wakeup_list(void)
    {
        uint8_t list = 0;
        for(size_t i = 0; i < 4; i++)
        {
            odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, (id_t)((int)id_t::_1 + i));
            if(sens.is_connected == 1)
            {
                list |= 0x01 << i;
            }
        }
        return list;
    }

private:
    ECId_t _gw_id;
    ECPort_t _port;
    id_t _id;

    int8_t _direction;
    float _offset_pos;

    uint8_t _mode;
};



class OdriveSensor
{
public:
    OdriveSensor()
    {
        _gw_id = ECId_UNKNOWN;
        _port = ECPort_1;
        _id = id_t::_1;
        _direction = 1;
    }

    void init(ECId_t gw_id, ECPort_t port, id_t id, direction_t act_direction, direction_t sens_direction)
    {
        if(id_t::_4 < id)
        {
            logger::err_out("odrive", "Invalid ID(%d)", (uint8_t)id);
            return;
        }

        _gw_id = gw_id;
        _port = port;
        _id = id;

        _offset_pos = 0;

        if(sens_direction == act_direction){
            _direction = 1;
        }else{
            _direction = -1;
        }
    }

    /// @brief 角度が取れる。Odriveのencoder_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_pos(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        sens.pos_estimate -= _offset_pos;
        return sens.pos_estimate * _direction;
    }

    /// @brief 速度が取れる。Odriveのencoder_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_vel(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.vel_estimate  * _direction;
    }

    /// @brief トルク指令値？が取れる。Odriveのtorques_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_setting_torque(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.torque_target * _direction;
    }

    /// @brief トルクが取れる。Odriveのtorques_msg_rateを設定する必要がある
    /// @param  
    /// @return 
    float get_torque(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        return sens.torque_estimate * _direction;
    }

    void set_offset_pos(double offset=0.0)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        _offset_pos = sens.pos_estimate - (offset * _direction);
    }

    /// @brief ログを出力する。
    /// @param  
    void print_err(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        logger::info_out("odrive", "%d, %x, %x, %x, %x, %x", sens.is_connected, sens.axis_error, sens.axis_state, sens.procedure_result, sens.active_errors, sens.disarm_reason);
    }

    /// @brief Odriveが起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, _id);
        if(sens.is_connected == 1)
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief Odriveが今つながっているか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=10)
    {
        uint64_t max_interval = odrive::get_max_interval(_gw_id, _port, _id, false);
        if(max_interval < (timeout_ms * 1000))
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief Odriveの起動しているインデックスリストを取得する
    /// @param  
    /// @return 0~3ビットがそれぞれ、id_1~id_4に対応している
    uint8_t wakeup_list(void)
    {
        uint8_t list = 0;
        for(size_t i = 0; i < 4; i++)
        {
            odrive::Odrive_sensor_t sens = odrive::get_sensor(_gw_id, _port, (id_t)((int)id_t::_1 + i));
            if(sens.is_connected == 1)
            {
                list |= 0x01 << i;
            }
        }
        return list;
    }

private:
    ECId_t _gw_id;
    ECPort_t _port;
    id_t _id;

    int8_t _direction;

    float _offset_pos;
};

}