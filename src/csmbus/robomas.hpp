#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "eth/eth_csmbus.h"

#include "logger/logger.hpp"

namespace csmbus
{

namespace robomas
{

typedef enum{
    Robomas_mode_DISABLE = 0,
    Robomas_mode_CURRENT = 1,
    Robomas_mode_RPM = 2,
    Robomas_mode_ANGLE = 3,
} Robomas_mode_t;

typedef struct{
    uint8_t mode;
    int16_t cur;
    int16_t rpm;
    int64_t ang;
}__attribute__((__packed__)) Robomas_power_t;

typedef struct{
    int16_t max_cur;
    int8_t direction;
    float rpm_p;
    float rpm_i;
    float rpm_d;
    float ang_p;
    float ang_i;
    float ang_d;
    int32_t pid_i_max;
}__attribute__((__packed__)) Robomas_param_t;

typedef struct
    {
        uint8_t is_received : 1;
        uint8_t is_connected : 1;
        uint8_t dummy : 6;
        int16_t rpm;
        int16_t cur;
        int16_t set_cur;
        int64_t ang; // [8191 / rota]
    }__attribute__((__packed__)) Robomas_sensor_t;

/// @brief Robomasのつながっている場所を登録する。
/// @param add_list 
void init(std::set<app_addr_t> add_list);

void send_param(ECId_t gw_id, ECPort_t port, id_t number, Robomas_param_t* param);
void set_power(ECId_t gw_id, ECPort_t port, id_t number, Robomas_power_t* power);
Robomas_sensor_t get_sensor(ECId_t gw_id, ECPort_t port, id_t number);
uint64_t get_max_interval(ECId_t gw_id, ECPort_t port, id_t number, bool is_reset);
}

/// @brief ロボマスモータのクラス
// センサ情報のみを取得する場合は、RobomasSensorを使う
class Robomas
{
public:
    enum class mot_t
    {
        m2006 = 0,
        m3508 = 1
    };

    // コントロールモード（速度制御、角度制御のPIDはGatewayで実行される）
    enum class mode_t
    {
        disable = robomas::Robomas_mode_DISABLE,
        current = robomas::Robomas_mode_CURRENT,
        rpm = robomas::Robomas_mode_RPM,
        angle = robomas::Robomas_mode_ANGLE
    };

    Robomas()
    {
        _gw_id = ECId_UNKNOWN;
        _port = ECPort_1;
        _id = id_t::_1;
        _mot = mot_t::m2006;
        _mode = (uint8_t)mode_t::disable;
        _offset_rot = 0;
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id ロボますモータのID
    /// @param mot モータの種類
    /// @param direction 回転方向
    /// @param mode ロボマスモータの制御方法
    /// @param max_cur 最大電流
    /// @param rpm_p 速度PIDのP
    /// @param rpm_i 速度PIDのI
    /// @param rpm_d 速度PIDのD
    /// @param ang_p 角度PIDのP
    /// @param ang_i 角度PIDのI
    /// @param ang_d 角度PIDのD
    void init(ECId_t gw_id, ECPort_t port, id_t id, mot_t mot, direction_t direction, mode_t mode=mode_t::disable, float max_cur=-1, 
                    float rpm_p=5, float rpm_i=0.07, float rpm_d=0.05, float ang_p=5, float ang_i=0.0, float ang_d=0.0)
    {
        if(id_t::_6 < id)
        {
            logger::err_out("robomas", "Invalid ID(%d)", (uint8_t)id);
            return;
        }

        _gw_id = gw_id;
        _port = port;
        _id = id;
        _mot = mot;

        if(direction == direction_t::forward){
            _param.direction = 1;
        }else{
            _param.direction = -1;
        }
        _mode = (uint8_t)mode;
        if(max_cur < 0)
        {
            switch (mot)
            {
            case mot_t::m2006:
                max_cur = 10;
                break;
            
            case mot_t::m3508:
                max_cur = 20;
                break;
            }
        }
        _param.max_cur = convert_cur(max_cur);
        _param.rpm_p = rpm_p;
        _param.rpm_i = rpm_i;
        _param.rpm_d = rpm_d;
        _param.ang_p = ang_p;
        _param.ang_i = ang_i;
        _param.ang_d = ang_d;
        _param.pid_i_max = 9000;
        robomas::send_param(gw_id, port, id, &_param);

        logger::can_diag_bind(gw_id, port, id, "Robomas", [this](){
            logger::diagnostic_t diag;
            uint64_t interval_us = robomas::get_max_interval(_gw_id, _port, _id, true);
            if(interval_us != 0 && (interval_us < (10*1000))){
                diag.status = true;
            }else{
                diag.status = false;
                diag.msg = "interval:" + std::to_string((float)interval_us / 1000);
            }
            return diag;
        });
    }

    /// @brief モード指定（しなくても自動で切り替わる）
    /// @param mode 
    void set_mode(mode_t mode)
    {
        _mode = (uint8_t)mode;
    }

    /// @brief 速度PIDのpidパラメータ設定
    /// @param rpm_p 
    /// @param rpm_i 
    /// @param rpm_d 
    void set_rpm_pid(float rpm_p=5, float rpm_i=0.07, float rpm_d=0.05)
    {
        _param.rpm_p = rpm_p;
        _param.rpm_i = rpm_i;
        _param.rpm_d = rpm_d;
        robomas::send_param(_gw_id, _port, _id, &_param);
    }

    /// @brief 角度PIDのpidパラメータ設定
    /// @param ang_p 
    /// @param ang_i 
    /// @param ang_d 
    void set_ang_pid(float ang_p=5, float ang_i=0.0, float ang_d=0.0)
    {
        _param.ang_p = ang_p;
        _param.ang_i = ang_i;
        _param.ang_d = ang_d;
        robomas::send_param(_gw_id, _port, _id, &_param);
    }

    /// @brief ロボマスのPIDの積分値の上限を指定する
    /// @param  
    void set_integral_cut(int32_t max_integral=9000)
    {
        _param.pid_i_max = max_integral;
        robomas::send_param(_gw_id, _port, _id, &_param);
    }

    /// @brief 電流制御する。もし、mode_tが違ったら、自動的に上書きされる
    /// @param cur 電流値[A]
    void set_current(float cur)
    {
        if(this->get_mode() != mode_t::current)
        {
            this->set_mode(mode_t::current);
        }

        robomas::Robomas_power_t power;
        power.mode = _mode;
        power.cur = convert_cur(cur);
        power.rpm = 0;
        power.ang = 0;
        robomas::set_power(_gw_id, _port, _id, &power);
        // TAGGER_INFO("csmbus", "robomas", "%d, %d, %d, %f", _gw_id, _port, _id, cur);
    }

    /// @brief 速度制御する。もし、mode_tが違ったら、自動的に上書きされる
    /// @param rpm 速度（尻）
    /// @param base_cur 速度のPIDの制御値にbase_curが足される
    void set_rpm(int32_t rpm, float base_cur = 0)
    {
        if(this->get_mode() != mode_t::rpm)
        {
            this->set_mode(mode_t::rpm);
        }

        robomas::Robomas_power_t power;
        power.mode = _mode;
        power.cur = convert_cur(base_cur);
        power.rpm = cut_max_rpm(rpm);
        power.ang = 0;
        robomas::set_power(_gw_id, _port, _id, &power);
        // TAGGER_INFO("csmbus", "robomas", "%d, %d, %d, %d, %f", _gw_id, _port, _id, rpm, base_cur);
    }


    /// @brief 角度制御する。もし、mode_tが違ったら、自動的に上書きされる
    /// @param geared_theta ギアヘッドの後の角度
    /// @param target_rpm   その角度に行くまでの最大速度
    /// @param base_cur     速度のPIDの制御値にbase_curが足される
    void set_angle(double geared_theta, int32_t target_rpm = INT16_MAX, float base_cur = 0)
    {
        if(this->get_mode() != mode_t::angle)
        {
            this->set_mode(mode_t::angle);
        }

        geared_theta /= this->get_gear_rate();
        geared_theta = (geared_theta + this->_offset_rot) / (M_PI * 2) * 8192;

        robomas::Robomas_power_t power;
        power.mode = _mode;
        power.cur = convert_cur(base_cur);
        power.rpm = cut_max_rpm(target_rpm);
        power.ang = (int64_t)round(geared_theta);
        robomas::set_power(_gw_id, _port, _id, &power);
        // TAGGER_INFO("csmbus", "robomas", "%d, %d, %d, %f, %d, %f", _gw_id, _port, _id, geared_theta, target_rpm, base_cur);
    }

    /// @brief 
    /// @param  
    /// @return　速度（尻） 
    int32_t get_rpm(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        return sens.rpm;
    }

    /// @brief 
    /// @param  
    /// @return　速度（ギアヘッド）
    double get_geared_rpm(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rpm = sens.rpm;
        rpm = rpm * this->get_gear_rate();
        return rpm;
    }

    /// @brief モータに流れている電流値
    /// @param  
    /// @return 電流値[A]
    float get_current(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        float cur = sens.cur;
        switch(_mot)
        {
        case mot_t::m2006:
            cur = cur * ((float)10 / (float)10000);
            break;
        case mot_t::m3508:
            cur = cur * ((float)20 / (float)16384);
            break;
        }
        return cur;
    }


    /// @brief Ethernet基盤が指定している電流値
    /// @param  
    /// @return 電流値[A]
    float get_set_cur(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        float cur = sens.set_cur;
        switch(_mot)
        {
        case mot_t::m2006:
            cur = cur * ((float)10 / (float)10000);
            break;
        case mot_t::m3508:
            cur = cur * ((float)20 / (float)16384);
            break;
        }
        return cur;
    }

    /// @brief 
    /// @param  
    /// @return 角度（尻）[2PI / rotation]
    double get_rot(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rot = sens.ang;
        rot = (rot / 8191) * M_PI * 2;
        rot -= _offset_rot;
        return rot;
    }

    /// @brief 
    /// @param  
    /// @return 角度（ギアヘッド）[2PI / rotation]
    double get_geared_rot(void)
    {
        double rot = this->get_rot() * this->get_gear_rate();
        return rot;
    }


    /// @brief 
    /// @param  
    /// @return ギア比
    double get_gear_rate(void)
    {
        switch(_mot)
        {
        case mot_t::m2006:
            return 1 / (double)36;
            break;
        case mot_t::m3508:
            return 187 / (double)3591;
            break;
        }
        return 1;
    }


    /// @brief ロボますモータをゼロセットする
    /// @param offset ゼロ値[2PI / rotation]
    void set_offset_rot(double offset=0.0)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rot = ((double)sens.ang / (double)8191) * M_PI * 2;;
        this->_offset_rot = rot - (offset / this->get_gear_rate());
    }


    /// @brief 現在のモードを取得する
    /// @param  
    /// @return 
    mode_t get_mode(void)
    {
        return (mode_t)_mode;
    }

    /// @brief ロボマスモータからのレスポンスがあったか
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        if(sens.is_connected == 1)
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief ロボマスモータが今つながっているか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=100)
    {
        uint64_t max_interval = robomas::get_max_interval(_gw_id, _port, _id, false);
        if(max_interval < (timeout_ms * 1000))
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief ロボマスモータのつながってるインデックすを取得する
    /// @param  
    /// @return 0~5ビットがそれぞれ、id_1~id_6に対応している
    uint8_t wakeup_list(void)
    {
        uint8_t list = 0;
        for(size_t i = 0; i < 6; i++)
        {
            robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, (id_t)((int)id_t::_1 + i));
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
    mot_t _mot;

    uint8_t _mode;
    robomas::Robomas_param_t _param;

    float _offset_rot;
    
    int16_t convert_cur(float cur)
    {
        int32_t result;
        switch(_mot)
        {
        case mot_t::m2006:
            result = (int)(cur * (10000 / 10));
            if(result < -10000)
            {
                result = -10000;
            }else if(10000 < result){
                result = 10000;
            }
            break;

        case mot_t::m3508:
            result = (int)(cur * (16384 / 20));
            if(result < -16384)
            {
                result = -16384;
            }else if(16384 < result){
                result = 16384;
            }
            break;
        default:
            result = 0;
            break;
        }
        return result;
    }

    int16_t cut_max_rpm(int32_t rpm)
    {
        int32_t result;
        switch(_mot)
        {
        case mot_t::m2006:
            result = rpm;
            if(result < -20200)
            {
                result = -20200;
            }else if(20200 < result){
                result = 20200;
            }
            break;

        case mot_t::m3508:
            result = rpm;
            if(result < -10000)
            {
                result = -10000;
            }else if(10000 < result){
                result = 10000;
            }
            break;
        default:
            result = 0;
            break;
        }
        return result;
    }
};


/// @brief ロボますの回転や電流のみを参照するクラス（出力に影響を与えない）
class RobomasSensor
{
public:
    RobomasSensor()
    {
        _gw_id = ECId_UNKNOWN;
        _port = ECPort_1;
        _id = id_t::_1;
        _mot = Robomas::mot_t::m2006;
        _direction = 1;
        _offset_rot = 0;
    }
    
    void init(ECId_t gw_id, ECPort_t port, id_t id, Robomas::mot_t mot, direction_t act_direction, direction_t sens_direction)
    {
        if(id_t::_6 < id)
        {
            logger::err_out("robomas", "Invalid ID(%d)", (uint8_t)id);
            return;
        }

        _gw_id = gw_id;
        _port = port;
        _id = id;
        _mot = mot;

        if(sens_direction == act_direction){
            _direction = 1;
        }else{
            _direction = -1;
        }
    }


    /// @brief 
    /// @param  
    /// @return　速度（尻） 
    int32_t get_rpm(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        return sens.rpm * _direction;
    }

    /// @brief 
    /// @param  
    /// @return　速度（ギアヘッド）
    double get_geared_rpm(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rpm = sens.rpm;
        rpm = rpm * this->get_gear_rate();
        return rpm  * _direction;
    }

    /// @brief モータに流れている電流値
    /// @param  
    /// @return 電流値[A]
    float get_current(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        float cur = sens.cur;
        switch(_mot)
        {
        case Robomas::mot_t::m2006:
            cur = cur * ((float)10 / (float)10000);
            break;
        case Robomas::mot_t::m3508:
            cur = cur * ((float)20 / (float)16384);
            break;
        }
        return cur * _direction;
    }


    /// @brief Ethernet基盤が指定している電流値
    /// @param  
    /// @return 電流値[A]
    float get_set_cur(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        float cur = sens.set_cur;
        switch(_mot)
        {
        case Robomas::mot_t::m2006:
            cur = cur * ((float)10 / (float)10000);
            break;
        case Robomas::mot_t::m3508:
            cur = cur * ((float)20 / (float)16384);
            break;
        }
        return cur * _direction;
    }

    /// @brief 
    /// @param  
    /// @return 角度（尻）[2PI / rotation]
    double get_rot(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rot = sens.ang;
        rot = (rot / 8191) * M_PI * 2;
        rot -= _offset_rot;
        return rot * _direction;
    }

    /// @brief 
    /// @param  
    /// @return 角度（ギアヘッド）[2PI / rotation]
    double get_geared_rot(void)
    {
        double rot = this->get_rot() * this->get_gear_rate();
        return rot;
    }


    /// @brief 
    /// @param  
    /// @return ギア比
    double get_gear_rate(void)
    {
        switch(_mot)
        {
        case Robomas::mot_t::m2006:
            return 1 / (double)36;
            break;
        case Robomas::mot_t::m3508:
            return 187 / (double)3591;
            break;
        }
        return 1;
    }


    /// @brief ロボますモータをゼロセットする
    /// @param offset ゼロ値[2PI / rotation]
    void set_offset_rot(double offset=0.0)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        double rot = ((double)sens.ang / (double)8191) * M_PI * 2;;
        this->_offset_rot = rot - ((offset / this->get_gear_rate()) * _direction);
    }

    /// @brief ロボマスモータが起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, _id);
        if(sens.is_connected == 1)
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief ロボマスモータが今つながっているか
    /// @param  
    /// @return 
    bool is_connecting(uint32_t timeout_ms=10)
    {
        uint64_t max_interval = robomas::get_max_interval(_gw_id, _port, _id, false);
        if(max_interval < (timeout_ms * 1000))
        {
            return true;
        }else{
            return false;
        }
    }

    /// @brief ロボマスモータのつながってるインデックすを取得する
    /// @param  
    /// @return 0~5ビットがそれぞれ、id_1~id_6に対応している
    uint8_t wakeup_list(void)
    {
        uint8_t list = 0;
        for(size_t i = 0; i < 6; i++)
        {
            robomas::Robomas_sensor_t sens = robomas::get_sensor(_gw_id, _port, (id_t)((int)id_t::_1 + i));
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
    Robomas::mot_t _mot;

    int8_t _direction;

    float _offset_rot;
};

}