#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can_csmbus/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth_csmbus/ec_timer.hpp"

namespace csmbus
{

class GyroYaw : protected can_csmbus::Device
{
public:
    GyroYaw()
    {
        _offset_yaw = 0;
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id can_id（小基盤の点滅してる数）
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        yaw_t yaw;
        yaw.gyro = 0;
        yaw.angle = 0;
        yaw.spin_count = 0;
        _yaw_reg.init(yaw);

        this->dev_init(gw_id, port, id);

        logger::can_diag_bind(gw_id, port, id, "Gyro", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(20);
            return diag;
        });
    }

    /// @brief [度]
    /// @param  
    /// @return 
    float get_angle(void)
    {
        yaw_t yaw = _yaw_reg.get_data().first;
        float angle = (float)yaw.angle / 100 + (yaw.spin_count * 360);
        angle -= _offset_yaw;
        return angle;
    }

    /// @brief [度]
    /// @param  
    /// @return 
    float get_gyro(void)
    {
        yaw_t yaw = _yaw_reg.get_data().first;
        return (float)yaw.gyro / 100;
    }

    std::pair<float, float> get_acc(void){
        yaw_t yaw = _yaw_reg.get_data().first;
        float angle = yaw.acc_angle * (180.0f / 127);
        float acc = yaw.acc / 100.0f;
        return std::make_pair(angle, acc);
    }

    RealTimer get_yaw_with_tim(float* gyro, float* angle)
    {
        auto data = _yaw_reg.get_data();

        *angle = (float)data.first.angle / 100 + (data.first.spin_count * 360);
        *angle -= _offset_yaw;
        *gyro = (float)data.first.gyro / 100;
        return data.second;
    }

    /// @brief 角度をゼロセットする
    /// @param  
    void offset_angle(void)
    {
        yaw_t yaw = _yaw_reg.get_data().first;
        _offset_yaw = (float)yaw.angle / 100 + (yaw.spin_count * 360);
    }

    /// @brief 起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        bool is_obtained = _yaw_reg.is_obtained();
        return is_obtained;
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained = _yaw_reg.is_obtained();
        auto timeout = _yaw_reg.get_data().second;
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }

private:
    /// @brief gyro：角速度、angle：角度（振り切れる）、spin_count：回転数
    typedef struct{
        int16_t gyro;
        int16_t angle;
        int8_t spin_count;
        int8_t acc_angle;
        uint16_t acc;
    }__attribute__((__packed__)) yaw_t;

private:
    RecvRegister<CCReg_0, yaw_t> _yaw_reg;

    float _offset_yaw;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        _yaw_reg.can_cb(reg, len, data);
    }
};

}