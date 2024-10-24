#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth/ec_timer.hpp"

namespace csmbus
{

class DistanceSick : protected can_csmbus::Device
{
public:
    DistanceSick()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id 
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        laser_t laser;
        laser.laser[0] = 0;
        laser.laser[1] = 0;
        laser.laser[2] = 0;
        laser.laser[3] = 0;
        _laser_reg.init(laser);

        this->dev_init(gw_id, port, id);

        logger::can_diag_bind(gw_id, port, id, "DistanceSick", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(20);
            return diag;
        });
    }

    uint16_t distance(port_t port)
    {
        if((uint8_t)port < 4)
        {
            laser_t laser = _laser_reg.get_data().first;
            return laser.laser[(uint8_t)port];
        }else{
            return 0;
        }
    }

    /// @brief 起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        bool is_obtained = _laser_reg.is_obtained();
        return is_obtained;
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained = _laser_reg.is_obtained();
        auto timeout = _laser_reg.get_data().second;
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }

    RealTimer distance_with_duration(std::array<uint16_t, 4>* distances)
    {
        auto data = _laser_reg.get_data();
        distances->at(0) = data.first.laser[0];
        distances->at(1) = data.first.laser[1];
        distances->at(2) = data.first.laser[2];
        distances->at(3) = data.first.laser[3];
        return data.second;
    }

private:
    typedef struct{
        uint16_t laser[4];
    }__attribute__((__packed__)) laser_t;

private:
    RecvRegister<CCReg_0, laser_t> _laser_reg;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        _laser_reg.can_cb(reg, len, data);
    }
};

}