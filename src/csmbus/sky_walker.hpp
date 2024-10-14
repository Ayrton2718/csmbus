#pragma once

#include <vector>
#include "smbus_type.hpp"
#include "can_smbus/cs_io.hpp"

#include "logger/logger.hpp"
#include <tut_tool/tt_timer.hpp>

namespace smbus
{

class SkyWalker : protected can_smbus::Device
{
public:
    SkyWalker()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id 
    void init(ESId_t gw_id, ESPort_t port, id_t id, port_t dev_port)
    {
        this->dev_init(gw_id, port, id);
        _port = dev_port;

        logger::can_diag_bind(gw_id, port, id, "SkyWalker", [this](){
            logger::diagnostic_t diag;
            diag.status = true;
            return diag;
        });
    }

    /// @brief 回す
    /// @param percent 0 ~ 100
    void set_power(float percent)
    {
        if(percent < 0 || 100 < percent)
        {
            percent = 0;
            logger::err_out("can_smbus", "Invalid Power(%f)", percent);
        }

        sky_t sky;
        sky.duty = (uint8_t)(255 * percent / 100);
        switch (_port)
        {
        case port_t::_1:
            _sky1_reg.send(this, sky);
            break;

        case port_t::_2:
            _sky2_reg.send(this, sky);
            break;
        
        default:
            logger::err_out("can_smbus", "Invalid Port(%d)", _port);
            break;
        }
    }

private:
    typedef struct{
        uint8_t duty;
    }__attribute__((__packed__)) sky_t;

    port_t _port;

    send_reg_t<ESReg_0, sky_t> _sky1_reg;
    send_reg_t<ESReg_1, sky_t> _sky2_reg;

private:
    virtual void can_callback(CSReg_t reg, uint8_t len, const uint8_t* data)
    {
        (void)(reg);
        (void)(len);
        (void)(data);
    }
};

}