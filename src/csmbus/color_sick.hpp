#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth/ec_timer.hpp"

namespace csmbus
{

class ColorSick : protected can_csmbus::Device
{
public:
    ColorSick()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id 
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        color_t color;
        color.l = 0;
        color.a = 0;
        color.b = 0;
        color.distance = 0;
        _color1_reg.init(color);
        _color2_reg.init(color);
        _color3_reg.init(color);
        _color4_reg.init(color);

        this->dev_init(gw_id, port, id);

        logger::can_diag_bind(gw_id, port, id, "DistanceColor", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(20);
            return diag;
        });
    }

    /// @brief LAB空間のカラーを取得する
    /// @param port ポート番号
    /// @return 
    std::array<float, 3> lab(port_t port)
    {
        std::pair<color_t, RealTimer> data;
        switch(port)
        {
        case port_t::_1:
            data = _color1_reg.get_data();
            break;
            
        case port_t::_2:
            data = _color2_reg.get_data();
            break;

        case port_t::_3:
            data = _color3_reg.get_data();
            break;

        case port_t::_4:
            data = _color4_reg.get_data();
            break;

        default:
            data.first.l = 0;
            data.first.a = 0;
            data.first.b = 0;
            break;
        }

        std::array<float, 3> lab;
        data = _color1_reg.get_data();
        lab[0] = (float)data.first.l / 100;
        lab[1] = (float)data.first.a / 100;
        lab[2] = (float)data.first.b / 100;
        return lab;
    }

    /// @brief 起動しているかがわかる
    /// @param port ポート番号
    /// @return 
    bool is_wakeup(void)
    {
        return _color1_reg.is_obtained();
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained = _color1_reg.is_obtained();
        auto timeout = _color1_reg.get_data().second;
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }
    
private:
    typedef struct{
        int16_t l;
        int16_t a;
        int16_t b;
        uint16_t distance;
    }__attribute__((__packed__)) color_t;

private:
    RecvRegister<CCReg_1, color_t> _color1_reg;
    RecvRegister<CCReg_2, color_t> _color2_reg;
    RecvRegister<CCReg_3, color_t> _color3_reg;
    RecvRegister<CCReg_4, color_t> _color4_reg;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        _color1_reg.can_cb(reg, len, data);
        _color2_reg.can_cb(reg, len, data);
        _color3_reg.can_cb(reg, len, data);
        _color4_reg.can_cb(reg, len, data);
    }
};

}