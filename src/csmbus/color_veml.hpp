#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can_csmbus/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth_csmbus/ec_timer.hpp"

namespace csmbus
{

class ColorVeml : protected can_csmbus::Device
{
public:
    ColorVeml()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id can_id（小基盤の点滅してる数）
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        colors_t colors;
        colors.red = 0;
        colors.blue = 0;
        colors.clear = 0;
        _color_reg.init(colors);

        this->dev_init(gw_id, port, id);

        logger::can_diag_bind(gw_id, port, id, "ColorVeml", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting();
            return diag;
        });
    }

    /// @brief rgbcを取得する
    /// @param  
    /// @return [uW/cm^2] ?
    std::array<float, 3> rbc(void)
    {
        colors_t colors = _color_reg.get_data().first;

        std::array<float, 3> result;
        result[0] = (float)colors.red / 41;
        result[1] = (float)colors.blue / 34;
        result[2] = (float)colors.clear / 57;
        return result;
    }

    /// @brief 起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        bool is_obtained = _color_reg.is_obtained();
        return is_obtained;
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained =  _color_reg.is_obtained();
        auto timeout =  _color_reg.get_data().second;
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }

private:
    typedef struct{
        int16_t red;
        int16_t blue;
        int16_t clear;
    }__attribute__((__packed__)) colors_t;

private:
    RecvRegister<CCReg_0, colors_t> _color_reg;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        _color_reg.can_cb(reg, len, data);
    }
};

}