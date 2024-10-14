#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can_csmbus/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth_csmbus/ec_timer.hpp"

namespace csmbus
{

class LedTape : protected can_csmbus::Device
{
public:
    LedTape()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id 
    /// @param send_cycle_ms 送信周期
    void init(ECId_t gw_id, ECPort_t port, id_t id, uint32_t send_cycle_ms=100)
    {
        this->dev_init(gw_id, port, id);

        _send_cycle_ms = send_cycle_ms;
        _tim.start();

        _befor_send.r = 0;
        _befor_send.g = 0;
        _befor_send.b = 0;
        _befor_send.hz = 0;
    }

    /// @brief ledを設定
    /// @param r 0 ~ 255
    /// @param g 0 ~ 255
    /// @param b 0 ~ 255
    /// @param hz 0 ~ 25 0は常時点灯 
    void set_rgb(uint8_t r, uint8_t g, uint8_t b, float hz)
    {
        uint8_t set_hz = static_cast<uint8_t>(hz * 10);
        led_tape_t rgb = {r, g, b, set_hz};
        if(!is_same_order(&rgb, &_befor_send) || _send_cycle_ms < _tim.getMs()){
            _tim.reset();
            _rgb_reg.send(this, rgb);
            // logger::info_out("led", "send, %d, %d, %d, %d", rgb.r, rgb.g, rgb.b, rgb.hz);
        }
    }

private:
    typedef struct{
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t hz; // hz * 10
    }__attribute__((__packed__)) led_tape_t;

    RealTimer _tim;
    uint32_t _send_cycle_ms;

    led_tape_t _befor_send;

    send_reg_t<ECReg_0, led_tape_t> _rgb_reg;

private:
    bool is_same_order(led_tape_t* now, led_tape_t* befor){
        if(befor->r != now->r || befor->g != now->g || befor->b != now->b || befor->hz != now->hz){
            *befor = *now;
            return false;
        }else{
            return true;
        }
    }

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        (void)(reg);
        (void)(len);
        (void)(data);
    }
};

}