#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth/ec_timer.hpp"

namespace csmbus
{

class Switch : protected can_csmbus::Device
{
public:
    Switch()
    {
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id can_id（小基盤の点滅してる数）
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        switch_t sw;
        for(size_t i = 0; i < 7; i++)
        {
            set_sw(&sw.list1, i, false);
        }
        _switch_reg.init(sw);

        this->dev_init(gw_id, port, id);

        logger::can_diag_bind(gw_id, port, id, "Switch", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(50);
            return diag;
        });
    }

    bool get(port_t port)
    {
        switch_t sw = _switch_reg.get_data().first;
        return get_sw(&sw.list1, static_cast<uint32_t>(port));
    }

    /// @brief 起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        bool is_obtained = _switch_reg.is_obtained();
        return is_obtained;
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained = _switch_reg.is_obtained();
        auto timeout = _switch_reg.get_data().second;
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }

private:
    typedef struct{
        uint8_t list1;
    }__attribute__((__packed__)) switch_t;


    inline bool get_sw(uint8_t* list, int index){
        if (index < 0 || index > 7) {
            return false;
        }
        return (*list >> index) & 1;
    }

    // 指定されたインデックスにbool値を設定する関数
    inline void set_sw(uint8_t* list, int index, bool value) {
        if (index < 0 || index > 7) {
            return;
        }
        if (value) {
            *list |= (1 << index); // 指定されたビットを1に設定
        } else {
            *list &= ~(1 << index); // 指定されたビットを0に設定
        }
    }

private:
    RecvRegister<CCReg_0, switch_t> _switch_reg;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        _switch_reg.can_cb(reg, len, data);
    }
};

}