#pragma once

#include <vector>
#include "smbus_type.hpp"
#include "can_smbus/cs_io.hpp"

#include "logger/logger.hpp"
#include <tut_tool/tt_timer.hpp>

namespace smbus
{

class IncrEncoder : protected can_smbus::Device
{
public:
    // PPR : Pulse per rotation.
    enum struct ppr_t
    {
        // : Pattern(1~4), 1 is on.
        // Recommended. Maximum RPM is Allowed 15000.
        _48 = 48,         // : 1111
        _96 = 96,         // : 1101
        _100 = 100,       // : 0111
        _125 = 125,       // : 1011
        _200 = 200,       // : 0101
        _250 = 250,       // : 1001
        _256 = 256,       // : 0011
        _512 = 512,       // : 0001

        // Not recommended. Maximum RPM is Allowed 7500.
        _192 = 192,       // : 1110
        _384 = 384,       // : 1100
        _400 = 400,       // : 0110
        _500 = 500,       // : 1010
        _800 = 800,       // : 0100
        _1000 = 1000,     // : 1000
        _1024 = 1024,     // : 0010
        _2048 = 2048,     // : 0000
    };
public:
    IncrEncoder()
    {
        _count.count = 0;

        _is_obtained = false;

        _offset_count = 0;
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id can_id（小基盤の点滅してる数）
    /// @param ppr 一回展で何パルスか？
    /// @param direction 回転方向
    void init(ESId_t gw_id, ESPort_t port, id_t id, ppr_t ppr=ppr_t::_125, direction_t direction=direction_t::forward)
    {
        this->dev_init(gw_id, port, id);

        _ppr = ppr;
        if(direction == direction_t::forward){
            _direct = 1; 
        }else{  
            _direct = -1;
        }

        _count.count = 0;

        _offset_count = 0;
        _is_obtained = false;
        _timeout.start();

        logger::can_diag_bind(gw_id, port, id, "IncrEncoder", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(10);
            return diag;
        });
    }

    /// @brief エンコーダの直接得られた値
    /// @param  
    /// @return 
    int32_t cnt(void)
    {
        count_t count = __get_count();
        return count.count * _direct;
    }

    /// @brief ラジアンを受け取る（offsetで引かれた値）
    /// @param  
    /// @return ラジアン
    double angle_rot(void)
    {
        return ((double)(this->cnt() - _offset_count) / (double)_ppr) * M_PI * 2;
    }

    /// @brief 角度（度）を受け取る（offsetで引かれた値）
    /// @param  
    /// @return 度
    double angle_degree(void)
    {
        return ((double)(this->cnt() - _offset_count) / (double)_ppr) * 360;
    }

    /// @brief 0セット
    /// @param  
    void offset_rot(void)
    {
        _offset_count = this->cnt();
    }

    tut_tool::RealTimer get_data_with_tim(double* rot, double* pps)
    {
        count_t count;
        auto tim = __get_count_with_tim(&count);
        *rot = ((double)(count.count - _offset_count) / (double)_ppr) * M_PI * 2;
        *rot *= _direct;
        *pps = ((((double)count.pulse_ms / 100) * 1000.0f) / (double)_ppr) * M_PI * 2;
        *pps *= _direct;
        return tim;
    }

    /// @brief 起動しているかがわかる
    /// @param  
    /// @return 
    bool is_wakeup(void)
    {
        bool is_obtained = __get_is_obtained();
        return is_obtained;
    }

    /// @brief 現在つながっているかどうか
    /// @param timeout_ms タイムアウト時間を設定[ms]
    /// @return 
    bool is_connecting(uint32_t timeout_ms=50)
    {
        bool is_obtained = __get_is_obtained();
        auto timeout = __get_timeout();
        if(is_obtained && (timeout.getMs() < timeout_ms))
        {
            return true;
        }else{
            return false;
        }
    }

private:
    typedef struct{
        int32_t count; //4
        int16_t pulse_ms; // pulse per milli second //2
        uint8_t checksum;
    }__attribute__((__packed__)) count_t;

    uint8_t calc_checksum(const count_t* count){
        const uint8_t* data = (const uint8_t*)count;
        return data[0] + data[1] + data[2] + data[3] + data[4] + data[5];
    }

private:
    std::mutex _locker;

    ppr_t _ppr;
    int8_t _direct;

    int32_t _offset_count;
    count_t _count;

    bool _is_obtained;
    tut_tool::RealTimer _timeout;

    count_t __get_count(void)
    {
        std::lock_guard<std::mutex> lock(_locker);
        return _count;
    }

    bool __get_is_obtained(void)
    {
        std::lock_guard<std::mutex> lock(_locker);
        return _is_obtained;
    }

    tut_tool::RealTimer __get_timeout(void)
    {
        std::lock_guard<std::mutex> lock(_locker);
        return _timeout;
    }

    tut_tool::RealTimer __get_count_with_tim(count_t* count)
    {
        std::lock_guard<std::mutex> lock(_locker);
        *count = _count;
        return _timeout;
    }

    virtual void can_callback(CSReg_t reg, uint8_t len, const uint8_t* data)
    {
        if((reg == CSReg_0) && (len == sizeof(count_t)))
        {
            std::lock_guard<std::mutex> lock(_locker);
            const count_t* data_count = (const count_t*)data;
            if(calc_checksum(data_count) == data_count->checksum){
                _count = *data_count;
                _is_obtained = true;
                _timeout.reset();
            }else{
                logger::err_out("can_smbus", "Invalid data gw_id(%d),port(%d),id(%d)", _gw_id, _port, _can_id);
            }
        }
    }
};

}