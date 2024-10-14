#pragma once

#include <vector>
#include "csmbus_type.hpp"
#include "can_csmbus/cc_io.hpp"

#include "logger/logger.hpp"
#include "eth_csmbus/ec_timer.hpp"

namespace csmbus
{

class AbsEncoder : protected can_csmbus::Device
{
public:
    AbsEncoder()
    {
        _count.angle = 0;
        _count.rot_count = 0;

        _is_obtained = false;

        _offset_rad = 0;
    }

    /// @brief 
    /// @param gw_id 
    /// @param port 
    /// @param id can_id（小基盤の点滅してる数）
    void init(ECId_t gw_id, ECPort_t port, id_t id)
    {
        this->dev_init(gw_id, port, id);

        _count.angle = 0;
        _count.rot_count = 0;

        _is_obtained = false;
        _timeout.start();

        logger::can_diag_bind(gw_id, port, id, "AbsEncoder", [this](){
            logger::diagnostic_t diag;
            diag.status = this->is_connecting(10);
            return diag;
        });
    }

    /// @brief　絶対角を取得する（振り切れる）
    /// @param  
    /// @return ラジアン
    double abs_rad(void)
    {
        count_t count = __get_count();
        double angle = ((double)count.angle / 16383) * M_PI * 2;
        return angle;
    }

    /// @brief 絶対角を取得する（振り切れる）
    /// @param  
    /// @return 度
    double abs_degree(void)
    {
        return abs_rad() * 180 / M_PI;
    }

    /// @brief　振り切れない
    /// @param  
    /// @return ラジアン
    double rot_rad(void)
    {
        count_t count = __get_count();
        double angle = ((double)count.angle / 16383 + count.rot_count) * M_PI * 2;
        return angle - _offset_rad;
    }

    /// @brief 振り切れない
    /// @param  
    /// @return 度
    double rot_degree(void)
    {
        return rot_rad() * 180 / M_PI;
    }

    /// @brief rot_rad/rot_degreeの原点をオフセット
    /// @param offset_rad 現在の角度をoffset_radとする（ラジアン）
    void offset_rot(double offset_rad = 0)
    {
        count_t count = __get_count();
        double angle = ((double)count.angle / 16383 + count.rot_count) * M_PI * 2;
        _offset_rad = angle - offset_rad;
    }

    /// @brief rot_rad/rot_degreeの原点をオフセット（絶対角度）
    /// @param offset_abs 現在の絶対角度をoffset_radとする（ラジアン）
    void offset_abs(double offset_rad = 0)
    {
        count_t count = __get_count();
        double angle = ((double)count.angle / 16383) * M_PI * 2;
        _offset_rad = angle - offset_rad;
    }

    RealTimer get_rot_with_tim(double* rot)
    {
        count_t count;
        auto tim = __get_count_with_tim(&count);
        double angle = ((double)count.angle / 16383 + count.rot_count) * M_PI * 2;
        *rot = angle - _offset_rad;
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
        int16_t rot_count;
        uint16_t angle;
        uint8_t checksum;
    }__attribute__((__packed__)) count_t;

    uint8_t calc_checksum(const count_t* count){
        const uint8_t* data = (const uint8_t*)count;
        return data[0] + data[1] + data[2] + data[3];
    }

private:
    std::mutex _locker;

    double _offset_rad;
    count_t _count;

    bool _is_obtained;
    RealTimer _timeout;

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

    RealTimer __get_timeout(void)
    {
        std::lock_guard<std::mutex> lock(_locker);
        return _timeout;
    }

    RealTimer __get_count_with_tim(count_t* count)
    {
        std::lock_guard<std::mutex> lock(_locker);
        *count = _count;
        return _timeout;
    }

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data)
    {
        if((reg == CCReg_0) && (len == sizeof(count_t)))
        {
            std::lock_guard<std::mutex> lock(_locker);
            const count_t* data_count = (const count_t*)data;
            if(calc_checksum(data_count) == data_count->checksum){
                _count = *data_count;
                _is_obtained = true;
                _timeout.reset();
            }else{
                logger::err_out("can_csmbus", "Invalid data gw_id(%d),port(%d),id(%d)", _gw_id, _port, _can_id);
            }
        }
    }
};

}