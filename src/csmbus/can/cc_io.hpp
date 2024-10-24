#pragma once

#include "cc_type.h"
#include <vector>

#include "../csmbus_type.hpp"
#include "../eth/eth_csmbus.h"
#include "../logger/logger.hpp"


namespace csmbus::can_csmbus
{

namespace io
{

typedef struct{
    uint16_t can_id;
    uint8_t len;
    uint8_t data[8];
}__attribute__((__packed__)) CanSMBus_packet_t;

typedef struct{
    uint8_t             count;
    CanSMBus_packet_t   packet[4];
}__attribute__((__packed__)) CanSMBus_m2s_t;

typedef struct{
    uint8_t             count;
    CanSMBus_packet_t   packet[4];
}__attribute__((__packed__)) CanSMBus_s2m_t;

typedef std::function<void(uint16_t raw_reg, uint8_t len, const uint8_t* data)> can_callback_t;

void init(std::set<app_addr_t> add_list);

void bind(ECId_t gw_id, ECPort_t port, CCId_t can_id, can_callback_t callback);
void send(ECId_t gw_id, ECPort_t port, CCId_t can_id, uint16_t raw_reg, uint8_t len, const uint8_t* data);
}


class Device
{
private:
    void callback(uint16_t raw_reg, uint8_t len, const uint8_t* data)
    {
        if(CCTYPE_IS_SYS_REG(raw_reg))
        {
            // sys
            if(CCTYPE_IS_WRITE_REG(raw_reg))
            {
                // CCReg_t reg = CCTYPE_GET_SYS_REG(raw_reg);
            }
        }
        else
        {
            // user
            if(CCTYPE_IS_WRITE_REG(raw_reg))
            {
                CCReg_t reg = CCTYPE_GET_USER_REG(raw_reg);
                this->can_callback(reg, len, data);
            }
        }
    }

protected:
    template <CCReg_t REG, typename T>
    class RecvRegister
    {
    private:
        volatile uint8_t _flip_flg = 0;
        T       _data[2];
        bool    _is_obtained = false;
        RealTimer _timeout;
    
    public:
        RecvRegister(){
            _flip_flg = 0;
            memset(&_data[0], 0x00, sizeof(T));
            memset(&_data[1], 0x00, sizeof(T));
            _is_obtained = false;
            _timeout.start();
        }

        void init(T def){
            memcpy(&_data[0], &def, sizeof(T));
            memcpy(&_data[1], &def, sizeof(T));
        }

        std::pair<T, RealTimer> get_data(void){
            return {_data[_flip_flg], _timeout};
        }

        bool is_obtained(void){
            return _is_obtained;
        }

        void can_cb(CCReg_t reg, uint8_t len, const uint8_t* data){
            if((reg == REG) && (len == sizeof(T)))
            {
                uint8_t write_index = (_flip_flg + 1) % 2;
                _data[write_index] = *((const T*)data);
                _flip_flg = write_index;
                
                _is_obtained = true;
                _timeout.reset();
            }
        }
    };

protected:
    ECId_t      _gw_id;
    ECPort_t    _port;
    CCId_t      _can_id;

    virtual void can_callback(CCReg_t reg, uint8_t len, const uint8_t* data) = 0;

    template <ECReg_t REG, typename T>
    struct send_reg_t
    {
        void send(Device* dev, T value)
        {
            static_assert(sizeof(T) <= 8, "Overflowed size");
            io::send(dev->_gw_id, dev->_port, dev->_can_id, CCTYPE_MAKE_USER_REG(CCTYPE_MAKE_WRITE_REG(REG)), sizeof(T), (const uint8_t*)&value);
        }
    };

public:
    Device()
    {
    }

    void dev_init(ECId_t gw_id, ECPort_t port, const id_t can_id)
    {
        _gw_id = gw_id;
        _port = port;
        _can_id = CCId_convertNum2Id((uint8_t)can_id);

        io::bind(_gw_id, _port, _can_id, std::bind(&Device::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }
};

}