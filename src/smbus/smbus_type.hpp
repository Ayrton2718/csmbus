#pragma once

#include "eth_smbus/es_type.h"

namespace smbus
{

enum struct port_t
{
    _1 = 0,
    _2 = 1,
    _3 = 2,
    _4 = 3
};

enum struct id_t
{
    _1 = 0,
    _2 = 1,
    _3 = 2,
    _4 = 3,
    _5 = 4,
    _6 = 5,
    _7 = 6,
    _8 = 7,
    _9 = 8,
    _10 = 9,
    _11 = 10,
    _12 = 11,
};

enum struct direction_t
{
    forward,
    inverse
};

typedef std::pair<ESId_t, ESPort_t> app_addr_t;

}