#pragma once

#include <string>
#include <sys/time.h>
#include <functional>
#include "../eth_csmbus/ec_type.h"
#include "../csmbus_type.hpp"

#include <blackbox/bb_logger.hpp>

namespace csmbus::logger
{

struct diagnostic_t{
    bool status;
    std::string msg;
};

typedef std::function<diagnostic_t(void)> diag_cb_t;

void init(rclcpp::Node* node, blackbox::LogRecorder* lr);
void destructor(void);

void err_out(std::string tag, std::string str);
void err_out(std::string tag, const char* fmt, ...);

void info_out(std::string tag, std::string str);
void info_out(std::string tag, const char* fmt, ...);

void ether_diag_bind(ECId_t gw_id, ECPort_t port, std::string name, diag_cb_t function);
void can_diag_bind(ECId_t gw_id, ECPort_t port, csmbus::id_t id, std::string name, diag_cb_t function);

}