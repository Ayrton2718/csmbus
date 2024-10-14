#pragma once

#include <string>
#include <sys/time.h>
#include <functional>
#include "../eth_smbus/es_type.h"
#include "../smbus_type.hpp"

#include <blackbox/bb_logger.hpp>
#include <blackbox/bb_diagnostics.hpp>

namespace smbus::logger
{

struct diagnostic_t{
    bool status;
    std::string msg;
};

typedef std::function<diagnostic_t(void)> diag_cb_t;

void init(blackbox::LogRecorder* lr, blackbox::DiagnosticUpdater* du);
void destructor(void);

void err_out(std::string tag, std::string str);
void err_out(std::string tag, const char* fmt, ...);

void info_out(std::string tag, std::string str);
void info_out(std::string tag, const char* fmt, ...);

void ether_diag_bind(ESId_t gw_id, ESPort_t port, std::string name, diag_cb_t function);
void can_diag_bind(ESId_t gw_id, ESPort_t port, smbus::id_t id, std::string name, diag_cb_t function);

}