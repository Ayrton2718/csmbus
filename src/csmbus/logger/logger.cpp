#include "logger.hpp"

#include <blackbox/bb_logger.hpp>

namespace smbus::logger
{

static std::mutex           g_locker;
volatile static bool        g_is_initialized = false;


static std::unique_ptr<blackbox::Logger>    g_info = nullptr;
static std::unique_ptr<blackbox::Logger>    g_error = nullptr;
static std::unique_ptr<blackbox::Logger>    g_diag_error = nullptr;
static blackbox::DiagnosticUpdater*         g_diag_updater = nullptr;

static std::mutex                                                                       g_can_locker;
static std::vector<std::pair<std::string, diag_cb_t>>   g_diag_list;

static void diagnostic_bind(std::string tag, diag_cb_t function);
static diagnostic_t can_diagnostics_task(void);

void init(blackbox::LogRecorder* lr, blackbox::DiagnosticUpdater* du)
{
    {
        std::lock_guard<std::mutex> lock(g_locker);
        if(g_is_initialized == false)
        {
            g_info = std::make_unique<blackbox::Logger>();
            g_info->init(lr, blackbox::INFO, "smbus");

            g_error = std::make_unique<blackbox::Logger>();
            g_error->init(lr, blackbox::ERR, "smbus");

            g_diag_error = std::make_unique<blackbox::Logger>();
            g_diag_error->init(lr, blackbox::ERR, "diag_err");
            
            g_diag_updater = du;

            g_is_initialized = true;
        }
    }

    diagnostic_bind("can_smbus", can_diagnostics_task);
}

void destructor(void)
{
    std::lock_guard<std::mutex> lock(g_locker);
    if(g_is_initialized)
    {
        g_info.release();
        g_error.release();
        g_diag_error.release();
        g_diag_updater = nullptr;

        g_is_initialized = false;
    }
}



void err_out(std::string tag, std::string str)
{
    std::lock_guard<std::mutex> lock(g_locker);
    if(g_error)
    {
        TAGGER(g_error.get(), "<" + tag + "> " + str);
    }
}

void err_out(std::string tag, const char* fmt, ...)
{
    std::string str;
    va_list ap;
    va_list ap_copy;
    va_copy(ap_copy, ap);

    va_start(ap, fmt);
    str.resize(1024);
    int len = vsnprintf(&str[0], 1024, fmt, ap);
    va_end(ap);

    if(len < 1024)
    {
        str.resize(len + 1);
    }else{
        str.resize(len + 1);  // need space for NUL

        va_start(ap_copy, fmt);
        vsnprintf(&str[0], len + 1, fmt, ap_copy);
        va_end(ap_copy);
    }

    err_out(tag, str);
}


void info_out(std::string tag, std::string str)
{
    std::lock_guard<std::mutex> lock(g_locker);
    if(g_info)
    {
        TAGGER(g_info.get(), "<" + tag + "> " + str);
    }
}

void info_out(std::string tag, const char* fmt, ...)
{
    std::string str;
    va_list ap;
    va_list ap_copy;
    va_copy(ap_copy, ap);

    va_start(ap, fmt);
    str.resize(1024);
    int len = vsnprintf(&str[0], 1024, fmt, ap);
    va_end(ap);

    if(len < 1024)
    {
        str.resize(len + 1);
    }else{
        str.resize(len + 1);  // need space for NUL

        va_start(ap_copy, fmt);
        vsnprintf(&str[0], len + 1, fmt, ap_copy);
        va_end(ap_copy);
    }

    info_out(tag, str);
}

void ether_diag_bind(ESId_t gw_id, ESPort_t port, std::string name, diag_cb_t function)
{
    std::string key = "\"eth_" + name + "(" + std::to_string(gw_id + 1) +"-"+ std::to_string(port + 1) +")\"";
    {        
        std::lock_guard<std::mutex> lock(g_can_locker);
        g_diag_list.push_back(std::make_pair(key, function));
    }
}

void can_diag_bind(ESId_t gw_id, ESPort_t port, smbus::id_t id, std::string name, diag_cb_t function)
{
    std::string key = "\"" + name + "(" + std::to_string(gw_id + 1) +"-"+ std::to_string(port + 1) +"-"+ std::to_string((int)id + 1) +")\"";
    {        
        std::lock_guard<std::mutex> lock(g_can_locker);
        g_diag_list.push_back(std::make_pair(key, function));
    }
}


static void diagnostic_bind(std::string tag, diag_cb_t function)
{
    diagnostic_updater::TaskFunction task = 
        [function](diagnostic_updater::DiagnosticStatusWrapper & stat) {
            diagnostic_t diag = function();
            uint8_t lvl;
            if(diag.status)
            {
                lvl = diagnostic_msgs::msg::DiagnosticStatus::OK;
            }else{
                lvl = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            }
            stat.summary(lvl, diag.msg);
        };

    g_locker.lock();
    if(g_diag_updater != nullptr)
    {
        g_diag_updater->diagnostic_bind(tag, task);
    }
    g_locker.unlock();
}

static diagnostic_t can_diagnostics_task(void)
{
    std::lock_guard<std::mutex> lock(g_can_locker);

    bool is_first = true;
    diagnostic_t status;
    status.status = true;
    status.msg = "\"count(" + std::to_string(g_diag_list.size()) + ")\":{";
    for(auto& it : g_diag_list)
    {
        diagnostic_t diag = it.second();
        if(diag.status == false)
        {
            if(!is_first){
                status.msg += ", ";
            } 
            status.msg += it.first + ":\"" + diag.msg + "\"";
            status.status = false;
            is_first = false;
        }
    }
    status.msg += '}';

    if(g_is_initialized && status.status == false){
        TAGGER(g_diag_error.get(), status.msg);
    }
    return status;
}

}