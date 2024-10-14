#pragma once

#include "logger/logger.hpp"
#include "tut_tool/tt_timer.hpp"

#include "eth_smbus/eth_smbus.h"

#include "robomas.hpp"
#include "odrive.hpp"
#include "can_smbus/cs_io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tuple>

#include <blackbox/blackbox.hpp>

namespace smbus
{

enum class ether_app_t
{
    none,
    robomas,
    can_smbus,
    odrive,
    robomas_smbus,
};

typedef std::vector<std::tuple<ESId_t, ether_app_t, ether_app_t>> ether_map_t;

class SystemNode : public rclcpp::Node, public blackbox::BlackBox, blackbox::DiagnosticUpdater, blackbox::LogRecorder
{
public:
    SystemNode(const std::string &name_space="") 
        : Node("smbus_system", name_space), blackbox::BlackBox(this, blackbox::debug_mode_t::RELEASE), blackbox::DiagnosticUpdater(this, "smbus_io", 0.1), blackbox::LogRecorder(this)
    {
        logger::init(this, this);

        ESCtrl_init();
    }

    void link_up(const ether_map_t ether_map, const uint32_t timeout_ms=10000){
        this->setup_ether(ether_map, timeout_ms);

        ESCtrl_safetyOff();

    }

    ~SystemNode(void)
    {
        logger::destructor();
    }

private:
    void setup_ether(const ether_map_t ether_map, const uint32_t timeout_ms)
    {
        tut_tool::RealTimer tim;
        tim.start();

        std::set<ESId_t> id_list;
        std::set<app_addr_t> robomas_list;
        std::set<app_addr_t> odrive_list;
        std::set<app_addr_t> can_smbus_list;

        for(auto it = ether_map.begin(); it != ether_map.end(); it++)
        {
            ESId_t id = std::get<0>(*it);
            ESPort_t port_list[2] = {ESPort_1, ESPort_2};
            ether_app_t app_list[2] = {std::get<1>(*it), std::get<2>(*it)};

            if(id != ESId_UNKNOWN)
            {
                id_list.insert(id);
                for(size_t i = 0; i < 2; i++)
                {
                    switch (app_list[i])
                    {

                    case ether_app_t::robomas:
                        robomas_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::can_smbus:
                        can_smbus_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::odrive :
                        odrive_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::robomas_smbus:
                        robomas_list.insert(std::make_pair(id, port_list[i]));
                        can_smbus_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::none:
                        break;

                    default:
                        logger::err_out("system", "Unkown appid! ID(%d), appid(%d)", id, app_list[i]);
                        break;
                    }
                }
            }else{
                logger::err_out("system", "Unknown id!");
            }
        }

        for (const auto& id : id_list) {
            logger::info_out("system", "Waiting boot(%d) ...", id);
            RCLCPP_INFO(this->get_logger(),  "Waiting boot(%d) ...", id);

            tut_tool::RealTimer cur_tim;
            cur_tim.start();
            bool is_linkup = false;
            while(rclcpp::ok() && (tim.getMs() < timeout_ms || cur_tim.getMs() < 500))
            {
                if(ESCtrl_isConnected(id)){
                    is_linkup = true;
                    break;
                }

                rclcpp::sleep_for(std::chrono::milliseconds(1));
            }

            ESCtrl_reset(id);
            ESCtrl_reset(id);
            ESCtrl_reset(id);
            
            if(is_linkup)
            {
                logger::info_out("system", "Connected(%d)!", id);
                RCLCPP_INFO(this->get_logger(),  "Connected(%d)!", id);
            }else{
                logger::err_out("system", "Can't Connected(%d)!", id);
                RCLCPP_ERROR(this->get_logger(),  "Can't Connected(%d)!", id);
            }
        }

        robomas::init(robomas_list);
        odrive::init(odrive_list);
        can_smbus::io::init(can_smbus_list);
    }
};

}