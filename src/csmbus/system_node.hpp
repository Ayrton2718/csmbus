#pragma once

#include "logger/logger.hpp"
#include "eth/eth_csmbus.h"

#include "robomas.hpp"
#include "odrive.hpp"
#include "can/cc_io.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <tuple>

#include <blackbox/blackbox.hpp>

namespace csmbus
{

enum class ether_app_t
{
    none,           // 何もない
    robomas,        // ロボマス
    can_csmbus,     // CAN CSMBus
    odrive,         // ODrive
    robomas_csmbus, // ロボマスとCAN CSMBusが共通のバスに接続されている
};

typedef std::vector<std::tuple<ECId_t, ether_app_t, ether_app_t>> ether_map_t;

// CSMBusのシステムノード
// 送受信のスレッドがこのノードで動いているため、このノードは必ず一つだけ動かす
class SystemNode : public rclcpp::Node, public blackbox::BlackBox, blackbox::LogRecorder
{
public:
    SystemNode(const std::string &name_space="") 
        : Node("csmbus_system", name_space), blackbox::BlackBox(this, blackbox::debug_mode_t::RELEASE), blackbox::LogRecorder(this)
    {
        logger::init(static_cast<rclcpp::Node*>(this), static_cast<blackbox::LogRecorder*>(this));

        ECCtrl_init();
    }

    // Gatewayの起動待機と、通信の初期化を行う
    // ether_mapは、GatewayのIDと、そのGatewayで動いているアプリケーションの種類を指定する
    /// @param ether_map イーサネットのマップ
    /// @param timeout_ms 起動待機のタイムアウト時間[ms]
    void link_up(const ether_map_t ether_map, const uint32_t timeout_ms=10000){
        this->setup_ether(ether_map, timeout_ms);
        ECCtrl_safetyOff();
    }

    ~SystemNode(void)
    {
        logger::destructor();
    }

private:
    void setup_ether(const ether_map_t ether_map, const uint32_t timeout_ms)
    {
        RealTimer tim;
        tim.start();

        std::set<ECId_t> id_list;
        std::set<app_addr_t> robomas_list;
        std::set<app_addr_t> odrive_list;
        std::set<app_addr_t> can_csmbus_list;

        for(auto it = ether_map.begin(); it != ether_map.end(); it++)
        {
            ECId_t id = std::get<0>(*it);
            ECPort_t port_list[2] = {ECPort_1, ECPort_2};
            ether_app_t app_list[2] = {std::get<1>(*it), std::get<2>(*it)};

            if(id != ECId_UNKNOWN)
            {
                id_list.insert(id);
                for(size_t i = 0; i < 2; i++)
                {
                    switch (app_list[i])
                    {

                    case ether_app_t::robomas:
                        robomas_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::can_csmbus:
                        can_csmbus_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::odrive :
                        odrive_list.insert(std::make_pair(id, port_list[i]));
                        break;

                    case ether_app_t::robomas_csmbus:
                        robomas_list.insert(std::make_pair(id, port_list[i]));
                        can_csmbus_list.insert(std::make_pair(id, port_list[i]));
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

            RealTimer cur_tim;
            cur_tim.start();
            bool is_linkup = false;
            while(rclcpp::ok() && (tim.getMs() < timeout_ms || cur_tim.getMs() < 500))
            {
                if(ECCtrl_isConnected(id)){
                    is_linkup = true;
                    break;
                }

                rclcpp::sleep_for(std::chrono::milliseconds(1));
            }

            ECCtrl_reset(id);
            ECCtrl_reset(id);
            ECCtrl_reset(id);
            
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
        can_csmbus::io::init(can_csmbus_list);
    }
};

}