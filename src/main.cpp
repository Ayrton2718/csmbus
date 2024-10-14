#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "csmbus/eth_csmbus/eth_csmbus.h"
#include "csmbus/csmbus.hpp"

using namespace csmbus;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::executors::MultiThreadedExecutor executor;

    ether_map_t ether_map = {
        {ECId_1, ether_app_t::robomas_smbus, ether_app_t::robomas_smbus},
        {ECId_2, ether_app_t::robomas_smbus, ether_app_t::odrive},
        {ECId_3, ether_app_t::robomas_smbus, ether_app_t::robomas_smbus},
        {ECId_4, ether_app_t::robomas_smbus, ether_app_t::odrive}
    };

    std::shared_ptr<SystemNode> system_node = std::make_shared<SystemNode>();
    std::vector<std::shared_ptr<rclcpp::Node>> node;
    node.push_back(system_node);

    system_node->link_up(ether_map, 500);

    for(auto it = node.begin(); it != node.end(); it++)
    {
        executor.add_node(*it);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}