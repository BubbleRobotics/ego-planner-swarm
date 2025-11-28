#include <rclcpp/rclcpp.hpp>
#include "drone_detect/drone_detector.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    /* Create a shared pointer object for the corresponding node */
    auto node = std::make_shared<detect::DroneDetector>("drone_detect");
    node->test();

    /* Run the node and monitor for exit signals */
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
