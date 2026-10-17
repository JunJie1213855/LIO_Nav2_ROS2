#include <memory>
#include <rclcpp/rclcpp.hpp>

std::shared_ptr<rclcpp::Node> makeGroundSeparationNode(bool standalone);

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
#ifdef GROUND_SEG_CLOUD_ONLY
        auto node = makeGroundSeparationNode(true);
#else
        auto node = makeGroundSeparationNode(false);
#endif
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("ground_separation"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
