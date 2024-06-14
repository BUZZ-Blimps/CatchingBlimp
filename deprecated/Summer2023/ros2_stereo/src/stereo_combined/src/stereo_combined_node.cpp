#include <rclcpp/rclcpp.hpp>

#include "StereoCombined.hpp"

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);

	RCLCPP_INFO(rclcpp::get_logger("stereo_combined_node"), "Stereo combined node running");

	std::shared_ptr<rclcpp::Node> stereo_combined_node = std::make_shared<StereoCombined>();
	rclcpp::spin(stereo_combined_node);
	rclcpp::shutdown();
	return 0;
}
