#include "norm_calc/norm_calc.h"
#include "norm_calc/chisel_box.h"
#include "norm_calc/edge_grid.h"

#include <iostream>
#include <memory>

using namespace std;

int main(int argc, char * argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create a node for norm_viewer
    auto node = rclcpp::Node::make_shared("norm_viewer");

    // Placeholder for norm_viewer functionality
    // This needs to be implemented based on the original ROS1 norm_viewer logic
    RCLCPP_INFO(node->get_logger(), "Norm Viewer Node Started");

    // Example placeholder for a subscription or other ROS2 functionality
    // would go here, but the original norm_viewer.cpp was primarily a
    // visualization tool that relied on ROS1-specific constructs.
    // A full ROS2 implementation would require re-architecting this part
    // using ROS2 visualization tools like RViz2 plugins or other methods.

    // For now, we just spin to keep the node alive for demonstration
    // In a real implementation, you would have publishers/subscribers
    // or other logic here.
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}