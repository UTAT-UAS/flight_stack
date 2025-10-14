#include "flight_stack/flight_core.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    bool is_simulation = false;
    for (int i = 0; i < argc; i++)
    {
        if (std::string(argv[i]) == "--simulation")
        {
            is_simulation = true;
        }
    }

    std::cout << "Starting FlightCore node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node1 = std::make_shared<FlightCore>(is_simulation);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node1);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}