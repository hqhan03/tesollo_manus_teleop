#include "teleop_slave/tesollo_slave_node.hpp"

TesolloSlaveNode::TesolloSlaveNode() : Node("tesollo_slave_node") {
    this->declare_parameter("ip", "169.254.186.72");
    this->declare_parameter("port", 502);
    
    ip_ = this->get_parameter("ip").as_string();
    port_ = this->get_parameter("port").as_int();

    finger_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/manus/finger_joints", 10,
        std::bind(&TesolloSlaveNode::fingerJointsCallback, this, std::placeholders::_1));

    try {
        RCLCPP_INFO(this->get_logger(), "Connecting to Tesollo DG-5F at %s:%d", ip_.c_str(), port_);
        delto_client_ = std::make_unique<DG5F_TCP>(ip_, port_);
        delto_client_->connect();
        delto_client_->start_control();
        connected_ = true;
        RCLCPP_INFO(this->get_logger(), "Tesollo Slave Node Started and Connected");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to DG-5F: %s", e.what());
    }
}

void TesolloSlaveNode::fingerJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!connected_ || !delto_client_ || msg->position.size() < 20) {
        return;
    }

    std::vector<double> delto_target(20, 0.0);

    // Motors 1-16 (Thumb, Index, Middle, Ring) direct mapping
    for (int i = 0; i < 16; ++i) {
        delto_target[i] = msg->position[i];
    }

    // Pinky Finger Custom Kinematics
    // Motor 17 (idx 16) is a fixed DOF not present in the human hand
    delto_target[16] = 0.0; 
    
    // Shift Pinky MCP Roll and Pitch
    delto_target[17] = msg->position[16];   // Pinky MCP Spread
    delto_target[18] = msg->position[17];   // Pinky MCP Stretch
    
    // Combine PIP and DIP into the distal Motor 20
    delto_target[19] = msg->position[18] + msg->position[19];

    // Send the positional targets down to the RS485 chain via ModbusTCP
    delto_client_->set_position_rad(delto_target);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TesolloSlaveNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
