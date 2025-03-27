#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <string>
#include <vector>

class QubeController : public rclcpp::Node {
public:
    QubeController() : Node("qube_controller") {
        // Declare parameters
        this->declare_parameter<double>("Kp", 0.1);
        this->declare_parameter<double>("Ki", 0.0);
        this->declare_parameter<double>("Kd", 0.0);
        this->declare_parameter<double>("setpoint", 0.0);
        this->declare_parameter<std::string>("angle", "rotor_joint");
        this->declare_parameter<double>("max_velocity", 10.0);

        // Get parameters
        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        setpoint_ = this->get_parameter("setpoint").as_double();
        joint_name_ = this->get_parameter("angle").as_string();
        max_velocity_ = this->get_parameter("max_velocity").as_double();

        // Initialize PID variables
        integral_ = 0.0;
        previous_error_ = 0.0;
        last_time_ = this->now();

        // Create subscriber and publisher
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&QubeController::joint_state_callback, this, std::placeholders::_1));

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/velocity_controller/commands", 10);
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Find the index of the joint
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_name_);
        if (it == msg->name.end()) {
            RCLCPP_ERROR(this->get_logger(), "Joint '%s' not found in joint_states", joint_name_.c_str());
            return;
        }
        int index = std::distance(msg->name.begin(), it);

        double current_position = msg->position[index];
        rclcpp::Time current_time = msg->header.stamp;

        // Calculate time difference
        double dt = (current_time - last_time_).seconds();

        if (dt <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Non-positive dt detected: %f. Skipping update.", dt);
            last_time_ = current_time;
            return;
        }

        // PID calculations
        double error = setpoint_ - current_position;
        integral_ += error * dt;
        double derivative = (error - previous_error_) / dt;

        double output = (Kp_ * error) + (Ki_ * integral_) + (Kd_ * derivative);

        // Clamp output to max velocity
        output = std::clamp(output, -max_velocity_, max_velocity_);

        // Update previous error and time
        previous_error_ = error;
        last_time_ = current_time;

        // Publish velocity command
        std_msgs::msg::Float64MultiArray velocity_msg;
        velocity_msg.data.push_back(output);
        velocity_pub_->publish(velocity_msg);
    }

    // Parameters
    double Kp_;
    double Ki_;
    double Kd_;
    double setpoint_;
    std::string joint_name_;
    double max_velocity_;

    // PID variables
    double integral_;
    double previous_error_;
    rclcpp::Time last_time_;

    // ROS2 Subscriber and Publisher
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QubeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}