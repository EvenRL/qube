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
        auto kp_desc = rcl_interfaces::msg::ParameterDescriptor{};
        kp_desc.description = "Proportional gain.";
        kp_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
            .set__from_value(0.0)
            .set__to_value(1000.0)};
        
        auto ki_desc = rcl_interfaces::msg::ParameterDescriptor{};
        ki_desc.description = "Integral gain.";
        ki_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
            .set__from_value(0.0)
            .set__to_value(1000.0)};

        auto kd_desc = rcl_interfaces::msg::ParameterDescriptor{};
        kd_desc.description = "Derivative gain.";
        kd_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
            .set__from_value(0.0)
            .set__to_value(1000.0)};
        
        auto setpoint_desc = rcl_interfaces::msg::ParameterDescriptor{};
        setpoint_desc.description = "Position/angle setpoint for qube, value in radians.";

        auto jointname_desc = rcl_interfaces::msg::ParameterDescriptor{};
        jointname_desc.description = "Name of joint to control";

        auto maxv_desc = rcl_interfaces::msg::ParameterDescriptor{};
        maxv_desc.description = "Max allowed velocity of qube";
        maxv_desc.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
            .set__from_value(0.0)
            .set__to_value(1000.0)};

        this->declare_parameter<double>("Kp", 1, kp_desc);
        this->declare_parameter<double>("Ki", 0.0, ki_desc);
        this->declare_parameter<double>("Kd", 0.0, kd_desc);
        this->declare_parameter<double>("setpoint", 0.0, setpoint_desc);
        this->declare_parameter<std::string>("joint_name", "motor_joint", jointname_desc);
        this->declare_parameter<double>("max_velocity", 10.0, maxv_desc);

        parameterCallback = this->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
            return this->parameter_callback(parameters);});

        // Get parameters
        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        setpoint_ = this->get_parameter("setpoint").as_double();
        joint_name_ = this->get_parameter("joint_name").as_string();
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

    rcl_interfaces::msg::SetParametersResult parameter_callback(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
            
        for (auto param : parameters) {
            if (param.get_name() == "Kp") {
                this->Kp_ = param.as_double(); 
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "Ki") {
                this->Ki_ = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "Kd") {
                this->Kd_ = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "setpoint") {
                this->setpoint_ = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "joint_name") {
                this->joint_name_ = param.as_string();
                result.successful = true;
                result.reason = "success";
            }
            else if (param.get_name() == "max_velocity") {
                this->max_velocity_ = param.as_double();
                result.successful = true;
                result.reason = "success";
            }
            else {
                result.successful = false;
                result.reason = "unknown parameter";
            }
        }
        return result;
    }

    // Parameters
    double Kp_;
    double Ki_;
    double Kd_;
    double setpoint_;
    std::string joint_name_;
    double max_velocity_;

    OnSetParametersCallbackHandle::SharedPtr parameterCallback;

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