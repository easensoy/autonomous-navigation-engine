// cmd_vel_gate: the stop mechanism between the navigator and the robot base.
//
// Forwards cmd_vel_nav to cmd_vel at a fixed rate on the steady (wall) clock and sends zero instead when
//   - the emergency stop is set (ane/estop, std_msgs/Bool, latched; true holds until false is sent), or
//   - the newest input is older than input_timeout_s (navigator stopped, crashed or stalled).
// Forwarded commands are clamped to max_linear_mps / max_angular_radps. The gate uses neither
// simulation time nor any simulator service, so it stops a base whether or not Gazebo is running, and
// it keeps working when simulation time stops advancing. The output state (FORWARD, STALE, ESTOP) is
// published on ane/gate_state whenever it changes.
//
// Gazebo's DiffDrive keeps executing the last command it received, so the gate must publish zero
// actively; silence would not stop the simulated robot.
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

class CmdVelGate : public rclcpp::Node {
public:
    CmdVelGate() : Node("cmd_vel_gate") {
        const double rate = declare_parameter<double>("rate_hz", 20.0);
        input_timeout_ = declare_parameter<double>("input_timeout_s", 0.3);
        max_linear_ = declare_parameter<double>("max_linear_mps", 0.6);
        max_angular_ = declare_parameter<double>("max_angular_radps", 1.5);

        output_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>("ane/gate_state", rclcpp::QoS(1).reliable().transient_local());
        input_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel_nav", 10, [this](const geometry_msgs::msg::Twist& msg) {
            last_ = msg;
            last_time_ = std::chrono::steady_clock::now();
            have_input_ = true;
        });
        estop_ = create_subscription<std_msgs::msg::Bool>("ane/estop", rclcpp::QoS(1).reliable().transient_local(),
                                                          [this](const std_msgs::msg::Bool& msg) {
                                                              if (msg.data != estop_set_) {
                                                                  RCLCPP_WARN(get_logger(), "emergency stop %s", msg.data ? "SET" : "released");
                                                              }
                                                              estop_set_ = msg.data;
                                                              tick();
                                                          });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { tick(); });
    }

private:
    void tick() {
        geometry_msgs::msg::Twist out;
        std::string state = "FORWARD";
        const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_time_).count();
        if (estop_set_) {
            state = "ESTOP";
        } else if (!have_input_ || age > input_timeout_) {
            state = "STALE";
        } else {
            out.linear.x = std::clamp(last_.linear.x, -max_linear_, max_linear_);
            out.angular.z = std::clamp(last_.angular.z, -max_angular_, max_angular_);
        }
        output_->publish(out);
        if (state != state_) {
            state_ = state;
            std_msgs::msg::String msg;
            msg.data = state;
            state_pub_->publish(msg);
        }
    }

    double input_timeout_ = 0.3;
    double max_linear_ = 0.6;
    double max_angular_ = 1.5;
    bool have_input_ = false;
    bool estop_set_ = false;
    std::string state_;
    geometry_msgs::msg::Twist last_;
    std::chrono::steady_clock::time_point last_time_{};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr input_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelGate>());
    rclcpp::shutdown();
    return 0;
}
