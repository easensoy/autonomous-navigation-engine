// fault_injector: test instrument for scenarios D, E and F. Transparent unless a fault is active.
//
//   ground_truth/odom -> odom          relayed unchanged, except during odom_dropout (not relayed)
//   cmd_vel           -> cmd_vel_robot relayed unchanged, except during drive_stall (zero relayed instead)
//
// ane/inject_fault (ane_interfaces/InjectFault) starts a fault for duration_s of simulation time. The
// start and end of each fault are published on ane/fault_events with source SCRIPTED.
#include <ane_interfaces/msg/navigator_event.hpp>
#include <ane_interfaces/srv/inject_fault.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

using ane_interfaces::msg::NavigatorEvent;
using ane_interfaces::srv::InjectFault;

class FaultInjector : public rclcpp::Node {
public:
    FaultInjector() : Node("fault_injector") {
        odom_out_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(50));
        cmd_out_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_robot", 10);
        events_ = create_publisher<NavigatorEvent>("ane/fault_events", rclcpp::QoS(50).reliable());
        odom_in_ = create_subscription<nav_msgs::msg::Odometry>("ground_truth/odom", rclcpp::QoS(50),
                                                                [this](const nav_msgs::msg::Odometry& msg) {
                                                                    if (!active("odom_dropout")) {
                                                                        odom_out_->publish(msg);
                                                                    }
                                                                });
        cmd_in_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, [this](const geometry_msgs::msg::Twist& msg) {
            cmd_out_->publish(active("drive_stall") ? geometry_msgs::msg::Twist{} : msg);
        });
        service_ = create_service<InjectFault>(
            "ane/inject_fault", [this](const std::shared_ptr<InjectFault::Request> request, std::shared_ptr<InjectFault::Response> response) {
                if ((request->fault != "odom_dropout" && request->fault != "drive_stall") || !(request->duration_s > 0.0)) {
                    response->accepted = false;
                    response->message = "fault must be odom_dropout or drive_stall with a positive duration";
                    return;
                }
                if (!fault_.empty()) {
                    response->accepted = false;
                    response->message = "a fault is already active";
                    return;
                }
                fault_ = request->fault;
                until_ = now().seconds() + request->duration_s;
                publish("FAULT_STARTED", request->duration_s);
                response->accepted = true;
                response->message = fault_ + " for " + std::to_string(request->duration_s) + " s";
            });
    }

private:
    bool active(const std::string& fault) {
        if (!fault_.empty() && now().seconds() >= until_) {
            publish("FAULT_ENDED", 0.0);
            fault_.clear();
        }
        return fault_ == fault;
    }

    void publish(const std::string& type, double value) {
        NavigatorEvent e;
        e.stamp = now();
        e.sequence = ++sequence_;
        e.wall_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        e.type = type;
        e.source = "SCRIPTED";
        e.value = value;
        e.detail = fault_;
        events_->publish(e);
        RCLCPP_WARN(get_logger(), "%s %s", type.c_str(), fault_.c_str());
    }

    std::string fault_;
    double until_ = 0.0;
    uint64_t sequence_ = 0;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_out_;
    rclcpp::Publisher<NavigatorEvent>::SharedPtr events_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_in_;
    rclcpp::Service<InjectFault>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaultInjector>());
    rclcpp::shutdown();
    return 0;
}
