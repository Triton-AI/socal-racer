#ifndef PILOT_NODE_HPP
#define PILOT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <pilot_services/srv/set_drive_mode.hpp>

#include <map>
#include <string>

namespace pilot{
    using std_msgs::msg::Float64;
    using sensor_msgs::msg::Joy;
    using std_msgs::msg::String;
    using pilot_services::srv::SetDriveMode;

    class PilotNode: public rclcpp::Node
    {
    public:
        explicit PilotNode(const rclcpp::NodeOptions & options);

    private:
    // Params
        bool motor_inverted;
        uint32_t max_fwd_current;
        uint32_t max_rev_current;
        uint32_t max_fwd_rpm;
        uint32_t max_rev_rpm;
        double max_left_steering;
        double max_right_steering;
        double neutral_steering;

        uint32_t js_steering_axis;
        uint32_t js_throttle_axis;
        uint32_t js_mode_button;
        uint32_t js_estop_button;


    // Pub N' Sub
        rclcpp::Publisher<Float64>::SharedPtr current_pub_;
        rclcpp::Publisher<Float64>::SharedPtr speed_pub_;
        rclcpp::Publisher<Float64>::SharedPtr steering_pub_;
        rclcpp::Publisher<String>::SharedPtr drive_mode_pub_;
        rclcpp::Subscription<Float64>::SharedPtr throttle_sub_; // [-1, 1]
        rclcpp::Subscription<Float64>::SharedPtr speed_sub_;
        rclcpp::Subscription<Float64>::SharedPtr steering_sub_; // [-1, 1]
        rclcpp::Subscription<Joy>::SharedPtr js_sub_;

        void commandSpeedCallback(const Float64::SharedPtr speed);
        void commandSteeringCallback(const Float64::SharedPtr steering);
        void commandThrottleCallback(const Float64::SharedPtr throttle);
        void jsCallback(const Joy:: SharedPtr joy);

    // Drive Mode
        enum DriveMode
        {
            HUMAN = 0,
            AI_STEERING = 1,
            AI = 2
        };
        DriveMode mode_;
        const std::map<std::string, DriveMode> str2mode =
        {{"HUMAN", DriveMode::HUMAN}, {"AI_STEERING", DriveMode::AI_STEERING}, {"AI", DriveMode::AI}};
        const std::map<DriveMode, std::string> mode2str =
        {{DriveMode::HUMAN, "HUMAN"}, {DriveMode::AI_STEERING, "AI_STEERING"}, {DriveMode::AI, "AI"}};
        const std::map<DriveMode, DriveMode> nextMode =
        {{DriveMode::HUMAN, DriveMode::AI_STEERING}, {DriveMode::AI_STEERING, DriveMode::AI}, {DriveMode::AI, DriveMode::HUMAN}};
    // Service: change drive mode
        rclcpp::Service<SetDriveMode>::SharedPtr mode_ser_;
        void changeDriveMode(const std::shared_ptr<SetDriveMode::Request> request, 
                            std::shared_ptr<SetDriveMode::Response> response);

    // Utils
        template <typename T>
        T clip(const T& n, const T& lower, const T& upper) 
        { return std::max(lower, std::min(n, upper)); }

        double calcCurrent(const double & raw_throttle);
        double calcSteering(const double & raw_steering);
        
        void cycleMode();
    };
}








#endif  // PILOT_NODE_HPP
