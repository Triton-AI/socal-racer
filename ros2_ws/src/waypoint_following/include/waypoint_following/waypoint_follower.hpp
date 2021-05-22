#ifndef WAYPOINT_FOLLOWER_HPP
#define WAYPOINT_FOLLOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <MiniPID.h>

namespace waypoint_following{
    using std_msgs::msg::Float64;
    using sensor_msgs::msg::NavSatFix;

    struct Waypoint
    {
        double latitude, longitude, altitude, rpm, heading;
    };
    typedef std::vector<Waypoint> Waypoints;

    class WaypointFollower: public rclcpp::Node
    {
    public:
        explicit WaypointFollower(const rclcpp::NodeOptions & options);
        ~WaypointFollower();
    private:
        class WaypointFileReader;
        class HeadingCalculator;
        class Localizer;

        rclcpp::Publisher<Float64>::SharedPtr speed_pub_;
        rclcpp::Publisher<Float64>::SharedPtr steering_pub_;
        rclcpp::Subscription<Float64>::SharedPtr speed_sub_;
        rclcpp::Subscription<NavSatFix>::SharedPtr gnss_sub_;

        void SpeedReceiveCallback(const Float64::SharedPtr speed);
        void GNSSReceiveCallback(const NavSatFix::SharedPtr nav);
        void calcAndSendCommands();

        std::unique_ptr<MiniPID> steering_controller_;
        std::unique_ptr<WaypointFileReader> file_reader_;
        std::unique_ptr<HeadingCalculator> heading_calc_;
        std::unique_ptr<Localizer> loc_;
        
        // Data
        std::shared_ptr<Waypoints> waypoints_;
        double real_speed;
        double speed_k;
        Waypoint current_location;
        double kp;
        double ki;
        double kd;
    };
}











#endif  // WAYPOINT_FOLLOWER_HPP