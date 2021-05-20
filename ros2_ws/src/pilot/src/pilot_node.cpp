#include "pilot/pilot_node.hpp"

#include <sstream>
#include <string>
#include <algorithm>


namespace pilot
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  PilotNode::PilotNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pilot_node", options),
    mode_(DriveMode::HUMAN)
  {
    // Params
    motor_inverted = declare_parameter("motor_inverted").get<bool>();
    max_fwd_current = declare_parameter("max_fwd_current").get<uint32_t>();
    max_rev_current = declare_parameter("max_rev_current").get<uint32_t>();
    max_fwd_rpm = declare_parameter("max_fwd_rpm").get<uint32_t>();
    max_rev_rpm = declare_parameter("max_rev_rpm").get<uint32_t>();
    max_left_steering = declare_parameter("max_left_steering").get<double>();
    max_right_steering = declare_parameter("max_right_steering").get<double>();
    neutral_steering = declare_parameter("neutral_steering").get<double>();

    js_steering_axis = declare_parameter("js_steering_axis").get<uint32_t>();
    js_throttle_axis = declare_parameter("js_throttle_axis").get<uint32_t>();
    js_mode_button = declare_parameter("js_mode_button").get<uint32_t>();
    js_estop_button = declare_parameter("js_estop_button").get<uint32_t>();

    // Pubs N' Subs
    mode_ser_ = create_service<SetDriveMode>("set_drive_mode", std::bind(&PilotNode::changeDriveMode, this, _1, _2));

    current_pub_ = create_publisher<Float64>("commands/motor/current", rclcpp::QoS{5});
    speed_pub_ = create_publisher<Float64>("commands/motor/speed", rclcpp::QoS{5});
    steering_pub_ = create_publisher<Float64>("commands/servo/position", rclcpp::QoS{5});
    drive_mode_pub_ = create_publisher<String>("pilot/mode", rclcpp::QoS{5});

    throttle_sub_ = create_subscription<Float64>("commands/pilot/throttle", rclcpp::QoS{5}, 
                      std::bind(&PilotNode::commandThrottleCallback, this, _1));
    speed_sub_ = create_subscription<Float64>("commands/pilot/speed", rclcpp::QoS{5}, 
                      std::bind(&PilotNode::commandSpeedCallback, this, _1));
    steering_sub_ = create_subscription<Float64>("commands/pilot/steering", rclcpp::QoS{5}, 
                      std::bind(&PilotNode::commandSteeringCallback, this, _1));
    js_sub_ = create_subscription<Joy>("joy", rclcpp::QoS{5}, 
                      std::bind(&PilotNode::jsCallback, this, _1));
  }
  
  void PilotNode::changeDriveMode(const std::shared_ptr<SetDriveMode::Request> request, 
                            std::shared_ptr<SetDriveMode::Response> response)
  {
    auto md = str2mode.find(request->mode);
    if (md == str2mode.end()){
      response->success = false;
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Pilot: Unrecognized drive mode. Options are \"HUMAN\", \"AI_STEERING\" and \"AI\"");
    }
    else
    {
      std::stringstream ss;
      ss << "Pilot: Changed drive mode from " << mode2str.find(mode_)->second << " to " << request->mode;
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str());
      mode_ = (*md).second;
      response->success = true;
    }

  }

  void PilotNode::commandSpeedCallback(const Float64::SharedPtr speed)
  {
    if (mode_ == DriveMode::AI)
    {
      double spd = speed->data;
      auto spd_msg = Float64();
      spd_msg.data = clip( spd, (double) max_rev_rpm * -1.0, (double) max_fwd_rpm) * (motor_inverted ? -1.0 : 1.0);
      speed_pub_->publish(spd_msg);
    }
  }
  
  void PilotNode::commandSteeringCallback(const Float64::SharedPtr steering)
  {
    if (mode_ != DriveMode::HUMAN)
    {

      auto str_msg = Float64();
      str_msg.data = calcSteering(steering->data);
      steering_pub_->publish(str_msg);
    }
  }

  void PilotNode::commandThrottleCallback(const Float64::SharedPtr throttle)
  {
    if (mode_ == DriveMode::AI)
    {
      auto current_msg = Float64();
      current_msg.data = calcCurrent(throttle->data);
      current_pub_->publish(current_msg);
    }
  }

  void PilotNode::jsCallback(const Joy:: SharedPtr joy)
  {
    if (mode_ == DriveMode::HUMAN)
    {
      auto throttle = joy->axes[js_throttle_axis];
      auto current_msg = Float64();
      current_msg.data = calcCurrent((double) throttle);
      current_pub_->publish(current_msg);
    }

    if (mode_ != DriveMode::AI)
    {
      auto steering = joy->axes[js_steering_axis];
      auto steering_msg = Float64();
      steering_msg.data = calcSteering((double) steering);
      steering_pub_->publish(steering_msg);
    }


  }

  double PilotNode::calcCurrent(const double & raw_throttle)
  {
      double thr = clip(raw_throttle, -1.0, 1.0);
      double current = 0.0;
      if (thr < 0.0) // turn left
      {
        current = thr * max_rev_current;
      }
      else //turn right
      {
        current = thr * max_fwd_current;
      }

      return current;
  }
  double PilotNode::calcSteering(const double & raw_steering)
  {
      double str = clip(raw_steering, -1.0, 1.0);
      double real_str = neutral_steering;
      if (str < 0.0) // turn left
      {
        real_str = neutral_steering + (max_left_steering - neutral_steering) * std::abs(str);
      }
      else //turn right
      {
        real_str = neutral_steering + (max_right_steering - neutral_steering) * str;
      } 
      return real_str;
  }

  void PilotNode::cycleMode()
  {
    //Cycle through the mode options
    auto lastMode = mode_;
    mode_ = (*nextMode.find(mode_)).second;
    std::stringstream ss;
    ss << "Pilot: Changed drive mode from " << mode2str.find(lastMode)->second << " to " << mode2str.find(mode_)->second;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ss.str());
  }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pilot::PilotNode)
