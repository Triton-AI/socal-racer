#include <waypoint_following/waypoint_follower.hpp>
#include <boost/geometry/geometry.hpp>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
using namespace boost::geometry;
namespace waypoint_following
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  class WaypointFollower::WaypointFileReader
  {
  public:
    std::shared_ptr<Waypoints> waypoints;

    WaypointFileReader(const std::string & waypoint_file_path)
    {
      waypoints = readWaypointsFromFile(waypoint_file_path);
    }

    std::shared_ptr<Waypoints> readWaypointsFromFile(const std::string & waypoint_file_path)
    {
      std::shared_ptr<Waypoints> pts(new Waypoints());
      std::ifstream f(waypoint_file_path.c_str());
      std::string line;
      std::getline(f, line); // Ignore first line
      while (std::getline(f, line))
      {
        auto tokens = tokenize(line, ",");
        if (tokens->size() == 5)
        {
          double lat = std::stod((*tokens)[0]);
          double lon = std::stod((*tokens)[1]);
          double alt = std::stod((*tokens)[2]);
          double rpm = std::stod((*tokens)[3]);
          double heading = std::stod((*tokens)[4]);
          pts->push_back(Waypoint{lat, lon, alt, rpm, heading});
        }
      }
      f.close();
      return pts;
    }

    std::shared_ptr<std::vector<std::string>> tokenize(const std::string & s, const std::string & del = " ")
    {
      std::shared_ptr<std::vector<std::string>> tokens(new std::vector<std::string>());
      int start = 0;
      int end = s.find(del);
      while (end != -1) {
          start = end + del.size();
          end = s.find(del, start);
          tokens->push_back(s.substr(start, end - start));
      }
      return tokens;
    }
  };

  class WaypointFollower::HeadingCalculator
  {
    Waypoint calc(const NavSatFix::SharedPtr nav)
    {
      return Waypoint{0.0, 0.0, 0.0, 0.0, 0.0};
    }
  };

  class WaypointFollower::Localizer
  {
  public:
    Localizer(std::shared_ptr<Waypoints> waypoints)
    {
      this->waypoints = waypoints;
      std::vector<model::d2::point_xy<double>> points = {};
      for (const auto & wp : *waypoints)
      {
        points.push_back(model::d2::point_xy<double>(wp.latitude, wp.longitude));
      }
      append(path_poly, points);
    }
    model::polygon<model::d2::point_xy<double>> path_poly;
    std::shared_ptr<Waypoints> waypoints;
    int cloest_idx = 0;
    double getCrossTrackError(const Waypoint & wp)
    {
      auto pt = model::d2::point_xy<double>(wp.latitude, wp.longitude);
      double cte = distance(pt, path_poly) * (within(pt, path_poly) ? 1.0 : -1.0);
      return cte;
    }

    double getHeadingError(const Waypoint & location)
    {
      // find the points
      std::vector<double> distances;
      distances.reserve(waypoints->size());
      for (const auto & wp: *waypoints)
      {
        distances.push_back(sqrt(pow(wp.latitude - location.latitude, 2) + pow(wp.longitude - location.longitude, 2)));
      }
      cloest_idx = std::min_element(distances.begin(), distances.end()) - distances.begin();

      // Calculate heading differences. left is negative and right is positive
      double target_heading = (*waypoints)[cloest_idx].heading;
      double real_heading = location.heading;
      double tl = 0.0;
      double tr = 0.0;
      if (target_heading > real_heading){
        tl = target_heading - real_heading;
        tr = real_heading + 2 * M_PI - target_heading;
      }
      else
      {
        tl = 2 * M_PI - real_heading + target_heading;
        tr = real_heading - target_heading;
      }
      return tl < tr ? -tl : tr;
    }

    double getTargetSpeed()
    {
      return (*waypoints)[cloest_idx].rpm;
    }
  };

  WaypointFollower::WaypointFollower(const rclcpp::NodeOptions & options)
  : rclcpp::Node("waypoint_follower_node", options)
  {
    speed_pub_ = create_publisher<Float64>("commands/motor/speed", rclcpp::QoS{5});
    steering_pub_ = create_publisher<Float64>("commands/servo/position", rclcpp::QoS{5});

    speed_sub_ = create_subscription<Float64>("commands/pilot/speed", rclcpp::QoS{5}, 
                      std::bind(&WaypointFollower::SpeedReceiveCallback, this, _1));
    gnss_sub_ = create_subscription<NavSatFix>("", rclcpp::QoS{5}, 
                      std::bind(&WaypointFollower::GNSSReceiveCallback, this, _1));
    //speed_k = declare_parameter("speed_k").get<double>();
    kp = declare_parameter("KP").get<double>();
    ki = declare_parameter("KI").get<double>();
    kd = declare_parameter("KD").get<double>();
    std::string waypoint_file = declare_parameter("waypoint_file").get<std::string>();
    steering_controller_ = std::unique_ptr<MiniPID>(new MiniPID(kp, ki, kd));
    file_reader_= std::unique_ptr<WaypointFileReader>(new WaypointFileReader(waypoint_file));
    heading_calc_ =  std::unique_ptr<HeadingCalculator>(new HeadingCalculator());
    loc_ = std::unique_ptr<Localizer>(new Localizer(file_reader_->waypoints));
  }


  void WaypointFollower::SpeedReceiveCallback(const Float64::SharedPtr speed)
  {
    real_speed = speed->data;
  }
  void WaypointFollower::GNSSReceiveCallback(const NavSatFix::SharedPtr nav)
  {
    double heading = 0.0;
    current_location = Waypoint{nav->latitude, nav->longitude, nav->altitude, 0.0, heading};
    calcAndSendCommands();
  }

  void WaypointFollower::calcAndSendCommands()
  {
    double cte = loc_->getCrossTrackError(current_location);
    double heading_error = loc_->getHeadingError(current_location);
    double steering = steering_controller_->getOutput(0.5 * cte + 0.5 * heading_error / (2 * M_PI));
    double target_speed = loc_->getTargetSpeed();
    auto steering_msg = Float64();
    steering_msg.data = steering;
    steering_pub_->publish(steering_msg);
    auto speed_msg = Float64();
    speed_msg.data = target_speed;
    speed_pub_->publish(speed_msg);

  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_following::WaypointFollower)