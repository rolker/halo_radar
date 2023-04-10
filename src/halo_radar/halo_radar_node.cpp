#include <ros/ros.h>
#include <tf2/utils.h>
#include <iostream>
#include "halo_radar.h"
#include "marine_sensor_msgs/RadarSector.h"
#include "marine_radar_control_msgs/RadarControlSet.h"
#include "marine_radar_control_msgs/RadarControlValue.h"
#include "nav_msgs/Odometry.h"
#include <future>
#include "angular_speed_estimator.h"

class RosRadar : public halo_radar::Radar
{
public:
  RosRadar(halo_radar::AddressSet const &addresses) : halo_radar::Radar(addresses)
  {
    ros::NodeHandle n;
    m_data_pub = n.advertise<marine_sensor_msgs::RadarSector>(addresses.label + "/data", 10);
    m_state_pub = n.advertise<marine_radar_control_msgs::RadarControlSet>(addresses.label + "/state", 10);
    m_state_change_sub =
        n.subscribe(addresses.label + "/change_state", 10, &RosRadar::stateChangeCallback, this);
    m_heartbeatTimer = n.createTimer(ros::Duration(1.0), &RosRadar::hbTimerCallback, this);
    ros::param::param<double>("~range_correction_factor", m_rangeCorrectionFactor, m_rangeCorrectionFactor);
    ros::param::param<std::string>("~frameId", m_frame_id, m_frame_id);

    startThreads();
  }

 protected:
  void processData(std::vector<halo_radar::Scanline> const &scanlines) override
  {
    if(scanlines.empty())
      return;
    marine_sensor_msgs::RadarSector rs;
    rs.header.stamp = ros::Time::now();
    rs.header.frame_id = m_frame_id;
    rs.angle_start = 2.0*M_PI*(360-scanlines.front().angle)/360.0;
    double  angle_max = 2.0*M_PI*(360-scanlines.back().angle)/360.0;
    if(scanlines.size() > 1)
    {
      if (angle_max > rs.angle_start && angle_max-rs.angle_start > M_PI) // have we looped around (also make sure angle are decreasing)
        angle_max -= 2.0*M_PI;
      rs.angle_increment = (angle_max-rs.angle_start)/double(scanlines.size()-1);

    }
    rs.range_min = 0.0;
    rs.range_max = scanlines.front().range;
    for (auto sl : scanlines)
    {
      marine_sensor_msgs::RadarEcho echo;
      for (auto i : sl.intensities)
        echo.echoes.push_back(i/15.0); // 4 bit int to float
      rs.intensities.push_back(echo);
    }

    auto angular_speed = m_estimator.update(rs.header.stamp,  rs.angle_start);
    double scan_time = 0.0;
    if(angular_speed != 0.0)
      scan_time = 2*M_PI/fabs(angular_speed);

    rs.scan_time = ros::Duration(scan_time);

    double time_increment = 0.0;
    if (scan_time > 0)
      time_increment = std::abs(rs.angle_increment)/scan_time;
    rs.time_increment =  ros::Duration(time_increment);

    m_data_pub.publish(rs);
  }

  void stateUpdated() override
  {
    marine_radar_control_msgs::RadarControlSet rcs;

    std::string statusEnums[] = {"standby", "transmit", ""};

    createEnumControl("status", "Status", statusEnums, rcs);
    createFloatControl("range", "Range", 25, 75000, rcs);

    std::string modeEnums[] = {"custom", "harbor", "offshore", "weather", "bird", ""};

    createEnumControl("mode", "Mode", modeEnums, rcs);
    createFloatWithAutoControl("gain", "gain_mode", "Gain", 0, 100, rcs);
    createFloatWithAutoControl("sea_clutter", "sea_clutter_mode", "Sea clutter", 0, 100, rcs);
    createFloatControl("auto_sea_clutter_nudge", "Auto sea clut adj", -50, 50, rcs);

    std::string seaStateEnums[] = {"calm", "moderate", "rough", ""};

    createEnumControl("sea_state", "Sea state", seaStateEnums, rcs);
    createFloatControl("rain_clutter", "Rain clutter", 0, 100, rcs);

    std::string lowMedHighEnums[] = {"off", "low", "medium", "high", ""};

    createEnumControl("noise_rejection", "Noise rejection", lowMedHighEnums, rcs);
    createEnumControl("target_expansion", "Target expansion", lowMedHighEnums, rcs);
    createEnumControl("interference_rejection", "Interf. rej", lowMedHighEnums, rcs);
    createEnumControl("target_separation", "Target separation", lowMedHighEnums, rcs);

    std::string scanSpeedEnums[] = {"off", "medium", "high", ""};

    createEnumControl("scan_speed", "Fast scan", scanSpeedEnums, rcs);

    std::string dopplerModeEnums[] = {"off", "normal", "approaching_only", ""};

    createEnumControl("doppler_mode", "VelocityTrack", dopplerModeEnums, rcs);
    createFloatControl("doppler_speed", "Speed threshold", 0.05, 15.95, rcs);
    createFloatControl("antenna_height", "Antenna height", 0.0, 30.175, rcs);
    createFloatControl("bearing_alignment", "Bearing alignment", 0, 360, rcs);
    createFloatWithAutoControl("sidelobe_suppression", "sidelobe_suppression_mode", "Sidelobe sup.", 0, 100, rcs);
    createEnumControl("lights", "Halo light", lowMedHighEnums, rcs);

    m_state_pub.publish(rcs);
  }

 private:
  void stateChangeCallback(const marine_radar_control_msgs::RadarControlValue::ConstPtr &cv)
  {
    sendCommand(cv->key, cv->value);
  }

  void hbTimerCallback(const ros::TimerEvent &e)
  {
    if (checkHeartbeat())
      stateUpdated();
  }

  void createEnumControl(std::string const &name, std::string const &label, std::string const enums[],
                         marine_radar_control_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_radar_control_msgs::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_ENUM;
      for (int i = 0; !enums[i].empty(); i++)
        rci.enums.push_back(enums[i]);
      rcs.items.push_back(rci);
    }
  }

  void createFloatControl(std::string const &name, std::string const &label, float min_value, float max_value,
                          marine_radar_control_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_radar_control_msgs::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_FLOAT;
      rci.min_value = min_value;
      rci.max_value = max_value;
      rcs.items.push_back(rci);
    }
  }

  void createFloatWithAutoControl(std::string const &name, std::string const &auto_name, std::string const &label,
                                  float min_value, float max_value, marine_radar_control_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end() && m_state.find(auto_name) != m_state.end())
    {
      marine_radar_control_msgs::RadarControlItem rci;
      rci.name = name;
      std::string value = m_state[name];
      if (m_state[auto_name] == "auto")
        value = "auto";
      rci.value = value;
      rci.label = label;
      rci.type = marine_radar_control_msgs::RadarControlItem::CONTROL_TYPE_FLOAT_WITH_AUTO;
      rci.min_value = min_value;
      rci.max_value = max_value;
      rcs.items.push_back(rci);
    }
  }

  ros::Publisher m_data_pub;
  ros::Publisher m_state_pub;
  ros::Subscriber m_state_change_sub;
  ros::Timer m_heartbeatTimer;

  double m_rangeCorrectionFactor = 1.024;
  std::string m_frame_id = "radar";

  AngularSpeedEstimator m_estimator;
};

std::shared_ptr<halo_radar::HeadingSender> headingSender;

void odometryCallback(const nav_msgs::Odometry::ConstPtr msg)
{
  if(headingSender)
  {
    double heading = 90.0-180.0*tf2::getYaw(msg->pose.pose.orientation)/M_PI;
    headingSender->setHeading(heading);
  }
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "halo_radar");
  std::vector<std::shared_ptr<RosRadar> > radars;
  std::vector<uint32_t> hostIPs;
  if (ros::param::has("~hostIPs"))
  {
    std::vector<std::string> hostIPstrings;
    ros::param::get("~hostIPs", hostIPstrings);
    for (auto s: hostIPstrings)
      hostIPs.push_back(halo_radar::ipAddressFromString(s));
  }

  std::future<void> scanResult = std::async(std::launch::async, [&] {
    while(radars.empty())
    {
      std::vector<halo_radar::AddressSet> as;
      if(hostIPs.empty())
        as = halo_radar::scan();
      else
        as = halo_radar::scan(hostIPs);
      if(as.empty())
        ROS_WARN_STREAM("No radars found!");
      for (auto a : as)
      {
        radars.push_back(std::shared_ptr<RosRadar>(new RosRadar(a)));
        if(!headingSender)
          headingSender = std::shared_ptr<halo_radar::HeadingSender>(new halo_radar::HeadingSender(a.interface));
      }
    }
  });

  ros::spin();

  return 0;
}
