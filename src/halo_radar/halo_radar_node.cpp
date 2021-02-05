#include <ros/ros.h>
#include <iostream>
#include "halo_radar.h"
#include "marine_msgs/KeyValue.h"
#include "marine_msgs/RadarSectorStamped.h"
#include "marine_msgs/RadarControlSet.h"
#include <future>

class RosRadar : public halo_radar::Radar
{
 public:
  RosRadar(halo_radar::AddressSet const &addresses) : halo_radar::Radar(addresses)
  {
    ros::NodeHandle n;
    m_data_pub = n.advertise<marine_msgs::RadarSectorStamped>("radar/" + addresses.label + "/data", 10);
    m_state_pub = n.advertise<marine_msgs::RadarControlSet>("radar/" + addresses.label + "/state", 10);
    m_state_change_sub =
        n.subscribe("radar/" + addresses.label + "/change_state", 10, &RosRadar::stateChangeCallback, this);
    m_heartbeatTimer = n.createTimer(ros::Duration(1.0), &RosRadar::hbTimerCallback, this);
    m_rangeCorrectionFactor = n.param("range_correction_factor", m_rangeCorrectionFactor);

    startThreads();
  }

 protected:
  void processData(std::vector<halo_radar::Scanline> const &scanlines) override
  {
    marine_msgs::RadarSectorStamped rss;
    rss.header.stamp = ros::Time::now();
    rss.header.frame_id = "radar";
    for (auto sl : scanlines)
    {
      marine_msgs::RadarScanline rs;
      rs.angle = sl.angle;
      rs.range = m_rangeCorrectionFactor * sl.range;
      for (auto i : sl.intensities)
        rs.intensities.push_back(i);
      rss.sector.scanlines.push_back(rs);
    }
    m_data_pub.publish(rss);
  }

  void stateUpdated() override
  {
    marine_msgs::RadarControlSet rcs;

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
  void stateChangeCallback(const marine_msgs::KeyValue::ConstPtr &kv)
  {
    sendCommand(kv->key, kv->value);
  }

  void hbTimerCallback(const ros::TimerEvent &e)
  {
    if (checkHeartbeat())
      stateUpdated();
  }

  void createEnumControl(std::string const &name, std::string const &label, std::string const enums[],
                         marine_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_msgs::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_msgs::RadarControlItem::CONTROL_TYPE_ENUM;
      for (int i = 0; !enums[i].empty(); i++)
        rci.enums.push_back(enums[i]);
      rcs.items.push_back(rci);
    }
  }

  void createFloatControl(std::string const &name, std::string const &label, float min_value, float max_value,
                          marine_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end())
    {
      marine_msgs::RadarControlItem rci;
      rci.name = name;
      rci.value = m_state[name];
      rci.label = label;
      rci.type = marine_msgs::RadarControlItem::CONTROL_TYPE_FLOAT;
      rci.min_value = min_value;
      rci.max_value = max_value;
      rcs.items.push_back(rci);
    }
  }

  void createFloatWithAutoControl(std::string const &name, std::string const &auto_name, std::string const &label,
                                  float min_value, float max_value, marine_msgs::RadarControlSet &rcs)
  {
    if (m_state.find(name) != m_state.end() && m_state.find(auto_name) != m_state.end())
    {
      marine_msgs::RadarControlItem rci;
      rci.name = name;
      std::string value = m_state[name];
      if (m_state[auto_name] == "auto")
        value = "auto";
      rci.value = value;
      rci.label = label;
      rci.type = marine_msgs::RadarControlItem::CONTROL_TYPE_FLOAT_WITH_AUTO;
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "halo_radar");
  std::vector<std::shared_ptr<RosRadar> > radars;
  std::future<void> scanResult = std::async(std::launch::async, [&] {
    auto as = halo_radar::scan();
    for (auto a : as)
    {
      radars.push_back(std::shared_ptr<RosRadar>(new RosRadar(a)));
    }
  });

  ros::spin();

  return 0;
}
