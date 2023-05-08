#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <marine_sensor_msgs/RadarSector.h>
#include "angular_speed_estimator.h"


int main(int argc, char* argv[])
{
  if(argc != 3)
  {
    std::cout << "Usage: fix_scan_time in.bag out.bag\n";
    exit(-1);
  }

  rosbag::Bag inbag;
  inbag.open(argv[1]);

  rosbag::Bag outbag;
  outbag.open(argv[2], rosbag::bagmode::Write);
  outbag.setCompression(rosbag::compression::LZ4);

  std::map<std::string, AngularSpeedEstimator> estimators;


  for(const auto m: rosbag::View(inbag))
  {
    auto sector = m.instantiate<marine_sensor_msgs::RadarSector>();
    if(sector != nullptr)
    {
      auto angular_speed = estimators[m.getTopic()].update(sector->header.stamp, sector->angle_start);
      double scan_time = 0.0;
      if(angular_speed != 0.0)
        scan_time = 2*M_PI/fabs(angular_speed);

      sector->scan_time = ros::Duration(scan_time);

      double time_increment = 0.0;
      if (scan_time > 0)
        time_increment = std::abs(sector->angle_increment)/scan_time;
      sector->time_increment =  ros::Duration(time_increment);

      outbag.write(m.getTopic(), m.getTime(), sector, m.getConnectionHeader());
      continue;
    }
    outbag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());

  }

  return 0;
}    
