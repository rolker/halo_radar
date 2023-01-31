#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <marine_sensor_msgs/RadarSector.h>

int main(int argc, char* argv[])
{
  if(argc != 3)
  {
    std::cout << "Usage: fix_angle_increments in.bag out.bag\n";
    exit(-1);
  }

  rosbag::Bag inbag;
  inbag.open(argv[1]);
  
  rosbag::Bag outbag;
  outbag.open(argv[2], rosbag::bagmode::Write);
  outbag.setCompression(rosbag::compression::LZ4);

  for(const auto m: rosbag::View(inbag))
  {
    auto sector = m.instantiate<marine_sensor_msgs::RadarSector>();
    if(sector != nullptr && sector->intensities.size() > 1)
    {
      if(sector->angle_increment > 0.0)
      {
        auto angle_finish = sector->angle_start + sector->angle_increment*(sector->intensities.size()-1);
        angle_finish -= 2*M_PI;
        sector->angle_increment = (angle_finish - sector->angle_start)/float(sector->intensities.size()-1);
        outbag.write(m.getTopic(), m.getTime(), sector, m.getConnectionHeader());
        continue;
      }
    }
    outbag.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
  }

  return 0;
}    
