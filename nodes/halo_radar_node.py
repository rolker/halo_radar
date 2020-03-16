#!/usr/bin/env python

import rospy
from marine_msgs.msg import RadarSectorStamped
from marine_msgs.msg import RadarScanline
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
import halo_radar.halo_radar
import datetime

class RadarPublisher:
    def __init__(self, prefix, radar):
        self.data_pub = rospy.Publisher(prefix+'/data',RadarSectorStamped,queue_size=10)
        self.state_pub = rospy.Publisher(prefix+'/state',Heartbeat,queue_size=10)
        self.radar = radar
        self.radar.state_callback = self.onState
        self.radar.state_update_period = 2
        self.radar.data_callback = self.onData
        
        self.state_change_subscriber = rospy.Subscriber(prefix+'/change_state', KeyValue, self.state_change_callback)

    def onState(self,state):
        keys = state.keys()
        keys.sort()
        hb = Heartbeat()
        hb.header.stamp = rospy.get_rostime()
        for k in keys:
            hb.values.append(KeyValue(k,str(state[k])))
        self.state_pub.publish(hb)
                             
            

    def onData(self,data):
        if data is not None:
            sector = RadarSectorStamped()
            sector.header.stamp = rospy.get_rostime()
            sector.header.frame_id = 'radar'

            for sl in data['scanlines']:
                scanline = RadarScanline()
                scanline.angle = sl['angle']
                scanline.range = sl['range']
                scanline.intensities = sl['intensities']
                sector.sector.scanlines.append(scanline)
            self.data_pub.publish(sector)
            
    def state_change_callback(self,data):
        self.radar.sendCommand(data.key,data.value)

if __name__ == '__main__':
    rospy.init_node('halo_radar')
    found_radars = halo_radar.halo_radar.scan()
    radars = []
    if found_radars is not None:
        i = found_radars[0]
        print i
        for r in found_radars[1]:
            print '\t',r
            radars.append(RadarPublisher('/base/radar/'+r['label'], halo_radar.halo_radar.HaloRadar(i,r['data'],r['send'],r['report'],r['label'])))

        #for r in radars:
            #r.radar.on()
        while not rospy.is_shutdown():
            for r in radars:
                r.radar.checkSockets()

