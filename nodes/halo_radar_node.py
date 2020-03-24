#!/usr/bin/env python

import rospy
from marine_msgs.msg import RadarSectorStamped
from marine_msgs.msg import RadarScanline
from marine_msgs.msg import RadarControlItem
from marine_msgs.msg import RadarControlSet
from marine_msgs.msg import KeyValue
import halo_radar.halo_radar
import datetime

class RadarPublisher:
    def __init__(self, prefix, radar):
        self.data_pub = rospy.Publisher(prefix+'/data',RadarSectorStamped,queue_size=10)
        self.state_pub = rospy.Publisher(prefix+'/state',RadarControlSet,queue_size=10)
        self.radar = radar
        self.radar.state_callback = self.onState
        self.radar.state_update_period = 2
        self.radar.data_callback = self.onData
        
        self.state_change_subscriber = rospy.Subscriber(prefix+'/change_state', KeyValue, self.state_change_callback)

    def onState(self,state):
        rcs = RadarControlSet()
        if 'status' in state:
            rci = RadarControlItem()
            rci.name = 'status'
            rci.value = state['status']
            rci.label = 'Status'
            rci.type = RadarControlItem.CONTROL_TYPE_ENUM
            rci.enums.append('standby')
            rci.enums.append('transmit')
            rcs.items.append(rci)
        if 'range' in state:
            rci = RadarControlItem()
            rci.name = 'range'
            rci.label = 'Range'
            rci.type = RadarControlItem.CONTROL_TYPE_FLOAT
            rci.value = str(state['range'])
            rci.min_value = 25
            rci.max_value = 75000
            rcs.items.append(rci)
        if 'mode' in state:
            rci = RadarControlItem()
            rci.name = 'mode'
            rci.value = state['mode']
            rci.label = 'Mode'
            rci.type = RadarControlItem.CONTROL_TYPE_ENUM
            rci.enums.append('custom')
            rci.enums.append('harbor')
            rci.enums.append('offshore')
            rci.enums.append('weather')
            rci.enums.append('bird')
            rcs.items.append(rci)
        if 'gain' in state and 'gain_mode' in state:
            rci = RadarControlItem()
            rci.name = 'gain'
            rci.label = 'Gain'
            rci.type = RadarControlItem.CONTROL_TYPE_FLOAT_WITH_AUTO
            if state['gain_mode'] == 'auto':
                rci.value = 'auto'
            else:
                rci.value = str(state['gain'])
            rci.min_value = 0
            rci.max_value = 100
            rcs.items.append(rci)
        if 'sea_clutter' in state and 'sea_clutter_mode' in state:
            rci = RadarControlItem()
            rci.name = 'sea_clutter'
            rci.label = 'Sea clutter'
            rci.type = RadarControlItem.CONTROL_TYPE_FLOAT_WITH_AUTO
            if state['sea_clutter_mode'] == 'auto':
                rci.value = 'auto'
            else:
                rci.value = str(state['sea_clutter'])
            rci.min_value = 0
            rci.max_value = 100
            rcs.items.append(rci)
        if 'auto_sea_clutter_nudge' in state:
            rci = RadarControlItem()
            rci.name = 'auto_sea_clutter_nudge'
            rci.label = 'Auto sea clutter nudge'
            rci.type = RadarControlItem.CONTROL_TYPE_FLOAT
            rci.value = str(state['auto_sea_clutter_nudge'])
            rci.min_value = -50
            rci.max_value = 50
            rcs.items.append(rci)
        if 'sea_state' in state:
            rci = RadarControlItem()
            rci.name = 'sea_state'
            rci.value = state['sea_state']
            rci.label = 'Sea state'
            rci.type = RadarControlItem.CONTROL_TYPE_ENUM
            rci.enums.append('calm')
            rci.enums.append('moderate')
            rci.enums.append('rough')
            rcs.items.append(rci)
        if 'rain_clutter' in state:
            rci = RadarControlItem()
            rci.name = 'rain_clutter'
            rci.label = 'Rain clutter'
            rci.type = RadarControlItem.CONTROL_TYPE_FLOAT
            rci.value = str(state['rain_clutter'])
            rci.min_value = 0
            rci.max_value = 100
            rcs.items.append(rci)
        
            

        self.state_pub.publish(rcs)

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

