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

    def createEnumControl(self, name, value, label, enums):
        rci = RadarControlItem()
        rci.name = name
        rci.value = value
        rci.label = label
        rci.type = RadarControlItem.CONTROL_TYPE_ENUM
        for e in enums:
            rci.enums.append(e)
        return rci
    
    def createFloatControl(self, name, value, label, min_value = 0.0, max_value = 0.0):
        rci = RadarControlItem()
        rci.name = name
        rci.value = str(value)
        rci.label = label
        rci.type = RadarControlItem.CONTROL_TYPE_FLOAT
        rci.min_value = min_value
        rci.max_value = max_value
        return rci
    
    def createFloatWithAutoControl(self, name, value, label, min_value = 0.0, max_value = 0.0):
        rci = RadarControlItem()
        rci.name = name
        rci.value = value
        rci.label = label
        rci.type = RadarControlItem.CONTROL_TYPE_FLOAT_WITH_AUTO
        rci.min_value = min_value
        rci.max_value = max_value
        return rci

    def onState(self,state):
        rcs = RadarControlSet()
        if 'status' in state:
            rcs.items.append(self.createEnumControl('status',state['status'],'Status',('standby','transmit')))
        if 'range' in state:
            rcs.items.append(self.createFloatControl('range',state['range'],'Range',25,75000))
        if 'mode' in state:
            rcs.items.append(self.createEnumControl('mode',state['mode'],'Mode',('custom','harbor','offshore','weather','bird')))
        if 'gain' in state and 'gain_mode' in state:
            if state['gain_mode'] == 'auto':
                value = 'auto'
            else:
                value = str(state['gain'])
            rcs.items.append(self.createFloatWithAutoControl('gain',value,'Gain',0,100))
        if 'sea_clutter' in state and 'sea_clutter_mode' in state:
            if state['sea_clutter_mode'] == 'auto':
                value = 'auto'
            else:
                value = str(state['sea_clutter'])
            rcs.items.append(self.createFloatWithAutoControl('sea_clutter',value,'Sea clutter',0,100))
        if 'auto_sea_clutter_nudge' in state:
            rcs.items.append(self.createFloatControl('auto_sea_clutter_nudge',state['auto_sea_clutter_nudge'],'Auto sea clutter nudge',-50,50))
        if 'sea_state' in state:
            rcs.items.append(self.createEnumControl('sea_state',state['sea_state'],'Sea state',('calm','moderate','rough')))
        if 'rain_clutter' in state:
            rcs.items.append(self.createFloatControl('rain_clutter',state['rain_clutter'],'Rain clutter',0,100))
        if 'noise_rejection' in state:
            rcs.items.append(self.createEnumControl('noise_rejection',state['noise_rejection'],'Noise rejection',('off','low','medium','high')))
        if 'target_expansion' in state:
            rcs.items.append(self.createEnumControl('target_expansion',state['target_expansion'],'Target expansion',('off','low','medium','high')))
        if 'interference_rejection' in state:
            rcs.items.append(self.createEnumControl('interference_rejection',state['interference_rejection'],'Interf. rej',('off','low','medium','high')))
        if 'target_separation' in state:
            rcs.items.append(self.createEnumControl('target_separation',state['target_separation'],'Target separation',('off','low','medium','high')))
        if 'scan_speed' in state:
            rcs.items.append(self.createEnumControl('scan_speed',state['scan_speed'],'Fast scan',('off','medium','high')))
        if 'doppler_mode' in state:
            rcs.items.append(self.createEnumControl('doppler_mode',state['doppler_mode'],'VelocityTrack',('off','normal','approaching_only')))
        if 'doppler_speed' in state:
            rcs.items.append(self.createFloatControl('doppler_speed',state['doppler_speed'],'Speed Threshold',0.05,15.95))
        if 'antenna_height' in state:
            rcs.items.append(self.createFloatControl('antenna_height',state['antenna_height'],'Antenna height',0.0,30.175))
        if 'bearing_alignment' in state:
            rcs.items.append(self.createFloatControl('bearing_alignment',state['bearing_alignment'],'Bearing alignment',0,365))
        if 'sidelobe_suppression' in state and 'sidelobe_suppression_mode' in state:
            if state['sidelobe_suppression_mode'] == 'auto':
                value = 'auto'
            else:
                value = str(state['sidelobe_suppression'])
            rcs.items.append(self.createFloatWithAutoControl('sidelobe_suppression',value,'Sidelobe sup.',0,100))
        if 'light' in state:
            rcs.items.append(self.createEnumControl('light',state['light'],'Halo light',('off','low','medium','high')))

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

