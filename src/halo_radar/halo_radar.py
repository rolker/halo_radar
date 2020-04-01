#!/usr/bin/env python

import socket
import struct
import datetime
import netifaces
import select

query_group = '236.6.7.5'
query_port = 6878

def bytesToIPAddress(b):
    return (str(b[0])+'.'+str(b[1])+'.'+str(b[2])+'.'+str(b[3]),b[4])

def decode(data):
    if len(data) > 2:
        id = struct.unpack('!H',data[:2])
        #print 'id:',id,hex(id[0])
        if id[0] == 0x01b2:
            serialno = struct.unpack('!16s',data[2:18])
            dataAddressA = struct.unpack('!4BH',data[88:94])
            sendAddressA = struct.unpack('!4BH',data[98:104])
            reportAddressA = struct.unpack('!4BH',data[108:114])
            dataAddressB = struct.unpack('!4BH',data[124:130])
            sendAddressB = struct.unpack('!4BH',data[134:140])
            reportAddressB = struct.unpack('!4BH',data[144:150])
            return ({'data':bytesToIPAddress(dataAddressA),
                     'send':bytesToIPAddress(sendAddressA),
                     'report':bytesToIPAddress(reportAddressA),
                     'label':'HaloA'},
                    {'data':bytesToIPAddress(dataAddressB),
                     'send':bytesToIPAddress(sendAddressB),
                     'report':bytesToIPAddress(reportAddressB),
                     'label':'HaloB'})

def scan():
    for i in netifaces.interfaces():
        #print '  ',i  
        addresses = netifaces.ifaddresses(i)
        if netifaces.AF_INET in addresses:
            for a in addresses[netifaces.AF_INET]:
                #print '    ',a['addr']
                listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                listen_sock.settimeout(1)
                listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listen_sock.bind(('',query_port))
                # Tell the operating system to add the socket to
                # the multicast group on all interfaces.
                group = socket.inet_aton(query_group)
                mreq = struct.pack('4s4s', group, socket.inet_aton(a['addr']))
                listen_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

            
                data = struct.pack('<BB',0x01,0xb1)
                send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                send_sock.bind((a['addr'],0))
                send_sock.sendto(data,(query_group,query_port))

                for i in range(2):
                    try:
                        data, address = listen_sock.recvfrom(100000)
                        if len(data) > 2:
                            #print len(data),'bytes from',address
                            return a['addr'],decode(data)
                    except socket.error:
                        pass


class HaloRadar:
    def __init__(self, interface, dataAddress, sendAddress, reportAddress, label):
        self.sendAddress = sendAddress

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.send_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.send_socket.bind((interface,0))
        
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data_socket.bind(('',dataAddress[1]))
        data_mreq = struct.pack('4s4s',socket.inet_aton(dataAddress[0]),socket.inet_aton(interface))
        self.data_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, data_mreq)
        
        self.report_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.report_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.report_socket.bind(('',reportAddress[1]))
        report_mreq = struct.pack('4s4s',socket.inet_aton(reportAddress[0]),socket.inet_aton(interface))
        self.report_socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, report_mreq)
        
        self.state = {'label':label, 'status':'unknown'}
        
        self.old_data = None
        self.lastAliveSent = None
        
        self.data_callback = None
        self.state_callback = None
        self.state_update_period = None
        self.last_state_update_time = None

    def checkSockets(self):
        self.checkAlive()
        ready =  select.select((self.data_socket,self.report_socket),(),(),0.1)
        for s in ready[0]:
            data,address = s.recvfrom(100000)
            if s.fileno() == self.data_socket.fileno():
                self.processData(data)
            else:
                self.processReport(data)

    def printData(self,data):
        unpack_str = str(len(data))+'B'
        bs = struct.unpack(unpack_str,data)
        hex_str = []
        for b in bs:
            hex_str.append(hex(b))
        print hex_str
        
    def detectChange(self,data):
        if self.old_data is not None:
            for i in range(len(data)):
                if data[i] != self.old_data[i]:
                    print i,hex(ord(self.old_data[i])),'->',hex(ord(data[i])),'\t',bin(ord(self.old_data[i])),'->',bin(ord(data[i]))
        self.old_data = data
            
    def processReport(self,data):
        id = struct.unpack('<H',data[:2])
        #print '\tid:',id,hex(id[0])
        new_state = {}
        if id[0] == 0xC401:
            radar_status = struct.unpack('B',data[2])
            new_state['status'] = {1:'standby',2:'transmit',5:'spinning_up'}[radar_status[0]]
                               
        elif id[0] == 0xC402:
            range_dm,mode = struct.unpack('<IxB',data[2:8])
            new_state['range'] = range_dm/10
            
            new_state['mode'] = {0:'custom',1:'harbor',2:'offshore',4:'weather',5:'bird'}[mode]

            gain_mode,gain = struct.unpack('<B3xB',data[8:13])
            new_state['gain'] = gain*100/255
            new_state['gain_mode'] = {0:'manual',1:'auto'}[gain_mode]

            sea_mode,sea = struct.unpack('B3xB',data[13:18])
            new_state['sea_clutter'] = sea*100/255
            new_state['sea_clutter_mode'] = {0:'manual',1:'auto'}[sea_mode]

            rain = struct.unpack('B',data[22])
            new_state['rain_clutter'] = rain[0]*100/255
            
            interference_rejection = struct.unpack('B',data[34])
            new_state['interference_rejection'] = {0:'off',1:'low',2:'medium',3:'high'}[interference_rejection[0]]
            
            target_expansion = struct.unpack('B',data[38])
            new_state['target_expansion'] = {0:'off',1:'low',2:'medium',3:'high'}[target_expansion[0]]
            
        elif id[0] == 0xC403:
            pass
        
        elif id[0] == 0xC404:
            b_alignment = struct.unpack('<H',data[6:8])
            new_state['bearing_alignment'] = b_alignment[0]/10.0
            
            a_height = struct.unpack('<H',data[10:12])
            new_state['antenna_height'] = a_height[0]/1000.0
            
            halo_light = struct.unpack('B',data[19])
            new_state['lights'] = {0:'off',1:'low',2:'medium',3:'high'}[halo_light[0]]

        elif id[0] == 0xC406:
            pass
        
        elif id[0] == 0xC408:
            new_state['sea_state'] = {0:'calm',1:'moderate',2:'rough'}[ord(data[2])]
            
            new_state['scan_speed'] = {0:'off',1:'medium',3:'high'}[ord(data[4])]
            
            new_state['sidelobe_suppression_mode'] = {0:'manual',1:'auto'}[ord(data[5])]
            new_state['sidelobe_suppression'] = ord(data[9])*100/255
            
            new_state['noise_rejection'] = {0:'off',1:'low',2:'medium',3:'high'}[ord(data[12])]
            new_state['target_separation'] = {0:'off',1:'low',2:'medium',3:'high'}[ord(data[13])]
            
            sea_clutter_nudge = struct.unpack('b',data[15])
            new_state['auto_sea_clutter_nudge'] = sea_clutter_nudge[0]
            
            
            if len(data) >= 21:
                doppler_state,doppler_speed = struct.unpack('<BH',data[18:21])
                new_state['doppler_mode'] = {0:'off',1:'normal',2:'approaching_only'}[doppler_state]
                new_state['doppler_speed'] = doppler_speed/100.0
            
        elif id[0] == 0xC409:
            pass

        elif id[0] == 0xC40A:
            pass
            
        elif id[0] == 0xC611:
            self.detectChange(data)
            pass #heartbeat
        else:
            print '\tid:',hex(id[0])
            pass

        state_updated = False
        for k in new_state.keys():
            if not k in self.state or self.state[k] != new_state[k]:
                self.state[k] = new_state[k]
                state_updated = True

        now = datetime.datetime.now()
        if state_updated and self.state_callback is not None:
            self.state_callback(self.state)
            self.last_state_update_time = now
            
        if self.state_update_period is not None and (self.last_state_update_time is None or now-self.last_state_update_time >= datetime.timedelta(seconds=self.state_update_period)):
            if self.state_callback is not None:
                self.state_callback(self.state)
            self.last_state_update_time = now
        #print self.state

    def processData(self,data):
        #print self.state['label'],'data:',len(data),'bytes'
        scanline_count,scanline_size = struct.unpack('<BH', data[5:8])
        #print 'ns:',scanline_count,'ss:',scanline_size
        
        sector = {'scanlines':[]}
        for i in range(scanline_count):
            cursor = 8+(i*(scanline_size+24))
            
            scanline = {}
            
            status,scan_number,large_range,angle,small_range,rotation = struct.unpack('<xBH2xHH2xHH', data[cursor:cursor+16])

            if status == 2: #valid
                
                #print 'status',status,'scan_number',scan_number,'large_range',large_range,'angle',angle,'small_range',small_range,'rotation',rotation
                if large_range == 128:
                    if small_range == -1:
                        scanline['range'] = 0
                    else:
                        scanline['range'] = small_range/4
                else:
                    scanline['range'] = large_range*small_range/512
                scanline['angle'] = angle*360.0/4096
                scanline['intensities'] = []
                for d in data[cursor+24:cursor+24+scanline_size]:
                    low = ord(d)&0x0f;
                    high = (ord(d)&0xf0)>>4;
                    scanline['intensities'].append(low)
                    scanline['intensities'].append(high)
                sector['scanlines'].append(scanline)
            #break
        #for sl in sector['scanlines']:
            #ray = ''
            #for i in sl['intensities'][:390]:
                #if i > 10:
                    #ray += '*'
                #else:
                    #ray += ' '
            #print ray
        if self.data_callback is not None:
            self.data_callback(sector)
        
    def sendCommandData(self,data):
        self.send_socket.sendto(data,self.sendAddress)

    def on(self):
        self.sendCommandData(struct.pack('!BBB',0x00,0xc1,0x01))
        self.sendCommandData(struct.pack('!BBB',0x01,0xc1,0x01))

    def off(self):
        self.sendCommandData(struct.pack('!BBB',0x00,0xc1,0x01))
        self.sendCommandData(struct.pack('!BBB',0x01,0xc1,0x00))

    def stayAlive(self):
        self.sendCommandData(struct.pack('!BB',0xA0,0xC1))
        self.sendCommandData(struct.pack('!BB',0x03,0xC2))
        self.sendCommandData(struct.pack('!BB',0x04,0xC2))
        self.sendCommandData(struct.pack('!BB',0x05,0xC2))

    def checkAlive(self):
        now = datetime.datetime.now()
        if self.lastAliveSent is None or now - self.lastAliveSent > datetime.timedelta(seconds=1):
            self.stayAlive()
            self.lastAliveSent = now
            #print 'staying alive!'
            
    def sendCommand(self, cmd, value):
        if cmd == 'status':
            if value == 'transmit':
                self.on()
            elif value == 'standby':
                self.off()
            else:
                print 'invalid status command'
        elif cmd == 'range':
            dm = int(float(value)*10)
            data = struct.pack('<HI',0xC103,dm)
            self.sendCommandData(data)
        elif cmd == 'bearing_alignment':
            ba = float(value)
            if ba < 0:
                ba += 360.0
            ba *= 10
            data = struct.pack('<HH',0xc105,ba)
            self.sendCommandData(data)
        elif cmd == 'gain':
            try:
                auto = False
                g = 0
                if value == 'auto':
                    auto = True
                else:
                    g = float(value)*255/100
                data = struct.pack('<H4xB3xB',0xc106,auto,g)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'sea_clutter':
            try:
                auto = False
                sc = 0
                if value == 'auto':
                    auto = True
                else:
                    sc = float(value)*255/100
                data = struct.pack('<HB3xB3xB',0xc106,0x02,auto,sc)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'rain_clutter':
            rc = float(value)*255/100
            data = struct.pack('<HB7xB',0xc106,0x04,rc)
            self.sendCommandData(data)
        elif cmd == 'sidelobe_suppression':
            try:
                auto = False
                ss = 0
                if value == 'auto':
                    auto = True
                else:
                    ss = float(value)*255/100
                data = struct.pack('<HB3xB3xB',0xc106,0x05,auto,ss)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'interference_rejection':
            try:
                v = ('off','low','medium','high').index(value)
                data = struct.pack('<HB',0xC108,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'sea_state':
            try:
                v = ('calm','moderate','rough').index(value)
                data = struct.pack('<HB',0xC10b,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'scan_speed':
            try:
                v = ('off','medium','high').index(value)
                if v == 2:
                    v+=1
                data = struct.pack('<HB',0xC10f,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'mode':
            try:
                v = ('custom','harbor','offshore','blank','weather','bird').index(value)
                data = struct.pack('<HB',0xC110,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'auto_sea_clutter_nudge':
            ascn = float(value)
            data = struct.pack('<HBbbB',0xC111,0x01,ascn,ascn,0x04)
            self.sendCommandData(data)
        elif cmd == 'target_expansion':
            try:
                v = ('off','low','medium','high').index(value)
                data = struct.pack('<HB',0xC112,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'noise_rejection':
            try:
                v = ('off','low','medium','high').index(value)
                data = struct.pack('<HB',0xC121,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'target_separation':
            try:
                v = ('off','low','medium','high').index(value)
                data = struct.pack('<HB',0xC122,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'doppler_mode':
            try:
                v = ('off','normal','approaching_only').index(value)
                data = struct.pack('<HB',0xC123,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        elif cmd == 'doppler_speed':
            ds = float(value)*100
            data = struct.pack('<HH',0xC124,ds)
            self.sendCommandData(data)
        elif cmd == 'antenna_height':
            mm = float(value)*1000
            data = struct.pack('<HII',0xC130,1,mm)
            self.sendCommandData(data)
        elif cmd == 'lights':
            try:
                v = ('off','low','medium','high').index(value)
                data = struct.pack('<HB',0xC131,v)
                self.sendCommandData(data)
            except ValueError:
                pass
        else:
            print 'unknown command:',cmd
            

def printSector(sector):
    for sl in sector['scanlines']:
        ray = ''
        for i in sl['intensities'][:390]:
            if i > 10:
                ray += '*'
            else:
                ray += ' '
        print ray

def printState(state):
    keys = state.keys()
    keys.sort()
    for k in keys:
        print "'"+k+"':",str(state[k])+',',
    print

if __name__ == '__main__':
    import sys

    found_radars = scan()
    radars = []
    if found_radars is not None:
        i = found_radars[0]
        print i
        for r in found_radars[1]:
            print '\t',r
            radars.append(HaloRadar(i,r['data'],r['send'],r['report'],r['label']))
        if len(sys.argv) > 1 and sys.argv[1] == 'off':
            for r in radars:
                r.off()
        else:
            #radars[0].data_callback = printSector
            radars[0].state_callback = printState
            radars[0].state_update_period = 2
            for r in radars:
                r.on()
            radars[0].sendCommand('mode','custom')
            while True:
                for r in radars:
                    r.checkSockets()
                    break


