# ROS Driver for the Halo series of marine radars

This driver interfaces with Simrad HALO radar via multicast UDP communication.

## Installation

Clone this repo into your workspace and make sure you have the following dependant packages:

* marine_sensor_msgs (pending rosdistro inclusion): https://github.com/apl-ocean-engineering/marine_msgs
* marine_radar_control_messages: https://github.com/CCOMJHC/marine_radar_control_msgs

Build your workspace.

### Optional packages

Plugin for rqt to view data and control settings: https://github.com/CCOMJHC/rqt_marine_radar

## Usage

rosrun halo_radar halo_radar

By default, the driver will scan all available interfaces. To restrict which interface(s) to use, specify the list of IP local addresses using the ~hostIPs parameter.

## Troubleshooting

To make sure route is available: sudo route add -net 224.0.0.0 netmask 224.0.0.0 eth0

Switches and routers between the radar and the machine running the driver may interfere with multicast packets. Consult network equipment documentation or simply the network path.
