#!/usr/bin/python

import pygatt
import time
import logging

import rospy
from sensor_msgs.msg import Joy
from functools import partial

# The BGAPI backend will attemt to auto-discover the serial device name of the
# attached BGAPI-compatible USB adapter.
adapter = pygatt.GATTToolBackend()

device = 0
direction = 0x32
thrust = 0

def joy_callback(data):

    global device
    global direction
    global thrust

    direction = int(max(0, min(0x63, 0x32 + data.axes[3] * 0x31)));
    thrust = int(max(0, min(0xfe, data.axes[1] * 0xfe)));

def main():

    global device
    global direction
    global thrust

    log = logging.getLogger(__name__)

    try:
        adapter.start()
        device = adapter.connect('C4:BE:84:E9:B4:DF');
        time.sleep(5);
    
	rospy.init_node('bionicbird_control')
    	rospy.Subscriber('/joy', Joy, joy_callback)
	rate = rospy.Rate(10) # 10hz

    	while not rospy.is_shutdown():
#            log.info("Sending [%x %x]", direction, thrust);
            device.char_write_handle(0x002a, [direction, thrust], False);
            rate.sleep()
	rospy.spin()
    
    finally:
        adapter.stop()

if __name__ == '__main__':
    main()
