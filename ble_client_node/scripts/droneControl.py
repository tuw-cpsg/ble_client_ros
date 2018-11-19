#!/usr/bin/python

import pygatt
import time
import rospy
from sensor_msgs.msg import Joy
from functools import partial


# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben


ADDRESS_TYPE   = pygatt.BLEAddressType.random

adapter = pygatt.GATTToolBackend()

device  = 0
thrust  = 0
yaw     = 1024
pitch   = 1024
roll    = 1024
arm = 0
acro = 0
watchdog = 0;
watchdog_old = 0;


def calcbytes():
    global thrustH, thrustL, yawH, yawL, pitchH, pitchL, rollH, rollL, armH, armL, acroH, acroL
    
    thrustH = thrust >> 8
    thrustL = 0x00FF & thrust 

    yawH = yaw >> 8
    yawL = 0x00FF & yaw

    pitchH = pitch >> 8
    pitchL = 0x00FF & pitch

    rollH = roll >> 8
    rollL = 0x00FF & roll

    armH = arm >> 8
    armL = 0x00FF & arm

    acroH = acro >> 8
    acroL = 0x00FF & acro


def joy_callback(data):
    global yaw,pitch,roll,thrust, arm, acro, watchdog

    watchdog = data.header.seq;
    yaw = int((((-1*data.axes[0])+1))*1023.5)

    thrust = 0 if (data.axes[1] < 0.0) else int((data.axes[1])*2047)
    roll  = int((((-1*data.axes[3])+1))*1023.5)  
    pitch = int((data.axes[4]+1)*1023.5);
    arm   = data.buttons[4]*2047
    acro  = data.buttons[5]*2047


def connect():
    DEVICE_ADDRESS = rospy.get_param('~device')  
    rospy.loginfo("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        rospy.loginfo("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None;

def sendControlData(device):
    try: 
        calcbytes()
        device.char_write_handle(0x0014, [thrustL, thrustH, rollL, rollH, pitchL, pitchH, yawL, yawH, armL, armH, acroL, acroH]) 
        # rospy.loginfo("%d %d %d %d %d %d %d %d %d %d %d %d", thrustH, thrustL, yawH, yawL, pitchH, pitchL, rollH, rollL, armH, armL, acroH, acroL)
        return True
    except pygatt.exceptions.NotConnectedError:
        return False;


def main(): 
    global watchdog_old
    try:
        rospy.init_node('droneControl')
        rospy.loginfo("Starting adapter");
        adapter.start()
        rospy.Subscriber('/joy', Joy, joy_callback)
        rate = rospy.Rate(90,9090909090) # 11ms

        connected = 0;
        while not rospy.is_shutdown():
            
            if not connected:
                rospy.loginfo("Connection faild ")
                device = connect()
                connected = True if device is not None else False

            if watchdog_old < watchdog: # only send Data if we get new Data From Joy
                watchdog_old = watchdog
                if connected:
                    connected = sendControlData(device)
                

            rate.sleep()
    finally:
        adapter.stop()

if __name__ == '__main__':
    main()
