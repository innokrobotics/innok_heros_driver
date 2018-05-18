#!/usr/bin/env python2
import can
import functools
import rospy
from std_msgs.msg import Bool
from innok_heros_driver.msg import LedState

import enum

    

can_interface = 'can0'
in_res_id = 504L
out_res_id = 632L


class SPSNode():
    def __init__(self):
        self.led = None
        self.bell = None
        self.warn_light = None
        self.front_light = None
        self.back_light = None
        self.button = False
        rospy.init_node('innok_sps_driver')
        rospy.Subscriber("front_flood_light", Bool,
                         lambda data:self.update(front_light=int(data.data)))
        rospy.Subscriber("back_flood_light", Bool,
                         lambda data:self.update(back_light=int(data.data)))
        rospy.Subscriber("warn_light", Bool,
                         lambda data:self.update(warn_light=int(data.data)))
        rospy.Subscriber("bell", Bool, 
                         lambda data:self.update(bell=int(data.data)))
        rospy.Subscriber("button_light", LedState, 
                         lambda data:self.update(led=(int(data.active), data.mode)))
        self.button_pub = rospy.Publisher("button", Bool)
        self.bus = can.interface.Bus(can_interface, bustype='socketcan')

    def run(self):
        for message in self.bus:
            if rospy.is_shutdown():
                break
            if message.arbitration_id == in_res_id:
            	self.led = (1 if message.data[0] & 0x1 else 0, #first bit=on/off
                            ((message.data[0] & (0x2 | 0x4))>>1)) #2,3 is mode
                button = 1 if message.data[0] & 0x8 else 0 #4th is pressed
                if button != self.button:
                    self.button_pub.publish(button)
                    self.button = button
                self.front_light = 1 if message.data[3] & 0x1 else 0
                self.back_light = 1 if message.data[3] & 0x2 else 0
                self.bell = message.data[1]
                self.warn_light = message.data[2]
                print('led %s, lights %s, bell %s, warn_light %s' % (self.led, (self.front_light, self.back_light), self.bell, self.warn_light))
             
            
    def update(self, led=None, bell=None, warn_light=None, front_light=None, back_light=None):
        led = self.led if led is None else led
        bell = self.bell if bell is None else bell
        warn_light = self.warn_light if warn_light is None else warn_light
        front_light = self.front_light if front_light is None else front_light
        back_light = self.back_light if back_light is None else back_light
        msg = can.Message(arbitration_id=632L, data=[led[0] | led[1]<<1,
                                                     bell,
                                                     warn_light,
                                                     (back_light<<1) | front_light
                                                    ], extended_id=True)
        self.bus.send(msg)


if __name__ == '__main__':
    n = SPSNode()
    n.run()

