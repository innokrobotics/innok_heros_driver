#!/usr/bin/env python2
import can
import functools
import rospy
from std_msgs.msg import Bool, Float32
from innok_heros_driver.msg import LedState
import numpy as np

import enum

    

can_interface = 'can0'
in12_res_id = 515L
in34_res_id = 771L
out_res_id = 643L

motor_max = 1500
motor_min = 100


def parse_content(message):
    message = np.array(message)
    status = message[0]
    pos = (message[1] << 8)  | message[2]
    return status, pos

class Nokia_Linear_Motors():
    def __init__(self):
        rospy.init_node('nokia_linear_motors')
        #self.button_pub = rospy.Publisher("button", Bool)
        self.bus = can.interface.Bus(can_interface, bustype='socketcan')
        speed = 1.0
        handle_set_fork = lambda f : self.set_motor(1, f.data, speed)
        handle_set_tilt = lambda f : self.set_motor(0, f.data, speed)
        rospy.Subscriber('fork_position', Float32, handle_set_fork)
        rospy.Subscriber('container_position', Float32, handle_set_tilt)

    def run(self):
        for message in self.bus:
            if rospy.is_shutdown():
                break
            if message.arbitration_id == in12_res_id:
                self.m1_status, self.m1_pos = parse_content(message.data[0:3])
                self.m2_status, self.m2_pos = parse_content(message.data[3:6])
                #print('m1_pos %s\nm2_pos %s' % (self.m1_pos, self.m2_pos))

            elif message.arbitration_id == in34_res_id:
                self.m3_status, self.m3_pos = parse_content(message.data[0:3])
                self.m4_status, self.m4_pos = parse_content(message.data[3:6])
                #print('m3_pos %s\nm4_pos %s' % (self.m3_pos, self.m4_pos))

             
            
    def update(self, motor_select, pos, speed):
        print(motor_select, pos, speed)
        msg = can.Message(arbitration_id=out_res_id,
                          data=[motor_select,
                                ((pos &0xFF00)>>8),
                                (pos & 0xFF),
                                speed], extended_id=False)
        self.bus.send(msg)

    def set_motor(self, select, pos, speed):
        if pos<0.0 or pos>1.0:
            return False
        if speed<0.0 or speed>1.0:
            return False
        pos = int(pos*(motor_max-motor_min)+motor_min)
        speed = int(speed*0xFF)
        select = int(select)
        self.update(select, pos, speed)


if __name__ == '__main__':
    n = Nokia_Linear_Motors()
    n.run()

