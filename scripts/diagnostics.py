#!/usr/bin/env python

import diagnostic_msgs
import diagnostic_updater
import rospy
import socket
import can
import struct
import bitstruct
import collections
from enum import Enum


class SwitchState(Enum):
    Up = 1
    Center = 0
    Down = -1

class BinaryButtonState(Enum):
    Inactive = 0
    Active = 1
    
class DriveState(Enum):
    Standby = 0
    Ready = 1
    NoBreak = 2
    EmergencyStop = 3
    EmergencyStopRemote = 4
    Timeout = 5
    Error = 6
    Error_Reset = 7
    SyncReady = 8
    NotDefined = 64  
    
class Controller(Enum):
    RC = 0
    HalfAutonomous = 1
    Autonomous = 2
    GenericController1 = 3
    GenericController2 = 4
    GenericController3 = 5
    NoController = 6
    
    
class Motorbox:
    def __init__(self, hardware_id):
        self.diagnostics = diagnostic_updater.Updater()
        self.diagnostics.setHardwareID(hardware_id)
        self.diagnostics.add("Motorbox Status Updater", self.generateDiagnosticMsg)

        self.dict = collections.OrderedDict()
        self.dict['state'] = DriveState.NotDefined.name
        self.dict['error_flags'] = 0
        self.dict['motor_left_alarm'] = 0
        self.dict['motor_left_warning'] = 0
        self.dict['motor_left_error'] = 0
        self.dict['motor_right_alarm'] = 0
        self.dict['motor_right_warning'] = 0
        self.dict['motor_right_error'] = 0
        self.dict['remoteEstopFlag'] = BinaryButtonState.Inactive.name
        self.dict['contactor'] = BinaryButtonState.Inactive.name
        self.dict['activeController'] = Controller.NoController.name
        self.dict['controllerSelector'] = Controller.NoController.name
        self.dict['driveMsgDelta'] = 0
        self.dict['validMsgCount'] = 0
        self.dict['looptime'] = 0
        
        if hardware_id == 'node_id: 1':
            self.dict['batterySOC'] = 0
            self.dict['batteryVoltage'] = 0.0
        
    def receiveStateInfo_1(self, data):
        # todo gscheite namen
        
        
        self.dict['state'] = DriveState(data[7]).name
        self.dict['error_flags'] = data[6]

        self.dict['motor_right_alarm'] = data[0]
        self.dict['motor_right_warning'] = data[1]
        self.dict['motor_right_error'] = data[4]
        
        self.dict['motor_left_alarm'] = data[2]
        self.dict['motor_left_warning'] = data[3]
        self.dict['motor_left_error'] = data[5]
        
        self.diagnostics.update()
        
    def receiveStateInfo_2(self, data):
        # todo gscheite namen
        # todo fix remote estop flag bug in firmware:
        # self.dict['remoteEstopFlag'] = BinaryButtonState(data[0]).name
        
        self.dict['contactor'] = BinaryButtonState(data[1]).name
        self.dict['activeController'] = Controller(data[5]).name
        self.dict['controllerSelector'] = Controller(data[6]).name
        self.dict['driveMsgDelta'] = data[3]
        self.dict['validMsgCount'] = data[4]
        self.dict['looptime'] = data[7]
        
        self.diagnostics.update()

                       

    def receiveBatteryInfo(self, data):
        
        soc, voltage = struct.unpack('<BH', data)
        
        self.dict['batterySOC'] = soc
        self.dict['batteryVoltage'] = (voltage / 100.0)
        self.diagnostics.update()
    
    def generateDiagnosticMsg(self, stat):
        
        status = diagnostic_msgs.msg.DiagnosticStatus.OK
        status_msg = "Motorbox Status:"
        
        if self.dict['state'] == DriveState.EmergencyStop.name or self.dict['state'] == DriveState.EmergencyStopRemote.name:
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            
            status_msg += ' Motorbox is in state {}!'.format(self.dict['state'])
        
        if self.dict['motor_left_warning'] is not 0 or self.dict['motor_right_warning'] is not 0:
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN    
            status_msg += ' Motorbox has warning!'
            
            
        if self.diagnostics.hwid == 'node_id: 1' and  self.dict['batterySOC'] < 20:
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            
            status_msg += ' Battery state of charge is lower than 20%!'
            
        if self.dict['state'] == DriveState.Error.name:    
            status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            status_msg += ' Motorbox is in state is {}'.format(self.dict['state'])
        
        
        stat.summary(status,status_msg)
        
        for x in self.dict:
            stat.add(x, self.dict[x])
        
        return stat
        
        
        
class RemoteControl:
    def __init__(self):
        
        self.diagnostics = diagnostic_updater.Updater()

        self.diagnostics.setHardwareID('Abitron NOVA-M Prop-2L')
        self.diagnostics.add("Remote Control Updater", self.generateDiagnosticMsg)
        self.diagnostics.period = 2

        self.dict = collections.OrderedDict()
        
        self.dict['A'] = SwitchState.Center.name
        self.dict['B'] = SwitchState.Down.name
        self.dict['C'] = SwitchState.Center.name
        self.dict['D'] = SwitchState.Center.name

        self.dict['analog1'] = 0.0
        self.dict['analog2'] = 0.0
        self.dict['analog3'] = 0.0

        self.dict['option1'] = BinaryButtonState.Inactive.name
        self.dict['option2'] = BinaryButtonState.Inactive.name
        self.dict['option3'] = BinaryButtonState.Inactive.name
        self.dict['option4'] = BinaryButtonState.Inactive.name

        
        self.dict['eStop'] = BinaryButtonState.Inactive.name
        self.dict['passiveStop'] = BinaryButtonState.Inactive.name
        self.dict['activeStop'] = BinaryButtonState.Inactive.name
        self.dict['release'] = BinaryButtonState.Inactive.name

        self.dict["connectionQuality"] =  "0%"
               
    def receiveAnalogData(self, data):
        self.dict['analog1'] = (data[0] - 127) / 127.0
        self.dict['analog2'] = (data[1] - 127) / 127.0
        self.dict['analog3'] = (data[2] / 255.0)
        
        
        self.dict['option1'] = BinaryButtonState(data[4] > 0xC0).name
        self.dict['option2'] = BinaryButtonState(data[4] < 0x70).name
        self.dict['option3'] = BinaryButtonState(data[3] > 0xC0).name
        self.dict['option4'] = BinaryButtonState(data[3] < 0x70).name
    
        
        self.diagnostics.update()

        
    def receiveBaseData(self, data):
        # todo cleanup
        dat1, dat2, dat3, dat4, res1, res2, count, error = struct.unpack("<8B", data)
        
        
        
        activeStop = error & 1
        passiveStop = ((error >> 1) & 1)
        rf0 = ((error >> 2) & 1)
        rf1 = ((error >> 3) & 1)
        if(rf0==0 and rf1==0):
            self.dict['connectionQuality'] = "0%"
        elif(rf0==1 and rf1==0):
            self.dict['connectionQuality']  = "<20%"
        elif(rf0==1 and rf1==1):
            self.dict['connectionQuality']  = "20%...60%"
        elif(rf0==0 and rf1==1):
            self.dict['connectionQuality']  = ">60%"
        
        
        joystickError, rf1, rf0, passiveStop, activeStop = bitstruct.unpack("u4u1u1u1u1", chr(error))
        
            
        b_up, a_down, a_up, release_switch, estop_switch = bitstruct.unpack("p1u1u1u1p2u1u1", chr(dat1))
        d_down, d_up, c_down, c_up = bitstruct.unpack("p3u1u1u1u1p1", chr(dat4))
        
        self.dict['eStop'] = BinaryButtonState(estop_switch).name
        self.dict['activeStop'] = BinaryButtonState(activeStop).name
        self.dict['passiveStop'] = BinaryButtonState(passiveStop).name
        self.dict['release'] = BinaryButtonState(release_switch).name
        
        if joystickError == 0:
            self.dict['joystickError'] = "no error"
        else:
            self.dict['joystickError'] = 'error joystick {}'.format(joystickError)
        
        if a_up:
            self.dict['A'] = SwitchState.Up.name

        elif a_down:
            self.dict['A'] = SwitchState.Down.name

        else:
            self.dict['A'] = SwitchState.Center.name


        if b_up:
            self.dict['B'] = SwitchState.Up.name
        else:
            self.dict['B'] = SwitchState.Down.name

        if c_up:
            self.dict['C'] = SwitchState.Up.name
        elif c_down:
            self.dict['C'] = SwitchState.Down.name
        else:
            self.dict['C'] = SwitchState.Center.name

        if d_up:
            self.dict['D'] = SwitchState.Up.name            
        elif d_down:
            self.dict['D'] = SwitchState.Down.name            
        else:
            self.dict['D'] = SwitchState.Center.name             
        
        self.diagnostics.update()
        
    
    def generateDiagnosticMsg(self, stat):
        
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Remote Control")
        
                
        for x in self.dict:
            stat.add(x, self.dict[x])
        
        return stat
        


class Diagnostics:
    
    def __init__(self):
        self.remoteControl = RemoteControl()
        
        self.motorbox1 = Motorbox('node_id: 1')
        self.motorbox2 = Motorbox('node_id: 2')
        
        can_interface = 'can0'
        self.bus = can.interface.Bus(can_interface, bustype='socketcan')
    
    def receive(self):
        while True:
            message = self.bus.recv(1.0)
            if message is None:
                print('Timeout occurred!')
            else:
                if message.arbitration_id == 0x1e4:
                    # Remote control analog data
                    self.remoteControl.receiveAnalogData(message.data)
                    
                elif message.arbitration_id == 0x2e4:
                    # Remote control base message
                    self.remoteControl.receiveBaseData(message.data)

                elif message.arbitration_id == 0x0DF:
                    self.motorbox1.receiveStateInfo_1(message.data)
                
                elif message.arbitration_id == 0x182:
                    self.motorbox2.receiveStateInfo_1(message.data)
                    
                elif message.arbitration_id == 0x0E1:
                    self.motorbox1.receiveStateInfo_2(message.data)
                
                elif message.arbitration_id == 0x382:
                    self.motorbox2.receiveStateInfo_2(message.data)
                    
                elif message.arbitration_id == 0xEB:
                    self.motorbox1.receiveBatteryInfo(message.data)
                    
                elif message.arbitration_id == 0x1F8:
                    pass
                
if __name__ == '__main__':
    rospy.init_node('can_diagnostic_node')
    diagnosticNode = Diagnostics()  
    diagnosticNode.receive()