import diagnostic_msgs
import diagnostic_updater
import rospy
import socket
import can
import struct
import collections
from enum import Enum
from std_msgs.msg import String

from innok_heros_driver.utils import check_soc_validity

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
    
class KeySwitchState(Enum):
    Autonomous = 0
    Manual = 1
    

    

class Motorbox:
    alarms = {}
    alarms[0] = ["None", ""]
    alarms[0x30] = ["Overload: ", "A load exceeding the rated torque was applied to the motor for 5 seconds or more."]
    alarms[0x28] = ["Sensor error: ",
                    "The motor sensor signal line experienced an open circuit during operation, or the motor signal connector came off."]
    alarms[0x42] = ["Initial sensor error: ", 
                    "The motor sensor signal line broke or motor signal connector came off before the main power supply was turned on."]
    alarms[0x22] = ["Overvoltage: ", 
                    ("<> The main power supply voltage exceeded the overvoltage detection level (approx. 72 VDC). " 
                    "<> Sudden starting/stopping of a large inertia load was performed.")]
    alarms[0x25] = ["Undervoltage: ", "The main power supply voltage dropped the undervoltage detection level (approx. 20 VDC)."]
    alarms[0x31] = ["Overspeed: ", "the rotation speed of the motor output shaft exceeded approx. 4800 r/min."]
    alarms[0x20] = ["Overcurrent: ", "Excessive current has flown through the driver due to ground fault, etc."]
    alarms[0x41] = ["EEPROM error: ", 
                    ("<> Stored data was damaged. "
                    "<> Data became no longer writable or readable.")]
    alarms[0x21] = ["Main circuit overheat: ", "The temperature inside the driver exceeded the main circuit overheat level."]
    alarms[0x6E] = ["External stop: ", "The EXT-ERROR input turned OFF."]
    alarms[0x46] = ["Initial operation error: ", "The main power supply was cycled when the FWD input or REV input was ON"]
    alarms[0x81] = ["Network bus error: ", "The bus of host network of the network converter turned off while the motor was operating"]
    alarms[0x83] = ["Communication switch setting error: ", "The communication function switch (SW2-No.4) was turned ON"]
    alarms[0x84] = ["RS-485 communication error: ", 
                    ("The number of consecutive RS-485 communication errors "
                    "reached the value set in the \"communication error alarm\" parameter")]
    alarms[0x85] = ["RS-485 communication timeout: ", 
                    ("The time set in the \"communication timeout\" parameter has elapsed, "
                    "and yet the communication could not be estalished with the host system")]
    alarms[0x8E] = ["Network converter error: ", "The network converter generated an alarm"]
    alarms[0x2D] = ["Main circuit output error: ", "The motor drive wire broke or motor drive connector came off"]

    warnings = {}
    warnings[0] = ["None", ""]
    warnings[0x21] = ["Main circuit overheat: ", "The temperature inside the driver exceeded the overheat warning level."]
    warnings[0x25] = ["Undervoltage: ", "The main power supply voltage dropped by approx. 10% or more from the rated voltage."]
    warnings[0x30] = ["Overload: ", "The load torque of the motor exceeded the overload warning level."]
    warnings[0x6C] = ["Operation error: ", "Operation error"]
    warnings[0x84] = ["RS-485 communication error: ", "A RS-485 communication error was detected."]



    def __init__(self, hardware_id):
        self.diagnostics = diagnostic_updater.Updater()
        self.diagnostics.setHardwareID(hardware_id)
        self.diagnostics.add("Battery", self.generateBatteryDiagnostics)
        self.diagnostics.add("Motorbox", self.generateMotorboxDiagnostics)

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
        
        self.bat_dict = collections.OrderedDict()
        self.bat_dict['batterySOC'] = 0
        self.bat_dict['batteryVoltage'] = 0.0
        
        
    def receiveStateInfo_1(self, data):
        # todo gscheite namen
        
        
        self.dict['state'] = DriveState(data[7]).name
        self.dict['error_flags'] = data[6]

        self.dict['motor_right_alarm'] = Motorbox.alarms[data[0]][0] + Motorbox.alarms[data[0]][1]
        self.dict['motor_right_warning'] = Motorbox.warnings[data[1]][0] + Motorbox.warnings[data[1]][1]
        self.dict['motor_right_error'] = data[4]
        
        self.dict['motor_left_alarm'] = Motorbox.alarms[data[2]][0] + Motorbox.alarms[data[2]][1]
        self.dict['motor_left_warning'] = Motorbox.warnings[data[3]][0] + Motorbox.warnings[data[3]][1]
        self.dict['motor_left_error'] = data[5]
        
        self.diagnostics.update()
        
    def receiveStateInfo_2(self, data):
        # TODO gscheite namen
        # TODO fix remote estop flag bug in firmware:
        
        self.dict['remoteEstopFlag'] = BinaryButtonState(data[0]).name
        self.dict['contactor'] = BinaryButtonState(data[1]).name
        self.dict['activeController'] = Controller(data[5]).name
        self.dict['controllerSelector'] = Controller(data[6]).name
        self.dict['driveMsgDelta'] = data[3]
        self.dict['validMsgCount'] = data[4]
        self.dict['looptime'] = data[7]
        
        self.diagnostics.update()

                       

    def receiveBatteryInfo(self, data):
        
        soc, voltage = struct.unpack('<BH', data)
        
        self.bat_dict['batterySOC'] = soc
        self.bat_dict['batteryVoltage'] = (voltage / 100.0)
        self.diagnostics.update()
        

            
    
    def generateMotorboxDiagnostics(self, stat):
        
        status = diagnostic_msgs.msg.DiagnosticStatus.OK

        status_msg = ''
        
        if self.dict['state'] == DriveState.EmergencyStop.name or self.dict['state'] == DriveState.EmergencyStopRemote.name:
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            
            status_msg = 'state is {}!'.format(self.dict['state'])

        elif self.dict['state'] == DriveState.Error.name :
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            status_msg = 'state is {}'.format(self.dict['state'])

        elif self.dict['state'] == DriveState.Error_Reset.name:
            status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            status_msg = 'state is {}'.format(self.dict['state'])
        
        elif self.dict['motor_left_warning'] != 'None' or self.dict['motor_right_warning'] != 'None':
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN    
            status_msg = 'Warning: ' 
            if self.dict['motor_left_warning'] != 'None':
                status_msg += 'left: ' + self.dict['motor_left_warning'] + " "
            if self.dict['motor_right_warning'] != 'None':
                status_msg += 'right: ' + self.dict['motor_left_warning']
        
        if status == diagnostic_msgs.msg.DiagnosticStatus.OK:
            status_msg = "OK"
        
        stat.summary(status,status_msg)
        
        for x in self.dict:
            stat.add(x, self.dict[x])
        
       
        return stat    
    
    def generateBatteryDiagnostics(self, stat):
        voltage = self.bat_dict['batteryVoltage']
        soc = self.bat_dict['batterySOC']
        if check_soc_validity(voltage, soc) == False:
            status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            status_msg = "Battery state of charge is not valid!"
        elif soc < 20:
            status = diagnostic_msgs.msg.DiagnosticStatus.WARN
            status_msg = "Battery state of charge is lower than 20%"
            
        elif soc < 5:
            status = diagnostic_msgs.msg.DiagnosticStatus.ERROR
            status_msg = "Battery state of charge is lower than 5%"
        else:
            status = diagnostic_msgs.msg.DiagnosticStatus.OK
            status_msg = "Battery OK"

        stat.summary(status, status_msg)
                
        for x in self.bat_dict:
            stat.add(x, self.bat_dict[x])
        
        return stat
            
        
class RemoteControl:
    def __init__(self):
        
        self.diagnostics = diagnostic_updater.Updater()

        self.diagnostics.setHardwareID('Abitron NOVA-M Prop-2L')
        self.diagnostics.add("Remote Control", self.generateDiagnosticMsg)

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
    
        self.joystickError = 0
        self.diagnostics.update()

        
    def receiveBaseData(self, data):
        # todo cleanup
        dat1, dat2, dat3, dat4, res1, res2, count, error = struct.unpack("<8B", data)
        
        
        
        activeStop = error & 1
        passiveStop = (error >> 1) & 1
        rf0 = (error >> 2) & 1
        rf1 = (error >> 3) & 1
        self.joystickError = ((error >> 4) & 0b1111)
        
        if(rf0==0 and rf1==0):
            self.dict['connectionQuality'] = "0%"
        elif(rf0==1 and rf1==0):
            self.dict['connectionQuality']  = "<20%"
        elif(rf0==1 and rf1==1):
            self.dict['connectionQuality']  = "20%...60%"
        elif(rf0==0 and rf1==1):
            self.dict['connectionQuality']  = ">60%"
        
        
        
        estop_switch = (dat1 & 1)
        release_switch = (dat1 << 1) & 1
        a_up = (dat1 << 4) & 1
        a_down = (dat1 << 5) & 1
        b_up = (dat1 << 7) & 1
       
        c_up = (dat2 << 1) & 1
        c_down = (dat2 << 2) & 1
        d_up = (dat2 << 3) & 1
        d_down = (dat2 << 4) & 1
       
        
        self.dict['eStop'] = BinaryButtonState(estop_switch).name
        self.dict['activeStop'] = BinaryButtonState(activeStop).name
        self.dict['passiveStop'] = BinaryButtonState(passiveStop).name
        self.dict['release'] = BinaryButtonState(release_switch).name
        
        if self.joystickError == 0:
            self.dict['joysticks'] = "no error"
        else:
            self.dict['joysticks'] = 'error joystick {}'.format(self.joystickError)
        
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
        
        if self.joystickError == 0:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Remote Control OK")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Remote Control joystick error: {}".format(self.dict['joystick']))
            
                
        for x in self.dict:
            stat.add(x, self.dict[x])
        
        return stat
        
class KeySwitch:
    def __init__(self):
        self.key_switch_pub = rospy.Publisher('key_switch', String, queue_size=10)
        self.diagnostics = diagnostic_updater.Updater()
        self.diagnostics.setHardwareID('none')
        self.diagnostics.add("KeySwitch", self.generateDiagnosticMsg)
        self.dict = collections.OrderedDict()
        self.dict['state'] = KeySwitchState.Manual.name
     
    def receiveKeySwitchInfo(self, data):
        if data[0] == 1 and data[1] == 1:
            self.dict['state'] = KeySwitchState.Manual.name
        else: 
            self.dict['state'] = KeySwitchState.Autonomous.name
        
        self.diagnostics.update()
        self.key_switch_pub.publish(self.dict['state'])

    def generateDiagnosticMsg(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Keyswitch OK")        
        for x in self.dict:
            stat.add(x, self.dict[x])
        
        return stat

class HerosDiagnostics:    
    def __init__(self):
        rospy.init_node('heros_diagnostics')
        self.remoteControl = RemoteControl()
        self.keyswitch = KeySwitch()
        self.motorbox1 = Motorbox('node_id: 1')
        self.motorbox2 = Motorbox('node_id: 2')
        
        can_interface = rospy.get_param('can_interface', 'can0')
        self.bus = can.interface.Bus(can_interface, bustype='socketcan')
    
    def run(self):
        while not rospy.is_shutdown():
            message = self.bus.recv(1.0)
            if message is None:
                rospy.logwarn('No can message received for 1s!')
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
                    if not self.motorbox2.dict['state'] == DriveState.NotDefined.name:
                        self.motorbox2.receiveBatteryInfo(message.data)
                    
                elif message.arbitration_id == 0x1F8:
                    self.keyswitch.receiveKeySwitchInfo(message.data)
        self.bus.shutdown()

