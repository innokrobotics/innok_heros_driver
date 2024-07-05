#!/usr/bin/env python
import rospy
import can
import struct
import diagnostic_msgs
from diagnostic_msgs.msg import DiagnosticStatus

import diagnostic_updater
from std_msgs.msg import Bool
from sensor_msgs.msg import BatteryState
from innok_heros_driver.utils import check_soc_validity

class CANBmsPublisher:
    def __init__(self):
        rospy.init_node('can_bms_publisher')
        can_interface = rospy.get_param('can_interface', 'can0')
        self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')

        self.can_messages = {
            0xEB: {
                'name': 'HerosROCTR_Voltage',
                'signals': [
                    {
                        'name': 'ROCTR_SOC',
                        'start_bit': 0,
                        'length': 8,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'Percent',
                        'publisher': None, #rospy.Publisher('HerosROCTR_Voltage/ROCTR_SOC/Percent', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'ROCTR_Voltage',
                        'start_bit': 8,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('HerosROCTR_Voltage/ROCTR_Voltage/mV', Float32, queue_size=10),
                        'value': None,
                    }
                ]
            },
            0x8020f17: {
                'name': 'XLAkku_Fuelgauge',
                'signals': [
                    {
                        'name': 'Voltage',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Fuelgauge/Voltage/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Current',
                        'start_bit': 16,
                        'length': 16,
                        'endian': 'little',
                        'signed': True,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'A',
                        'publisher': None, #rospy.Publisher('XLAkku_Fuelgauge/Current/A', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'RemCapacity',
                        'start_bit': 32,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'A',
                        'publisher': None,
                        'value': None,
                    },
                                        {
                        'name': 'RemCapacityPerc',
                        'start_bit': 48,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '%',
                        'publisher': None, 
                        'value': None,
                    },
                ]
            },
            0x08021F17: {
                'name': 'XLAkku_State',
                'signals': [
                    {
                        'name': 'Temperature',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': True,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'Celsius',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Temperature/Celsius', Int32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_CHG',
                        'start_bit': 16,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_CHG', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_DSG',
                        'start_bit': 17,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_DSG', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_XCHG',
                        'start_bit': 18,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_XCHG', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_XDSG',
                        'start_bit': 19,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_XDSG', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'SOH',
                        'start_bit': 56,
                        'length': 8,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'Percent',
                        'publisher': None, #rospy.Publisher('XLAkku_State/SOH/Percent', UInt8, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'ErrorStatusFlagsCUV',
                        'start_bit': 32,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/ErrorStatusFlagsCUV', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_HCFET',
                        'start_bit': 20,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_HCFET', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Transistor_state_FET_Req',
                        'start_bit': 21,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/Transistor_state_FET_Req', Bool, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'FAN_Status',
                        'start_bit': 23,
                        'length': 1,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': '',
                        'publisher': None, #rospy.Publisher('XLAkku_State/FAN_Status', Bool, queue_size=10),
                        'value': None,
                    },
                    # ... more signals go here ...
                ]
            },
            0x08022F17: {
                'name': 'XLAkku_Cellvoltage_A',
                'signals': [
                    {
                        'name': 'Cell_01',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_A/Cell_01/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_02',
                        'start_bit': 16,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_A/Cell_02/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_03',
                        'start_bit': 32,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_A/Cell_03/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_04',
                        'start_bit': 48,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_A/Cell_04/mV', Float32, queue_size=10),
                        'value': None,
                    }
                ]
            },
            0x08023F17: {
                'name': 'XLAkku_Cellvoltage_B',
                'signals': [
                    {
                        'name': 'Cell_05',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_B/Cell_05/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_06',
                        'start_bit': 16,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_B/Cell_06/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_07',
                        'start_bit': 32,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_B/Cell_07/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_08',
                        'start_bit': 48,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_B/Cell_08/mV', Float32, queue_size=10),
                        'value': None,
                    }
                ]
            },
            0x08024F17: {
                'name': 'XLAkku_Cellvoltage_C',
                'signals': [
                    {
                        'name': 'Cell_09',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_C/Cell_09/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_10',
                        'start_bit': 16,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_C/Cell_10/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_11',
                        'start_bit': 32,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_C/Cell_11/mV', Float32, queue_size=10),
                        'value': None,
                    },
                    {
                        'name': 'Cell_12',
                        'start_bit': 48,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_C/Cell_12/mV', Float32, queue_size=10),
                        'value': None,
                    }
                ]
            },
            0x08025F17: {
                'name': 'XLAkku_Cellvoltage_D',
                'signals': [
                    {
                        'name': 'Cell_13',
                        'start_bit': 0,
                        'length': 16,
                        'endian': 'little',
                        'signed': False,
                        'scale': 1,
                        'offset': 0,
                        'unit': 'mV',
                        'publisher': None, #rospy.Publisher('XLAkku_Cellvoltage_D/Cell_13/mV', Float32, queue_size=10),
                        'value': None,
                    },
                ]
            },
            0x10001007: {
                'name': 'XLAkku_StayAlive',
                # no signal, nothing will be sent
                'signals': []
            },
        }

        self.can_ids = list(self.can_messages.keys())

        self.state_pub = rospy.Publisher('battery_state', BatteryState, queue_size=10)
        self.battery_state_msg = BatteryState()
        self.battery_state_msg.cell_voltage = [0.0] * 13
        self.battery_state_msg.location = 'BMS'
        self.battery_state_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.diagnostics = diagnostic_updater.Updater()
        self.diagnostics.setHardwareID('CAN BMS')
        self.diagnostics.add("XLAkku", self.generateBatteryDiagnostics)

        self.ros_messages = {
            'XLAkku_Power/XLAkku_on_off': {
                'can_id': 0x10002f07,
                'subscriber': rospy.Subscriber('XLAkku_Power/XLAkku_on_off', Bool, self.ros_msg_callback, callback_args=0x10002f07),
            },
            'XLAkku_CANBroadcast/XLAkku_broadcast_on_off': {
                'can_id': 0x10003f07,
                'subscriber': rospy.Subscriber('XLAkku_CANBroadcast/XLAkku_broadcast_on_off', Bool, self.ros_msg_callback, callback_args=0x10003f07),
            }
        }

        rospy.Timer(rospy.Duration(0.001), self.receive_callback)


    def receive_callback(self, event):
        message = self.bus.recv(1.0)
        # If a message was received
        if message:
            # Check if the CAN ID of this message is one of the ones we're interested in
            if message.arbitration_id in self.can_ids:
                # For each signal in this CAN message
                for signal in self.can_messages[message.arbitration_id]['signals']:
                    # Extract and decode the signal value
                    raw_value = self.extract_signal(
                        message.data, 
                        signal['start_bit'], 
                        signal['length'], 
                        signal['endian'], 
                        signal['signed']
                    )
                    signal['value']  = raw_value * signal['scale'] + signal['offset']

                    if signal['publisher'] is not None:
                        # Publish the signal value
                        signal['publisher'].publish(signal['value'])
            
                    if self.can_messages[message.arbitration_id]['name'] == 'XLAkku_Fuelgauge':
                        if signal['name'] == 'Voltage':
                            self.battery_state_msg.voltage = signal['value'] / 100.0
                        elif signal['name'] == 'Current':
                            self.battery_state_msg.current = signal['value']
                        elif signal['name'] == 'RemCapacity':
                            self.battery_state_msg.capacity = signal['value']  
                        elif signal['name'] == 'RemCapacityPerc':
                            self.battery_state_msg.percentage = signal['value']  

                    elif self.can_messages[message.arbitration_id]['name'] == 'XLAkku_State':
                        if signal['name'] == 'Temperature':
                            self.battery_state_msg.temperature = signal['value']
                    elif 'Cellvoltage' in self.can_messages[message.arbitration_id]['name']:
                        # extract cell number and set value in corresponding field of 
                        self.battery_state_msg.cell_voltage[int(signal['name'].strip('Cell_')) - 1] = signal['value']

                if self.can_messages[message.arbitration_id]['name'] == 'XLAkku_Fuelgauge':
                    self.publish_battery_state()
                self.diagnostics.update()


    def extract_signal(self, data, start_bit, length, endian, signed):     
        bit_offset = start_bit % 8
        byte_offset = start_bit // 8
        num_bytes = (length + bit_offset + 7) // 8

        permissible_lengths = [1, 8, 16, 32, 64]
        if length not in permissible_lengths:
            raise ValueError(f'Signal length must be one of {permissible_lengths}, not {length} bits')
        
        raw_data = data[byte_offset : byte_offset+num_bytes]

        if length == 1:
            value = (raw_data[0] >> bit_offset) & 0b1
        else:
            if signed:
                format_str = {1: 'b', 2: 'h', 4: 'i', 8: 'q'}[num_bytes]
            else:
                format_str = {1: 'B', 2: 'H', 4: 'I', 8: 'Q'}[num_bytes]
            if endian == 'big':
                raw_data = raw_data[::-1]
            value = struct.unpack(format_str, raw_data)[0]

        return value

    def publish_battery_state(self):
        self.battery_state_msg.header.seq += 1
        self.battery_state_msg.header.stamp = rospy.Time.now()
        self.state_pub.publish(self.battery_state_msg)

    def ros_msg_callback(self, msg, can_id):
        data = [1 if msg.data else 0]

        # Create a CAN message with the received data and send it
        can_msg = can.Message(arbitration_id=can_id,
                              data=data, is_extended_id=True)
        self.bus.send(can_msg)

    def generateBatteryDiagnostics(self, stat):
        voltage = self.battery_state_msg.voltage
        soc = self.battery_state_msg.percentage     
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

        for can_id in self.can_messages:
            for signal in self.can_messages[can_id]['signals']:
                stat.add(signal['name'], signal['value'])
        return stat
    
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            print("Shutting down...")
            self.bus.shutdown()