innok_heros_driver
==================

ROS driver for the Innok Heros robot plattform.

Start with

    roslaunch innok_heros_driver innok_heros_driver.launch bms_available:=true

If the robot is equipped with the XLAkku that includes the CAN battery management system (BMS), set the argument `bms_available`to `true`


This command will start the following nodes:

1. `innok_heros_can_driver`: Driver that communicate with the robot platform via CAN bus.
2. `can_bms_node`: Recieves data from the BMS via CAN and publishes it on ROS topics. Only launched if BMS is available.
3. `battery_watchdog`: This node can shut off the robot before its battery is completely drained.


Copyright (c) 2014-2024 Innok Robotics GmbH