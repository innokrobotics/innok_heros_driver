#!/usr/bin/env python

import os
import subprocess

import rospy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse


class BatteryWatchdog:
    def __init__(self):
        rospy.init_node("battery_watchdog")

        # Get the threshold value from the parameter server
        self.percentage_threshold = 5.0
        self.enable_threshold = 10.0
        self.shutdown_behaviour = "battery"
        self.shutdown_timeout = 0.0

        self.update_parameters()

        self.initial_percentage = None
        self.shutdown_enabled = True  # Flag to enable/disable the shutdown feature

        rospy.loginfo("Battery Watchdog Node initialized.")
        rospy.loginfo(f"percentage_threshold: {self.percentage_threshold}")
        rospy.loginfo(f"enable_threshold: {self.enable_threshold}")
        rospy.loginfo(f"shutdown_behaviour: {self.shutdown_behaviour}")
        rospy.loginfo(f"shutdown_timeout: {self.shutdown_timeout}")

        # Create the service for enabling/disabling the shutdown feature
        rospy.Service("~enable_shutdown", SetBool, self.enable_shutdown_callback)

        # Subscribe to the /battery_state topic
        rospy.Subscriber("/battery_state", BatteryState, self.battery_state_callback)

        # Create publisher to turn off battery
        self.battery_on_off_pub = rospy.Publisher(
            "XLAkku_Power/XLAkku_on_off", Bool, queue_size=10
        )

        # Shutdown timer instance
        self.shutdown_timer = None

    def update_parameters(self):
        # Get the threshold value from the parameter server
        percentage_threshold = rospy.get_param("~percentage_threshold", 5.0)
        enable_threshold = rospy.get_param("~enable_threshold", 10.0)
        if percentage_threshold > enable_threshold:
            rospy.logwarn(
                f"percentage_threshold is greater than enable_threshold! Using percentage_threshold as enable_threshold ({enable_threshold})."
            )
            percentage_threshold = enable_threshold
            rospy.set_param("~percentage_threshold", percentage_threshold)

        self.percentage_threshold = percentage_threshold
        self.enable_threshold = enable_threshold

        self.shutdown_behaviour = rospy.get_param("~shutdown_behaviour", "battery")
        self.shutdown_timeout = rospy.get_param("~shutdown_timeout", 0)
        if self.shutdown_behaviour != "battery" and self.shutdown_behaviour != "pc":
            rospy.logwarn(
                "Invalid shutdown behavior! Using default behaviour 'battery'."
            )
            self.shutdown_behaviour = "battery"

    def battery_state_callback(self, msg):
        self.update_parameters()
        if (
            not self.shutdown_enabled
            and self.initial_percentage
            and msg.percentage > self.initial_percentage
            and msg.percentage > self.enable_threshold
        ):
            rospy.loginfo(
                "Battery percentage above threshold again: Enabling Shutdown feature."
            )
            self.enable_shutdown_feature(True)
        if not self.shutdown_enabled:
            return
        current_percentage = msg.percentage
        if self.initial_percentage is None:
            # Message is called for the first time
            self.initial_percentage = current_percentage
            rospy.loginfo(f"Initial percentage: {self.initial_percentage}")
            if self.initial_percentage < self.enable_threshold:
                rospy.logwarn(
                    f"Initial percentage is below the threshold ({self.initial_percentage} < {self.enable_threshold}). Shutdown feature will be disabled."
                )
                self.enable_shutdown_feature(False)
        elif current_percentage < self.percentage_threshold:
            rospy.logfatal(
                f"Battery percentage is below the threshold ({current_percentage} < {self.percentage_threshold}). Initiating system shutdown..."
            )
            self.initiate_system_shutdown()

    def enable_shutdown_feature(self, enable: bool) -> str:
        message = "Shutdown feature is now {}".format(
            "enabled" if enable else "disabled"
        )
        if self.shutdown_enabled != enable:
            self.shutdown_enabled = enable
            rospy.loginfo(message)
        if not enable and self.shutdown_timer is not None:
            self.shutdown_timer.shutdown()
            self.shutdown_timer = None
            rospy.loginfo(
                "System shutdown has been cancelled by disabling the shutdown feature."
            )
        return message

    def enable_shutdown_callback(self, req):
        response = SetBoolResponse()
        response.success = True
        response.message = self.enable_shutdown_feature(req.data)
        return response

    def initiate_system_shutdown(self):
        if self.shutdown_behaviour == "pc":
            shutdown_callback = self.shutdown_pc
        elif self.shutdown_behaviour == "battery":
            shutdown_callback = self.shutdown_battery
        rospy.logwarn(
            f"System will shutdown in {self.shutdown_timeout} seconds using the behaviour '{self.shutdown_behaviour}'!"
        )
        rospy.loginfo(
            "The Shutdown can be cancelled by disabling the shutdown feature."
        )
        self.shutdown_timer = rospy.Timer(
            rospy.Duration(self.shutdown_timeout + 0.001), shutdown_callback
        )

    def shutdown_pc(self, event):
        shutdown_cmd = "sudo shutdown -h now"
        rospy.logwarn("Powering down pc now: '{shutdown_cmd}'")
        result = subprocess.run(
            [shutdown_cmd], shell=True, capture_output=True, text=True
        )
        rospy.loginfo(result)

    def shutdown_battery(self, event):
        rospy.logwarn("Switching off the battery now")
        self.battery_on_off_pub.publish(False)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

