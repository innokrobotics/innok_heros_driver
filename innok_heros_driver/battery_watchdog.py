#!/usr/bin/env python

import os
import subprocess

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.parameter import Parameter

def one_shot_callback(timer, callback):
    def wrapper():
        callback()
        timer.cancel()
    return wrapper

class BatteryWatchdog(Node):
    def __init__(self):
        super().__init__("battery_watchdog")


        self.initial_percentage = None
        self.shutdown_enabled = True  # Flag to enable/disable the shutdown feature

        
        # Declare parameters
        self.declare_parameter("percentage_threshold", 5.0)
        self.declare_parameter("enable_threshold", 10.0)
        self.declare_parameter("shutdown_behaviour", "battery")
        self.declare_parameter("shutdown_timeout", 0.0)

        # Load parameters
        self.update_parameters()

        self.get_logger().info(f"percentage_threshold: {self.percentage_threshold}")
        self.get_logger().info(f"enable_threshold: {self.enable_threshold}")
        self.get_logger().info(f"shutdown_behaviour: {self.shutdown_behaviour}")
        self.get_logger().info(f"shutdown_timeout: {self.shutdown_timeout}")

        # Create the service for enabling/disabling the shutdown feature
        self.create_service(SetBool, "~/enable_shutdown", self.enable_shutdown_callback)

        # Subscribe to the /battery_state topic
        self.create_subscription(BatteryState, "/battery_state", self.battery_state_callback, qos_profile=10)

        # Create publisher to turn off battery
        self.battery_on_off_pub = self.create_publisher(Bool, "XLAkku_Power/XLAkku_on_off", qos_profile=1)

        # Shutdown timer instance
        self.shutdown_timer = None
        
        self.get_logger().info("Battery Watchdog Node initialized.")

        

    def update_parameters(self):
        # Get the threshold value from the parameter server
        percentage_threshold = self.get_parameter("percentage_threshold").value
        enable_threshold = self.get_parameter("enable_threshold").value
        if percentage_threshold > enable_threshold:
            self.get_logger().warn(
                f"percentage_threshold is greater than enable_threshold! Using percentage_threshold as enable_threshold ({enable_threshold})."
            )
            percentage_threshold = enable_threshold
            self.set_parameters([Parameter("percentage_threshold", Parameter.Type.DOUBLE, percentage_threshold)])
        self.percentage_threshold = percentage_threshold
        self.enable_threshold = enable_threshold

        self.shutdown_behaviour = self.get_parameter("shutdown_behaviour").value
        self.shutdown_timeout = self.get_parameter("shutdown_timeout", ).value
        if self.shutdown_behaviour != "battery" and self.shutdown_behaviour != "pc":
            self.get_logger().warn(
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
            self.get_logger().info(
                "Battery percentage above threshold again: Enabling Shutdown feature."
            )
            self.enable_shutdown_feature(True)
        if not self.shutdown_enabled:
            return
        current_percentage = msg.percentage
        if self.initial_percentage is None:
            # Message is called for the first time
            self.initial_percentage = current_percentage
            self.get_logger().info(f"Initial percentage: {self.initial_percentage}")
            if self.initial_percentage < self.enable_threshold:
                self.get_logger().warn(
                    f"Initial percentage is below the threshold ({self.initial_percentage} < {self.enable_threshold}). Shutdown feature will be disabled."
                )
                self.enable_shutdown_feature(False)
        elif current_percentage < self.percentage_threshold:
            self.get_logger().fatal(
                f"Battery percentage is below the threshold ({current_percentage} < {self.percentage_threshold}). Initiating system shutdown..."
            )
            self.initiate_system_shutdown()

    def enable_shutdown_feature(self, enable: bool) -> str:
        message = "Shutdown feature is now {}".format(
            "enabled" if enable else "disabled"
        )
        if self.shutdown_enabled != enable:
            self.shutdown_enabled = enable
            self.get_logger().info(message)
        if not enable and self.shutdown_timer is not None:
            self.shutdown_timer.shutdown()
            self.shutdown_timer = None
            self.get_logger().info(
                "System shutdown has been cancelled by disabling the shutdown feature."
            )
        return message

    def enable_shutdown_callback(self, request: SetBool.Request, response: SetBool.Response):
        response.success = True
        response.message = self.enable_shutdown_feature(request.data)
        return response

    def initiate_system_shutdown(self):
        self.shutdown_timer = self.create_timer(
            self.shutdown_timeout + 0.001, None
        )
        if self.shutdown_behaviour == "pc":
            shutdown_callback = self.shutdown_pc
        elif self.shutdown_behaviour == "battery":
            shutdown_callback = self.shutdown_battery        
        self.shutdown_timer.callback = one_shot_callback(self.shutdown_timer, shutdown_callback)
        self.get_logger().warn(
            f"System will shutdown in {self.shutdown_timeout} seconds using the behaviour '{self.shutdown_behaviour}'!"
        )
        self.get_logger().info(
            "The Shutdown can be cancelled by disabling the shutdown feature."
        )


    def shutdown_pc(self):
        shutdown_cmd = "sudo shutdown -h now"
        self.get_logger().warn("Powering down pc now: '{shutdown_cmd}'")
        result = subprocess.run(
            [shutdown_cmd], shell=True, capture_output=True, text=True
        )
        self.get_logger().info(result)

    def shutdown_battery(self):
        self.get_logger().warn("Switching off the battery now")
        self.battery_on_off_pub.publish(Bool(data=False))

    def run(self):
        rclpy.spin(self)
