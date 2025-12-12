#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################


import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
# our stuff (really just want Destination()
import create_points as creep

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self, spot, count):
        super().__init__(f"minimal_publisher_{count}")
        
        prefix = f"/px4_{count}"

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscriptions -------------------------------------------------
        self.status_sub = self.create_subscription(
            VehicleStatus,
            prefix + "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile,
        )

        # Velocity teleop removed for position-based control
        # self.offboard_velocity_sub = self.create_subscription(
        #     Twist,
        #     '/offboard_velocity_cmd',
        #     self.offboard_velocity_callback,
        #     qos_profile)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            prefix+ "/fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile,
        )

        self.my_bool_sub = self.create_subscription(
            Bool,
            prefix+"/arm_message",
            self.arm_message_callback,
            qos_profile,
        )
        
        # listned to where to go
        self.pos_sub = self.create_subscription(
            TrajectorySetpoint,
            prefix+"/position_command",
            self.pos_callback,
            qos_profile=qos_profile,
        )

        # --- Publishers ----------------------------------------------------
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            prefix+"/fmu/in/offboard_control_mode",
            qos_profile,
        )

        # Velocity publisher not used in position control version
        # self.publisher_velocity = self.create_publisher(
        #     Twist,
        #     '/fmu/in/setpoint_velocity/cmd_vel_unstamped',
        #     qos_profile)

        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint,
            prefix+"/fmu/in/trajectory_setpoint",
            qos_profile,
        )

        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand,
            prefix+"/fmu/in/vehicle_command",
            10,
        )

        # --- Timers --------------------------------------------------------
        # Arm / mode state machine timer
        arm_timer_period = 0.1  # seconds (> 2 Hz)
        self.arm_timer_ = self.create_timer(
            arm_timer_period,
            self.arm_timer_callback,
        )

        # Offboard command loop timer
        timer_period = 0.02  # seconds (50 Hz)
        self.timer = self.create_timer(
            timer_period,
            self.cmdloop_callback,
        )

        # --- State variables ----------------------------------------------
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED

        # In position-control version we do not use velocity commands
        # self.velocity = Vector3()

        self.yaw = 0.0          # commanded yaw
        self.trueYaw = 0.0      # measured yaw from VehicleAttitude

        self.offboardMode = False
        self.flightCheck = False
        self.myCnt = 0
        self.arm_message = False
        self.failsafe = False

        self.current_state = "IDLE"
        self.last_state = self.current_state

        # Desired position setpoint: [x, y, z, yaw]
        # Initialize to some safe default (e.g., hover at (0,0,5m,0rad))
        self.desired_position = np.array([spot.x, spot.y, -5.0, 0.0], dtype=float)

    # ----------------------------------------------------------------------
    # Subscriptions callbacks
    # ----------------------------------------------------------------------
    

    def pos_callback(self, msg):
        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]
        yaw = self.desired_position[3]

        if z == 0.0:
            self.get_logger().warn(f"Ignoring position_command with z=0: {msg.position}")
            return
        
        self.goto_position(x, y, z, yaw)
        
        self.arm_message = True
        self.get_logger().info(f"pos_cmd: {msg.position}")



    def arm_message_callback(self, msg: Bool):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    # State machine: arms, takes off, switches to offboard
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if self.flightCheck and self.arm_message:
                    self.current_state = "ARMING"
                    self.get_logger().info("Arming")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Arming, Flight Check Failed")
                elif (
                    self.arm_state == VehicleStatus.ARMING_STATE_ARMED
                    and self.myCnt > 10
                ):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info("Arming, Takeoff")
                self.arm()  # send arm command

            case "TAKEOFF":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Takeoff, Flight Check Failed")
                elif (
                    self.nav_state
                    == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
                ):
                    self.current_state = "LOITER"
                    self.get_logger().info("Takeoff, Loiter")
                self.arm()       # keep sending arm command
                self.take_off()  # send takeoff command

            # Waits while taking off. Once LOITER, switch to OFFBOARD.
            case "LOITER":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Loiter, Flight Check Failed")
                elif (
                    self.nav_state
                    == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
                ):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Loiter, Offboard")
                self.arm()

            case "OFFBOARD":
                if (
                    (not self.flightCheck)
                    or self.arm_state != VehicleStatus.ARMING_STATE_ARMED
                    or self.failsafe
                ):
                    self.current_state = "IDLE"
                    self.get_logger().info("Offboard, Flight Check Failed")
                else:
                    self.state_offboard()

        # If disarmed, require a new arm trigger
        if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.arm_message = False

        # Log state transitions
        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.myCnt += 1

    def state_offboard(self):
        self.myCnt = 0
        # VEHICLE_CMD_DO_SET_MODE: param1 = base mode, param2 = custom mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
        )
        self.offboardMode = True

    # Arms the vehicle
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
        )
        self.get_logger().info("Arm command sent")

    # Takes off to a fixed altitude (meters)
    def take_off(self):
        # param1 = minimum pitch, param7 = altitude (m)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=1.0,
            param7=5.0,
        )
        self.get_logger().info("Takeoff command sent")

    # Publishes a VehicleCommand to /fmu/in/vehicle_command
    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param7: float = 0.0,
    ):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7  # altitude for takeoff command
        msg.command = command  # command ID
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # microseconds
        self.vehicle_command_publisher_.publish(msg)

    # Receives and sets vehicle status values
    def vehicle_status_callback(self, msg: VehicleStatus):
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(
                f"FlightCheck: {msg.pre_flight_checks_pass}"
            )

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    # Attitude callback: extract yaw from quaternion
    def attitude_callback(self, msg: VehicleAttitude):
        orientation_q = msg.q

        # trueYaw is the drone's current yaw value (FLU, radians)
        self.trueYaw = -(
            np.arctan2(
                2.0
                * (
                    orientation_q[3] * orientation_q[0]
                    + orientation_q[1] * orientation_q[2]
                ),
                1.0
                - 2.0
                * (
                    orientation_q[0] * orientation_q[0]
                    + orientation_q[1] * orientation_q[1]
                ),
            )
        )

    # ----------------------------------------------------------------------
    # Offboard command loop - position control version
    # ----------------------------------------------------------------------
    def cmdloop_callback(self):
        # Only publish if the vehicle is in a sane state
        # if (not self.flightCheck or
        #     self.arm_state != VehicleStatus.ARMING_STATE_ARMED or
        #     self.failsafe):
        #     return

        # 1) Publish OffboardControlMode indicating position control
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        self.publisher_offboard_mode.publish(offboard_msg)

        # 2) Publish a TrajectorySetpoint with desired position + yaw
        traj = TrajectorySetpoint()
        traj.timestamp = int(Clock().now().nanoseconds / 1000)

        # Desired position [x, y, z]
        traj.position[0] = float(self.desired_position[0])
        traj.position[1] = float(self.desired_position[1])
        traj.position[2] = float(self.desired_position[2])

        # We are not commanding velocity/accel -> NaN
        traj.velocity[0] = float("nan")
        traj.velocity[1] = float("nan")
        traj.velocity[2] = float("nan")

        traj.acceleration[0] = float("nan")
        traj.acceleration[1] = float("nan")
        traj.acceleration[2] = float("nan")

        # Yaw command from desired_position[3], no yaw rate
        traj.yaw = float(self.desired_position[3])
        traj.yawspeed = 0.0

        self.publisher_trajectory.publish(traj)


    # API to set a new position setpoint (x, y, z, yaw)
    def goto_position(self, x: float, y: float, z: float, yaw: float):
        """
        Set the desired position (x, y, z) in NED frame and yaw (rad).

        This just updates the internal setpoint; the cmdloop_callback
        actually publishes it periodically while offboardMode is True.
        """
        #fix for falling down need to convert to negative
        self.desired_position = np.array([x, y, -z, yaw], dtype=float)
        self.get_logger().info(
            f"New position setpoint: x={x}, y={y}, z={-z}, yaw={yaw}"
        )


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
