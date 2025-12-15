# #!/usr/bin/env python
# ############################################################################
# #
# #   Copyright (C) 2022 PX4 Development Team. All rights reserved.
# #
# # Redistribution and use in source and binary forms, with or without
# # modification, are permitted provided that the following conditions
# # are met:
# #
# # 1. Redistributions of source code must retain the above copyright
# #    notice, this list of conditions and the following disclaimer.
# # 2. Redistributions in binary form must reproduce the above copyright
# #    notice, this list of conditions and the following disclaimer in
# #    the documentation and/or other materials provided with the
# #    distribution.
# # 3. Neither the name PX4 nor the names of its contributors may be
# #    used to endorse or promote products derived from this software
# #    without specific prior written permission.
# #
# # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# # OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# # POSSIBILITY OF SUCH DAMAGE.
# #
# ############################################################################


# import rclpy
# from rclpy.node import Node
# import numpy as np
# from rclpy.clock import Clock
# from rclpy.qos import (
#     QoSProfile,
#     QoSReliabilityPolicy,
#     QoSHistoryPolicy,
#     QoSDurabilityPolicy,
# )
# # our stuff (really just want Destination()
# import create_points as creep

# from px4_msgs.msg import OffboardControlMode
# from px4_msgs.msg import TrajectorySetpoint
# from px4_msgs.msg import VehicleStatus
# from px4_msgs.msg import VehicleAttitude
# from px4_msgs.msg import VehicleCommand
# from px4_msgs.msg import VehicleLocalPosition
# from std_msgs.msg import Bool
# from geometry_msgs.msg import Twist, Vector3


# class OffboardControl(Node):

#     def __init__(self, spot, count):
#         super().__init__(f"minimal_publisher_{count}")
        
#         prefix = f"/px4_{count}"


#         self.offboard_behavior = None
#         self.hover_x = 0.0
#         self.hover_y = 0.0
#         self.hover_z = 0.0
#         self.hover_yaw = 0.0

#         qos_profile = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=1,
#         )

#         # --- Subscriptions -------------------------------------------------
#         self.status_sub = self.create_subscription(
#             VehicleStatus,
#             prefix + "/fmu/out/vehicle_status",
#             self.vehicle_status_callback,
#             qos_profile,
#         )

#         self.local_pos_sub = self.create_subscription(
#             VehicleLocalPosition,
#             prefix + "/fmu/out/vehicle_local_position",
#             self.local_position_callback,
#             qos_profile,
#         )

#         self.current_position = np.zeros(3)  # x, y, z (NED)
#         self.current_yaw = 0.0

#         # Velocity teleop removed for position-based control
#         # self.offboard_velocity_sub = self.create_subscription(
#         #     Twist,
#         #     '/offboard_velocity_cmd',
#         #     self.offboard_velocity_callback,
#         #     qos_profile)

#         self.attitude_sub = self.create_subscription(
#             VehicleAttitude,
#             prefix+ "/fmu/out/vehicle_attitude",
#             self.attitude_callback,
#             qos_profile,
#         )

#         self.my_bool_sub = self.create_subscription(
#             Bool,
#             prefix+"/arm_message",
#             self.arm_message_callback,
#             qos_profile,
#         )
        
#         # listned to where to go
#         self.pos_sub = self.create_subscription(
#             TrajectorySetpoint,
#             prefix+"/position_command",
#             self.pos_callback,
#             qos_profile=qos_profile,
#         )

#         # --- Publishers ----------------------------------------------------
#         self.publisher_offboard_mode = self.create_publisher(
#             OffboardControlMode,
#             prefix+"/fmu/in/offboard_control_mode",
#             qos_profile,
#         )

#         # Velocity publisher not used in position control version
#         self.publisher_velocity = self.create_publisher(
#             Twist,
#             '/fmu/in/setpoint_velocity/cmd_vel_unstamped',
#             qos_profile)

#         self.publisher_trajectory = self.create_publisher(
#             TrajectorySetpoint,
#             prefix+"/fmu/in/trajectory_setpoint",
#             qos_profile,
#         )

#         self.vehicle_command_publisher_ = self.create_publisher(
#             VehicleCommand,
#             prefix+"/fmu/in/vehicle_command",
#             10,
#         )

#         # --- Timers --------------------------------------------------------
#         # Arm / mode state machine timer
#         arm_timer_period = 0.1  # seconds (> 2 Hz)
#         self.arm_timer_ = self.create_timer(
#             arm_timer_period,
#             self.arm_timer_callback,
#         )

#         # Offboard command loop timer
#         timer_period = 0.02  # seconds (50 Hz)
#         self.timer = self.create_timer(
#             timer_period,
#             self.cmdloop_callback,
#         )

#         # --- State variables ----------------------------------------------
#         self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
#         self.arm_state = VehicleStatus.ARMING_STATE_ARMED

#         # In position-control version we do not use velocity commands
#         self.velocity = Vector3()

#         self.yaw = 0.0          # commanded yaw
#         self.trueYaw = 0.0      # measured yaw from VehicleAttitude

#         self.offboardMode = False
#         self.flightCheck = False
#         self.myCnt = 0
#         self.arm_message = False
#         self.failsafe = False

#         self.current_state = "IDLE"
#         self.last_state = self.current_state

#         # Desired position setpoint: [x, y, z, yaw]
#         # Initialize to some safe default (e.g., hover at (0,0,5m,0rad))
#         self.desired_position = np.array([spot.x, spot.y, -5.0, 0.0], dtype=float)

#     # ----------------------------------------------------------------------
#     # Subscriptions callbacks
#     # ----------------------------------------------------------------------
    

#     def pos_callback(self, msg):
#         x = msg.position[0]
#         y = msg.position[1]
#         z = msg.position[2]
#         yaw = self.desired_position[3]

#         if z == 0.0:
#             self.get_logger().warn(f"Ignoring position_command with z=0: {msg.position}")
#             return
        
#         self.goto_position(x, y, z, yaw)
        
#         self.arm_message = True
#         self.get_logger().info(f"pos_cmd: {msg.position}")



#     def arm_message_callback(self, msg: Bool):
#         self.arm_message = msg.data
#         self.get_logger().info(f"Arm Message: {self.arm_message}")

#     # State machine: arms, takes off, switches to offboard
#     def arm_timer_callback(self):
#         match self.current_state:
#             case "IDLE":
#                 if self.flightCheck and self.arm_message:
#                     self.current_state = "ARMING"
#                     self.get_logger().info("Transition: IDLE → ARMING")
                
#             case "ARMING":
#                 if not self.flightCheck or self.failsafe:
#                     self.current_state = "IDLE"
#                     self.get_logger().info("ARMING failed → IDLE")
#                     return

#                 self.arm()

#                 if self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
#                     self.current_state = "TAKEOFF"
#                     self.get_logger().info("Transition: ARMING → TAKEOFF")

#             case "TAKEOFF":
#                 if not self.flightCheck or self.failsafe:
#                     self.current_state = "IDLE"
#                     self.get_logger().info("TAKEOFF failed → IDLE")
#                     return

#                 self.arm()
#                 self.take_off()

#                 if self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
#                     self.current_state = "OFFBOARD"
#                     self.enter_offboard()
#                     self.start_hover()
#                     self.get_logger().info("Transition: LOITER → OFFBOARD")

#             # Waits while taking off. Once LOITER, switch to OFFBOARD.
#             case "LOITER":
#                 if not self.flightCheck:
#                     self.current_state = "IDLE"
#                     self.get_logger().info("Loiter, Flight Check Failed")
#                 elif (
#                     self.nav_state
#                     == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
#                 ):
#                     self.current_state = "OFFBOARD"
#                     self.get_logger().info("Loiter, Offboard")
#                 self.arm()

#             case "OFFBOARD":
#                 if (
#                     not self.flightCheck
#                     or self.arm_state != VehicleStatus.ARMING_STATE_ARMED
#                     or self.failsafe
#                 ):
#                     self.current_state = "IDLE"
#                     self.offboardMode = False
#                     self.get_logger().info("OFFBOARD lost → IDLE")
#                     return

#                 # Healthy OFFBOARD → keep controlling
#                 if self.offboard_behavior is None:
#                     self.offboard_behavior = "HOVER"

            

#         # If disarmed, require a new arm trigger
#         if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
#             self.arm_message = False

#         # Log state transitions
#         if self.last_state != self.current_state:
#             self.last_state = self.current_state
#             self.get_logger().info(self.current_state)

#         self.myCnt += 1

#     def enter_offboard(self):
#         self.myCnt = 0
#         # VEHICLE_CMD_DO_SET_MODE: param1 = base mode, param2 = custom mode
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
#             1.0,
#             6.0,
#         )
#         self.offboardMode = True

#     def state_offboard(self):
#         # Always publish offboard heartbeat
#         self.publish_offboard_control_mode()

#         if self.offboard_behavior == "HOVER":
#             self.publish_hover_setpoint()
#         elif self.offboard_behavior == "GOTO":
#             self.publish_goto_setpoint()
#         elif self.offboard_behavior == "LAND":
#             self.publish_land_setpoint()

#     # Arms the vehicle
#     def arm(self):
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
#             1.0,
#         )
#         self.get_logger().info("Arm command sent")

#     # Takes off to a fixed altitude (meters)
#     def take_off(self):
#         # param1 = minimum pitch, param7 = altitude (m)
#         self.publish_vehicle_command(
#             VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
#             param1=1.0,
#             param7=5.0,
#         )
#         self.get_logger().info("Takeoff command sent")

#     # Publishes a VehicleCommand to /fmu/in/vehicle_command
#     def publish_vehicle_command(
#         self,
#         command: int,
#         param1: float = 0.0,
#         param2: float = 0.0,
#         param7: float = 0.0,
#     ):
#         msg = VehicleCommand()
#         msg.param1 = param1
#         msg.param2 = param2
#         msg.param7 = param7  # altitude for takeoff command
#         msg.command = command  # command ID
#         msg.target_system = 1
#         msg.target_component = 1
#         msg.source_system = 1
#         msg.source_component = 1
#         msg.from_external = True
#         msg.timestamp = int(Clock().now().nanoseconds / 1000)  # microseconds
#         self.vehicle_command_publisher_.publish(msg)

#     # Receives and sets vehicle status values
#     def vehicle_status_callback(self, msg: VehicleStatus):
#         if msg.nav_state != self.nav_state:
#             self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

#         if msg.arming_state != self.arm_state:
#             self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

#         if msg.failsafe != self.failsafe:
#             self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

#         if msg.pre_flight_checks_pass != self.flightCheck:
#             self.get_logger().info(
#                 f"FlightCheck: {msg.pre_flight_checks_pass}"
#             )

#         self.nav_state = msg.nav_state
#         self.arm_state = msg.arming_state
#         self.failsafe = msg.failsafe
#         self.flightCheck = msg.pre_flight_checks_pass

#     # Attitude callback: extract yaw from quaternion
#     def attitude_callback(self, msg: VehicleAttitude):
#         orientation_q = msg.q

#         # trueYaw is the drone's current yaw value (FLU, radians)
#         self.trueYaw = -(
#             np.arctan2(
#                 2.0
#                 * (
#                     orientation_q[3] * orientation_q[0]
#                     + orientation_q[1] * orientation_q[2]
#                 ),
#                 1.0
#                 - 2.0
#                 * (
#                     orientation_q[0] * orientation_q[0]
#                     + orientation_q[1] * orientation_q[1]
#                 ),
#             )
#         )

#     # ----------------------------------------------------------------------
#     # Offboard command loop - position control version
#     # ----------------------------------------------------------------------
#     def cmdloop_callback(self):

#         if not self.offboardMode:
#             return

#         self.state_offboard()
#         # 1) Publish OffboardControlMode indicating position control
#         offboard_msg = OffboardControlMode()
#         offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
#         offboard_msg.position = True
#         offboard_msg.velocity = False
#         offboard_msg.acceleration = False
#         self.publisher_offboard_mode.publish(offboard_msg)

#         # 2) Publish a TrajectorySetpoint with desired position + yaw
#         traj = TrajectorySetpoint()
#         traj.timestamp = int(Clock().now().nanoseconds / 1000)

#         # Desired position [x, y, z]
#         traj.position[0] = float(self.desired_position[0])
#         traj.position[1] = float(self.desired_position[1])
#         traj.position[2] = float(self.desired_position[2])

#         # We are not commanding velocity/accel -> NaN
#         traj.velocity[0] = float("nan")
#         traj.velocity[1] = float("nan")
#         traj.velocity[2] = float("nan")

#         traj.acceleration[0] = float("nan")
#         traj.acceleration[1] = float("nan")
#         traj.acceleration[2] = float("nan")

#         # Yaw command from desired_position[3], no yaw rate
#         traj.yaw = float(self.desired_position[3])
#         traj.yawspeed = 0.0

#         self.publisher_trajectory.publish(traj)


#     # API to set a new position setpoint (x, y, z, yaw)
#     def goto_position(self, x: float, y: float, z: float, yaw: float):
#         """
#         Set the desired position (x, y, z) in NED frame and yaw (rad).

#         This just updates the internal setpoint; the cmdloop_callback
#         actually publishes it periodically while offboardMode is True.
#         """
#         #fix for falling down need to convert to negative
#         self.desired_position = np.array([x, y, -z, yaw], dtype=float)
#         self.offboard_behavior = "GOTO"
#         self.get_logger().info(
#             f"New position setpoint: x={x}, y={y}, z={-z}, yaw={yaw}"
#         )

#     def start_hover(self):
#         self.hover_position = self.current_position.copy()
#         self.hover_yaw = self.current_yaw
#         self.offboard_behavior = "HOVER"

#     def publish_hover_setpoint(self):
#         self.publish_position_setpoint(
#             x=self.hover_position.x,
#             y=self.hover_position.y,
#             z=self.hover_position.z,
#             yaw=self.hover_yaw
#         )

#     def publish_goto_setpoint(self):
#         if self.desired_position is None:
#             return

#         self.publish_position_setpoint(
#             x=self.desired_position[0],
#             y=self.desired_position[1],
#             z=self.desired_position[2],
#             yaw=self.desired_position[3],
#         )

#     def publish_land_setpoint(self):
#         z = self.current_position[2] + 0.05  # NED: increase z → go down

#         self.publish_position_setpoint(
#             x=self.current_position[0],
#             y=self.current_position[1],
#             z=z,
#             yaw=self.current_yaw,
#         )

#     def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float):
#         traj = TrajectorySetpoint()
#         traj.timestamp = int(Clock().now().nanoseconds / 1000)

#         traj.position[0] = float(x)
#         traj.position[1] = float(y)
#         traj.position[2] = float(z)

#         traj.velocity[0] = float("nan")
#         traj.velocity[1] = float("nan")
#         traj.velocity[2] = float("nan")

#         traj.acceleration[0] = float("nan")
#         traj.acceleration[1] = float("nan")
#         traj.acceleration[2] = float("nan")

#         traj.yaw = float(yaw)
#         traj.yawspeed = 0.0

#         self.publisher_trajectory.publish(traj)

#     def local_position_callback(self, msg: VehicleLocalPosition):
#         self.current_position[0] = msg.x
#         self.current_position[1] = msg.y
#         self.current_position[2] = msg.z

#     def publish_offboard_control_mode(self):
#         msg = OffboardControlMode()
#         msg.timestamp = int(Clock().now().nanoseconds / 1000)
#         msg.position = True
#         msg.velocity = False
#         msg.acceleration = False
#         msg.attitude = False
#         msg.body_rate = False
#         self.publisher_offboard_mode.publish(msg)


# def main(args=None):
#     rclpy.init(args=args)

#     offboard_control = OffboardControl()

#     rclpy.spin(offboard_control)

#     offboard_control.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


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

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy # ROS 2 Python client library
import numpy as np # Used for math (cos, sin) to generate circular trajectories.
from rclpy.node import Node # Imports the base ROS 2 node class. 
                            # Your class must inherit from this to:
                                # create publishers/subscribers
                                # create timers
                                # access ROS clocks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy # PX4 requires specific QoS or messages silently won’t arrive.

from px4_msgs.msg import OffboardControlMode # tells PX4 what kind of control you are sending (position / velocity / etc)
from px4_msgs.msg import TrajectorySetpoint # the actual position setpoint PX4 should fly to
from px4_msgs.msg import VehicleStatus # PX4’s state machine feedback (armed, nav mode, failsafe, etc)
from px4_msgs.msg import VehicleCommand # Import Vehicle commands


class OffboardControl(Node):

    # Initializes the ROS node
    # Node name = minimal_publisher
    # This name appears in ros2 node list
    def __init__(self):
        super().__init__('minimal_publisher')

        # QoS profiles

        # Publisher QoS
        # BEST_EFFORT means that PX4 does not retry messages → this matches PX4 behavior
        # TRANSIENT_LOCAL means that PX4 will receive the last message even if it subscribes late
        # KEEP_LAST, depth=0 means that they store only the most recent sample
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # Subscriber Qos
        # only new thing is VOLATILE which means only receive new messages
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # Subscribes to PX4 vehicle status.
        # This gives you navigation mode, arming state, failsafe state
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile_sub)

        # Only difference from the last one is this is legacy / compatibility — some PX4 versions publish _v1.
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile_sub
        )

        # This is the OFFBOARD heartbeat.
        # Publishes OffboardControlMode to PX4.
        # If this stops → PX4 exits OFFBOARD → drone falls.
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, 
            'fmu/in/offboard_control_mode', 
            qos_profile_pub
        )

        # Publishes the actual position setpoint PX4 flies to.
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, 
            'fmu/in/trajectory_setpoint', 
            qos_profile_pub
        )
        
        # Creates a timer that calls cmdloop_callback() at 50 Hz.
        # PX4 requires OffboardControlMode at > 2 Hz, and TrajectorySetpoint at > 2 Hz
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period


        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)

        # Stores PX4 state machine values. Initially set to defaults
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega
        # which would result in large discontinuities in setpoints
        # Internal trajectory state:
        # theta = angle around circle, others copied from parameters
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

    # Called every time PX4 publishes VehicleStatus.
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        # Stores PX4 state so the control loop can react.
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    # Takes off to a fixed altitude (meters)
    def take_off(self):
        # param1 = minimum pitch, param7 = altitude (m)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=1.0,
            param7=5.0,
        )
        self.get_logger().info("Takeoff command sent")

    def enter_offboard(self):
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

    



    # The heart of OFFBOARD
    def cmdloop_callback(self):

        # Publish offboard control modes
        # Most important block in the file
        # Every thick it tells PX4: “I am controlling POSITION” refreshes OFFBOARD heartbeat
        # Without this: PX4 exits OFFBOARD, motors idle, and gravity wins
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position=True
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        self.arm()
        self.take_off()
        self.enter_offboard()

        # Only send trajectory if PX4 confirms that vehicle is armed, vehicle is already in OFFBOARD
        # This prevents: publishing setpoints too early, PX4 rejecting commands
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            # Generates a circle in XY, fixed altitude.
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude

            # Sends the setpoint to PX4. PX4 tracks it using position controller.
            self.publisher_trajectory.publish(trajectory_msg)

            # Advances the angle → smooth circular motion.
            self.theta = self.theta + self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()