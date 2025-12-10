#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import time
from rclpy.clock import Clock

from std_msgs.msg import Bool
from px4_msgs.msg import TrajectorySetpoint

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)


# ay lemme drive da boat
class Lemme_Drive(Node):
    def __init__(self, spot, drone_id):
        super().__init__(f"lemme_drive_{drone_id}")
        self.spot = spot
        self.id = drone_id
        self.prefix = f"/px4_{drone_id}"
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        
        self.arm_pub = self.create_publisher(
            Bool,
            self.prefix + "/arm_message",
            qos_profile=qos_profile,
        )
        
        self.pos_pub = self.create_publisher(
            TrajectorySetpoint,
            self.prefix + "/position_command",
            qos_profile=qos_profile,
        )
        
        self.command_timer = self.create_timer(1, self.command_timer_callback)
        
        # send arm
        self.arm_timer = self.create_timer(2.0, self.send_arm_trigger)
        
    # send an arm emssage (once)
    def send_arm_trigger(self):
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
        self.destroy_timer(self.arm_timer)
        
    def command_timer_callback(self):
        
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        
        # head on over chappie
        traj_msg.position[0] = float(self.spot.x)
        traj_msg.position[1] = float(self.spot.y)
        traj_msg.position[2] = 0.0
        traj_msg.yaw = 0.0
        
        # I DOOOOONT CAAAHE
        traj_msg.velocity[0] = traj_msg.velocity[1] = traj_msg.velocity[2] = float("nan")
        traj_msg.acceleration[0] = traj_msg.acceleration[1] = traj_msg.acceleration[2] = float("nan")
        #self.get_logger().info(f"sent msg {self.id}")
        self.pos_pub.publish(traj_msg)
   
   

        
        
        
        
        
        
