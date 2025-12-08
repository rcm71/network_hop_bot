import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from drone_msgs.msg import DroneStatus
import random
import time

class DroneNode(Node):

    def __init__(self, drone_id):
        super().__init__("drone_" + str(drone_id))
        self.id = drone_id

        self.publisher = self.create_publisher(
            DroneStatus, 
            "/drone_status", 
            10
        )

        self.subscription = self.create_subscription(
            DroneStatus,
            "/drone_status",
            self.status_callback,
            10
        )

        self.timer = self.create_timer(0.2, self.publish_status)

    def publish_status(self):
        msg = DroneStatus()
        msg.id = self.id

        # Example: fake data for simulation
        msg.position = Point(
            x=random.uniform(0, 50),
            y=random.uniform(0, 50),
            z=random.uniform(5, 20)
        )
        msg.velocity = Vector3(
            x=0.1, y=0.0, z=0.0
        )
        msg.battery_level = random.uniform(30, 100)
        msg.link_quality = random.uniform(0, 1)
        msg.role = "relay"
        msg.timestamp = time.time()

        self.publisher.publish(msg)

    def status_callback(self, msg):
        if msg.id != self.id:
            self.get_logger().info(
                f"Received status from Drone {msg.id}: "
                f"Position: ({msg.position.x:.1f}, {msg.position.y:.1f}, {msg.position.z:.1f}), "
                f"link={msg.link_quality:.2f}"
            )

def main():
    rclpy.init()
    node = DroneNode(drone_id=1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()