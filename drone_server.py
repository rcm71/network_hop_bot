import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from multi_hop_interfaces.action import NetworkSimulator
import time


class NetworkSimActionServer(Node):
    def __init__(self):
        super().__init__('networksim_action_server')

        self.reference_distance = 1.0
        self.reference_loss = 40.0
        self.path_loss_exp = 2.2
        self.receiver_sensitivity = -85.0
        self.noise_floor = -95.0

        self.drone_positions = {}

        # SETUP SUBSCRIPTION(S) TO WRITE DRONE POSITIONS
        # CALLBACK ASSUMES DICTIONARY INDEXED BY ID -> POSITION
        
        self._action_server = ActionServer(
            self,
            NetworkSimulator,
            'networksimulator',
            self.execute_callback
        )

    def get_distance(self, pos1, pos2):
        return math.sqrt(
            (pos1.x - pos2.x)**2 + 
            (pos1.y - pos2.y)**2 + 
            (pos1.z - pos2.z)**2
        )

    def calculate_rssi(self, distance):
        tx_power = 20.0
        if distance <= 0.1:
            return tx_power

        path_loss = self.reference_loss + 10 * self.path_loss_exp * math.log10(distance / self.reference_distance)

        return tx_power - path_loss

    def execute_callback(self, goal_handle):
        sender_id = goal_handle.sender_id
        self.get_logger().info(f'TRANSMISSION LEAVING {sender_id}')
        result = NetworkSimulator.Result()

        # deeb ugg
        self.get_logger().info(f'Positions: {self.drone_positions}')

        sender_pos = self.drone_positions[sender_id]
        drones_reached = []
        potential_drones = [name for name in self.drone_positions if name != sender_id]

        for drone in potential_drones:
            drone_pos = self.drone_positions[drone]
            dist = self.get_distance(sender_pos, drone_pos)
            rssi = self.calculate_rssi(dist)
            if rssi >= self.reciever_sensitivity:
                drones_reached.append(drone)

        goal_handle.succeed()
        self.get_logger().info(f'{sender_id} COMPLETE')
        return result    

def main(args=None):
    rclpy.init(args=args)
    server = NetworkSimActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
