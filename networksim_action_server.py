import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from multi_hop_controller.action import NetworkSimulator


class NetworkSimActionServer(Node):

    def __init__(self):
        super().__init__('networksim_action_server')
        self._action_server = ActionServer(
            self,
            NetworkSimulator,
            'networksimulator',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = NetworkSimulator.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    networksim_action_server = NetworkSimActionServer()

    rclpy.spin(networksim_action_server)


if __name__ == '__main__':
    main()
