import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from multi_hop_interfaces.action import NetworkSimulator


class NetworkSimActionClient(Node):

    def __init__(self):
        super().__init__('networksim_action_client')
        self._action_client = ActionClient(self, NetworkSimulator, 'networksimulator')

    def send_goal(self, request):
        goal_msg = NetworkSimulator.Goal()
        goal_msg.sender_id = request

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = NetworkSimActionClient()

    future = action_client.send_goal('goal :3')

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
