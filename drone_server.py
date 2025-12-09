import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from multi_hop_controller.action import NetworkSimulator
import time


class NetworkSimActionServer(Node):
    def __init__(self):
        super().__init__('networksim_action_server')

        # Create the Action Server
        self._action_server = ActionServer(
            self,
            NetworkSimulator,
            'networksimulator',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        # Receive the goal message from the client
        goal = goal_handle.request
        self.get_logger().info(f'Received goal: {goal}')

        # Optional: send feedback while processing
        for i in range(5):
            feedback_msg = NetworkSimulator.Feedback()
            feedback_msg.status_message = f'Processing step {i + 1}/5'
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # simulate work

        # Create and send the result back
        result = NetworkSimulator.Result()
        result.response = f"Goal processed: {goal}"
        goal_handle.succeed()  # mark the goal as succeeded
        self.get_logger().info(f'Goal completed: {goal}')
        return result
    

    
def main(args=None):
    rclpy.init(args=args)
    server = NetworkSimActionServer()
    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()