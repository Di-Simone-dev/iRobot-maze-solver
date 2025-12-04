import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import time

from custom_msg import Solve


class MazeSolverActionServer(Node):

    def __init__(self):
        super().__init__('maze_solver_action_server')
        self._solve_action_server = ActionServer(
            self,
            Solve,
            'maze_solve',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Feedback msg
        feedback_msg = Solve.Feedback()
        feedback_msg.current_position = [0, 0]

        # Publishing feedback msg
        goal_handle.publish_feedback(feedback_msg)

        # Publishing goal status
        goal_handle.succeed()

        # Result
        result = Solve.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    maze_solver_action_server = MazeSolverActionServer()

    rclpy.spin(maze_solver_action_server)


if __name__ == '__main__':
    main()