import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from custom_msg.action import MazeSolver

class MazeSolverActionServer(Node):
  def __init__(self):
    super().__init__('maze_solver_action_server')
    self._action_server = ActionServer(self, MazeSolver, 'maze_solver', self.callback)
    
    def callback(self):
      self.get_logger().info('Executing goal...')
      result = MazeSolver.Result()
      return result

def main():
  rclpy.init(args=args)
  maze_solver_action_server = MazeSolverActionServer()
  rclpy.spin(maze_solver_action_server)

if __name__ == "__main__":
  main()