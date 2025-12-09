import rclpy
from rclpy import qos
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from irobot_create_msgs import DockStatus, KidnapStatus, HazardDetectionVector, IrIntensityVector

from custom_msg import Solve, ActuatorMove, ActuatorDock, Stop


class MazeSolverActionServer(Node):

    def __init__(self):
        super().__init__('maze_solver_action_server')

        # ====================
        # States
        # ====================
        self._is_docked = False
        self._is_kidnapped = False
        self._hazard = []

        # ====================
        # Topic subscription
        # ====================
        
        # DOCK STATUS
        dock_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._dock_status_subscription = self.create_subscription(
            DockStatus,
            'dock_status',
            self.execute_dock_status_callback,
            dock_qos
        )
        self._dock_status_subscription

        # KIDNAP STATUS
        kidnap_status_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._kidnap_status_subscription = self.create_subscription(
            KidnapStatus,
            'kidnap_status',
            self.execute_kidnap_status_callback,
            kidnap_status_qos
        )
        self._kidnap_status_subscription

        # HAZARD VECTOR
        hazard_detection_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._hazard_detection_subscription = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.execute_hazard_detection_callback,
            hazard_detection_qos
        )
        self._hazard_detection_subscription

        # IR VECTOR
        ir_intensity_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._ir_intensity_subscription = self.create_subscription(
            IrIntensityVector,
            'ir_intensity',
            self.execute_ir_intensity_callback,
            ir_intensity_qos
        )
        self._ir_intensity_subscription

        # ====================
        # Service subscriptions
        # ====================
        self._actuator_e_stop_client = self.create_client(
            Stop,
            'actuator_power'
        )


        # ====================
        # Action client subscriptions
        # ====================
        self._actuator_movement_action_client = ActionClient(
            self,
            ActuatorMove,
            'actuator_movement'
        )

        self._actuator_dock_action_client = ActionClient(
            self,
            ActuatorDock,
            'actuator_dock'
        )


        # ====================
        # Action servers
        # ====================
        self._solve_action_server = ActionServer(
            self,
            Solve,
            'maze_solve',
            self.execute_solve_callback)
    

    # ====================
    # Callbacks
    # ====================

    # ====================
    # Topic Callbacks
    # ====================

    # DOCK STATUS CALLBACK
    def execute_dock_status_callback(self, msg):
        self._is_docked = msg.is_docked

    # KIDNAPP STATUS CALLBACK
    def execute_kidnap_status_callback(self, msg):
        self._is_kidnapped = msg.is_kidnapped
    
    # HAZARD VECTOR CALLBACK
    def execute_hazard_detection_callback(self, msg):
        for hazard in msg.detections:
            self._hazard.append(hazard)
    
    # IR INTENSITY CALLBACK
    def execute_ir_intensity_callback(self, msg):
        for ir in msg.readings:
            self._hazard.append(ir)

    
    # ====================
    # Service Callbacks
    # ====================

    # ====================
    # Action Callbacks
    # ====================
    def execute_solve_callback(self, goal_handle):
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