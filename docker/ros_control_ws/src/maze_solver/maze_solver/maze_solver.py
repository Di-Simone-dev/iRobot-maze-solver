import rclpy
from rclpy import qos
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node

from irobot_create_msgs import DockStatus, KidnapStatus, HazardDetectionVector, IrIntensityVector

from custom_msg import Solve, ActuatorMove, ActuatorDock, Stop

from sensor_msgs import PointCloud2


class MazeSolver(Node):

    def __init__(self):
        super().__init__('maze_solver')

        # ====================
        # States
        # ====================
        self._is_docked = False
        self._is_kidnapped = False
        self._hazard = []
        self._ir_sensors = []
        self._lidar_scan = []
        self._algoritm = "";

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

        # LIDAR SCAN
        lidar_scan_qos = qos.QoSProfile(
            depth = 10,
            reliability = qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability = qos.QoSDurabilityPolicy.VOLATILE
        )
        self._lidar_scan_subscription = self.create_subscription(
            PointCloud2,
            'lidar/scan',
            self.execute_lidar_scan_callback,
            lidar_scan_qos
        )
        self._lidar_scan_subscription

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
            execute_callback=self.execute_solve_callback,
            goal_callback=self.goal_solve_callback,
            cancel_callback=self.cancel_solve_callback
        )
    

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

    # LIDAR SCAN CALLBACK
    def execute_lidar_scan_callback(self, msg):
        self._lidar_scan.clear()
        self._lidar_scan = msg
    
    # HAZARD VECTOR CALLBACK
    def execute_hazard_detection_callback(self, msg):
        self._hazard.clear()
        for hazard in msg.detections:
            self._hazard.append(hazard)
    
    # IR INTENSITY CALLBACK
    def execute_ir_intensity_callback(self, msg):
        self._ir_sensors.clear()
        for ir in msg.readings:
            self._ir_sensors.append(ir)

    
    # ====================
    # Service Callbacks
    # ====================

    # ====================
    # Action Callbacks
    # ====================

    # Decide if accept or refuse the current goal
    def goal_solve_callback(self, goal_request):
        if(goal_request.algorithm == "PLEDGE"
           and goal_request.algorithm == "TREMAUX" 
           and goal_request.start_position.count() == 2
           and goal_request.start_position[0] >= 0
           and goal_request.start_position[1] >= 0
           and goal_request.end_position.count() == 2
           and goal_request.end_position[0] >= 0
           and goal_request.end_position[1] >= 0):
            
            return GoalResponse.ACCEPT
        
        return GoalResponse.REJECT

    # Decide if the goal is cancellable
    def cancel_solve_callback(self, goal_handle):
        self.get_logger().info('Cancel goal requested')
        return CancelResponse.ACCEPT
    
    # Execute the goal
    def execute_solve_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Setting algoritm type
        self._algoritm = goal_handle.algoritm

        # Check if docked, then undock
        if self._is_docked:
            msg_goal = ActuatorDock.Goal()
            msg_goal.type = "UNDOCK"

            # Sync undock status
            future_goal = self._actuator_dock_action_client.send_goal_async(msg_goal)
            rclpy.spin_until_future_complete(self, future_goal)

            goal_handle = future_goal.result()

            if not goal_handle.accepted:
                self.get_logger().warn("Goal rifiutato")
                return

            self.get_logger().info("Goal accettato")

            future_result = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, future_result)

            if not future_result.result().result.status:
               self.get_logger().warn("Goal rifiutato")
               return
        

        # ====================
        #    BEHAVIOUR
        # ====================
        

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

    maze_solver_action_server = MazeSolver()

    rclpy.spin(maze_solver_action_server)


if __name__ == '__main__':
    main()