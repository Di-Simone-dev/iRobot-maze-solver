import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from maze_solver.helpers import *
from custom_msg.action import ActuatorMove
from maze_solver import config
import math

class ChooseDirectionTremaux(Behaviour):
    def __init__(self, name="Move Forward Trémaux"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="goal_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="paused", access=py_trees.common.Access.READ)
        self.BB.register_key(key="kidnapped", access=py_trees.common.Access.READ)
        self.BB.register_key(key="hazard", access=py_trees.common.Access.READ)
        self.BB.register_key(key="ir_sensors", access=py_trees.common.Access.READ)
        self.BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        self.BB.register_key(key="heading", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="reached_exit", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="last_action", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="visited", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="allow_visit_fallback", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="algorithm_mode", access=py_trees.common.Access.READ)
        self.BB.register_key(key="pledge_counter", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="heading_global", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="busy", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="actuator_movement_action_client", access=py_trees.common.Access.READ)

    def update(self):
        if self.BB.get("reached_exit"):
            return Status.SUCCESS
        
        chosen_dir = self.BB.get("chosen_direction")
        
        if chosen_dir is None:
            return Status.FAILURE
        
        if chosen_dir == self.BB.get("heading"):
            return Status.SUCCESS
        
        self.BB.set("busy", True)

        # ROTAZIONE
        angle = self.BB.get("heading") - chosen_dir
        angle = round(angle * math.pi / 180, 2)
        goal_msg = ActuatorMove.Goal()
        goal_msg.type = "ANGLE"
        goal_msg.distance = angle
        goal_msg.max_speed = config.ROTATION_SPEED
        actuator_movement_action_client = self.BB.get("actuator_movement_action_client")
        future = actuator_movement_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.end_movement_callback)

        self.BB.set("heading", chosen_dir)  # Aggiorna orientamento

        return Status.FAILURE
    
    def end_movement_callback(self, future):
        self.BB.set("busy", False)