import py_trees
from helpers import *

#STEP 2 DEL PLEDGE
class FollowWall(py_trees.behaviour.Behaviour):
    def __init__(self, name="Follow Wall"):
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

    def update(self):
        current_pos = self.BB.get("current_position")
        heading = self.BB.get("heading")
        counter = self.BB.get("pledge_counter")
        last_action = self.BB.get("last_action")
        
        forward = forward_cell(current_pos, heading)
        
        # Check success condition: aligned with global heading and path is free
        if counter == 0 and heading == self.BB.get("heading_global") and is_free(forward):
            return py_trees.common.Status.SUCCESS
        
        # Get all adjacent cells
        left = left_cell(current_pos, heading)
        right = right_cell(current_pos, heading)
        back = backward_cell(current_pos, heading)
        backleft = backleft_cell(current_pos, heading)
        backright = backright_cell(current_pos, heading)
        
        # Debug prints for decision points
        if "Turn" not in last_action and is_free(back):
            free_dirs = [d for d, cell in [("left", left), ("right", right), ("forward", forward)] if is_free(cell)]
            #if len(free_dirs) >= 2:
            #    print(f"Choice between {' and '.join(free_dirs)} with pledge counter {counter}")
        
        # Determine turn priority based on pledge counter
        prioritize_left = counter > 0
        
        # Try movements in priority order
        new_heading, action = self._choose_direction(
            heading, forward, left, right, back, backleft, backright,
            last_action, prioritize_left
        )
        
        if action == "FAILURE":
            self.BB.set("last_action", "Wall ended → no free path")
            return py_trees.common.Status.FAILURE
        
        self.BB.set("heading", new_heading)
        self.BB.set("last_action", action)
        
        # Return SUCCESS if moving forward (no heading change), FAILURE if turning
        return py_trees.common.Status.SUCCESS if new_heading == heading else py_trees.common.Status.FAILURE
    
    def _choose_direction(self, heading, forward, left, right, back, backleft, backright, last_action, prioritize_left):
        """Choose direction based on pledge counter priority."""
        
        if prioritize_left:
            # Priority: left → forward → right → back
            if is_free(left) and not is_free(backleft) and "Turn Left" not in last_action:
                return (heading - 90) % 360, "Turn Left (wall follow)"
            if is_free(forward):
                return heading, "Continue forward"
            if is_free(right) and not is_free(backright):
                return (heading + 90) % 360, "Turn Right (wall follow)"
            if is_free(back):
                return (heading - 180) % 360, "Turn Back (2 times left)"
        else:
            # Priority: right → forward → left → back
            if is_free(right) and not is_free(backright) and "Turn Right" not in last_action:
                return (heading + 90) % 360, "Turn Right (wall follow)"
            if is_free(forward):
                return heading, "Continue forward"
            if is_free(left) and not is_free(backleft):
                return (heading - 90) % 360, "Turn Left (wall follow)"
            if is_free(back):
                return (heading - 180) % 360, "Turn Back (2 times left)"
        
        return heading, "FAILURE"