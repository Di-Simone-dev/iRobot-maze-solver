import py_trees
from maze_solver import config
from time import sleep
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

class LidarMap(py_trees.behaviour.Behaviour):

    def __init__(self, name="LIDAR_MAP"):
        super().__init__(name)
        self.BB = self.attach_blackboard_client(name=self.name)
        self.BB.register_key(key="lidar_scan", access=py_trees.common.Access.READ)
        self.BB.register_key(key="map", access=py_trees.common.Access.WRITE)
        self.BB.register_key(key="clock", access=py_trees.common.Access.READ)
        self.BB.register_key(key="current_position", access=py_trees.common.Access.READ)
        self.BB.register_key(key="logger", access=py_trees.common.Access.READ)
        

    def update(self):
        # Check time
        while(1):
            if(type(self.BB.get("lidar_scan")) != LaserScan):
                sleep(0.200)
            else:
                self.BB.get("logger").info(f"LIDAR: {self.BB.get("lidar_scan")}")
                if((self.BB.get("clock").now() - Time.from_msg(self.BB.get("lidar_scan").header.stamp)).nanoseconds < 750_000_000):
                    angle_increment = self.BB.get("lidar_scan").angle_increment
                    map = self.BB.get("map")

                    # Fisrt ray
                    first_ray = 1.5708 - config.ANGLE
                    angle = 0
                    position = 0
                    while(angle < first_ray):
                        angle += angle_increment
                        position += 1
                    
                    if(abs(angle - angle_increment - first_ray) < abs(angle - first_ray)):
                        position -= 1
                    
                    if(self.BB.get("lidar_scan").ranges[position] > 1.12):
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "free"
                    else:
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "wall"

                    #Central ray
                    central_ray = 1.5708
                    angle = 0
                    position = 0
                    while(angle < central_ray):
                        angle += angle_increment
                        position += 1
                    
                    if(abs(angle - angle_increment - central_ray) < abs(angle - central_ray)):
                        position -= 1
                    
                    if(self.BB.get("lidar_scan").ranges[position] > 1.12):
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "free"
                    else:
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "wall"
                    
                    #Last ray
                    last_ray = 1.5708 + config.ANGLE
                    angle = 0
                    position = 0
                    while(angle < last_ray):
                        angle += angle_increment
                        position += 1
                    
                    if(abs(angle - angle_increment - last_ray) < abs(angle - last_ray)):
                        position -= 1

                    if(self.BB.get("lidar_scan").ranges[position] > 1.12):
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "free"
                    else:
                        map[(self.BB.get("current_position")[0], self.BB.get("current_position")[1])] = "wall"

                    self.BB.get("logger").info("LIDAR Map ended!")

                    return py_trees.common.Status.SUCCESS
