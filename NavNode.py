#!/usr/bin/env python
"""
Navigation node for ROS2.


Author: Hunter Fauntleroy
"""
import math
import time
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action.client import ActionClient
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils
import tf_transformations


import numpy as np

class Point:
    def __init__(self, value=0, x=None, y=None):
        self.value = value
        self.x = x
        self.y = y


class TempNode(rclpy.node.Node):

    priority_point_value = 1000


    def __init__(self):
        super().__init__('temp_node')
        self.map = None # occupancy grid map
        self.start_time = None
        self.goal_future = None # future for current goal
        self.cancel_future = None # future for goal cancellation
        self.victims = [] # list of victims found
        self.path = [] # list of points to travel to. When empty it draws from points (Use linked list)
        self.points = [] # store points in list
        self.point_index = 0 # index for current point in points list
        self.target_point = None # current target point for pathfinding
        self.goal = None # current navigation goal for action client
        self.timeout = 60.0  # 60 seconds timeout per goal
        self.future_event = None
        self.priority_point_value = 1000

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Create publisher and subscriptions
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=latching_qos)

        # Declare TempNode initialized
        self.get_logger().info("TempNode initialized")

        # Create timers for periodic checks
        self.create_timer(.1, self.goal_checker_callback)
        self.create_timer(1.0, self.path_planning_callback)
        self.create_timer(1.0, self.check_for_detours)



        # Declare parameters
        self.declare_parameter('node_count', 50)
        self.declare_parameter('threshold', 0.0) # Change to parameters needed



    # -------------------- callbacks --------------------
    def map_callback(self, map_msg):
        """Process the map message for movement planning."""
        self.map = map_utils.Map(map_msg)

    def goal_checker_callback(self):

        """Periodically check in on the progress of navigation."""

        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")
                self.ac.destroy()
                self.future_event.set_result(False)
        else:

            if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS. EXITING!")
                self.path[self.point_index] = None
                self.point_index += 1
                self.ac.destroy()
                self.future_event.set_result(True)

            if self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")
                self.path[self.point_index] = None
                self.point_index += 1
                self.ac.destroy()
                self.future_event.set_result(False)

            elif time.time() - self.start_time > self.timeout:
                self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
                self.cancel_future = self.goal_future.result().cancel_goal_async()

    def path_planning_callback(self):
        """Periodically check and plan path if needed."""
        if self.map is None: # Cannot plan without a map
            self.get_logger().info("No map available for path planning.")
            return

        if self.points.length == 0:
            self.get_logger().info("Generating new points.")
            self.find_random_valid_points(number_of_nodes=50)
            return

    def check_for_detours(self):
        """Check if there are better points to navigate to en route to current target."""
        if not self.points or len(self.points) < 2:
            return  # Not enough points to check for detours
        target_point = self.points.pop()

        current_position = self.get_current_position() # Placeholder for getting current position
        for point in self.points:
            distance_to_next = math.hypot(current_position.x - point.x, current_position.y - point.y)
            distance_to_target = math.hypot(current_position.x - target_point.x, current_position.y - target_point.y)

            if point.value == self.priority_point_value or distance_to_next < distance_to_target / 2.0 and point.value / self.start_time > 5:  # Detour threshold and value threshold change to parameters
                self.get_logger().info(f"Detour detected to point ({point.x}, {point.y})")
                self.path.insert(point) # change to stack stuff
                self.chart_path_to_target(point) # change
                break

        pass



    # -------------------- planning utilities --------------------


    def find_random_valid_points(self, number_of_nodes=20):
        """Sample random cells and return Points (world coords) that are free.
           Points are then scored by node_value().
        """

        if self.map is None:
            return []
        points = []

        max_tries = number_of_nodes * 6
        tries = 0
        while len(points) < number_of_nodes and tries < max_tries:
            tries += 1
            x = np.random.randint(0, self.map.info.width - 1)
            y = np.random.randint(0, self.map.info.height - 1)
            val = map_utils.get_cell(self.map, x, y)
            if val == 0:
                points.append(Point(value=0, x=x, y=y))
            self.node_value(points)

        return points



    def node_value(self, points, range=4, threshold=10):
        """Assign values to points based on free space around them and spacing."""

        if not points or self.map is None:
            return

        # Reset values before scoring
        for p in points:
            p.value = 0.0

        for point in points:
            # Clamp search window to map bounds
            x_min = max(0, point.x - range)
            x_max = min(self.map.info.width - 1, point.x + range)
            y_min = max(0, point.y - range)
            y_max = min(self.map.info.height - 1, point.y + range)

            for i in range(x_min, x_max + 1):  # Iterate through square area within bounds
                for j in range(y_min, y_max + 1):
                    val = self.map.get_cell(i, j)
                    if val == 0:
                        point.value += 1 # Increment value for each free cell in range
                for j in range(y_min, y_max + 1):
                    val = self.map.get_cell(i, j)
                    if val == 0:
                        point.value += 1 # Increment value for each free cell in range

        # Penalize points that are too close to each other, reward spaced-out points
        for p in points:
            for q in points:
                if q is p:
                    continue
                dist = math.hypot(p.x - q.x, p.y - q.y)
                if dist < threshold:
                    # if another point nearby has high value, give a small boost (encourage cluster usefulness)
                    p.value += 0.1 * q.value
                    p.value -= 0.5  # small penalty for crowding

        # Finally, sort points by value
        self.sort_points_by_value(points)


    # -------------------- path planning --------------------

    def create_nav_goal(self, point, theta):
        """Create a NavigateToPose Goal from a Point and orientation theta (radians)."""
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = point.x
        goal.pose.pose.position.y = point.y

        # We need to convert theta to a quaternion....
        quaternion = tf_transformations.quaternion_from_euler(0, 0, theta, 'rxyz')
        goal.pose.pose.orientation.x = quaternion[0]
        goal.pose.pose.orientation.y = quaternion[1]
        goal.pose.pose.orientation.z = quaternion[2]
        goal.pose.pose.orientation.w = quaternion[3]
        return goal


    def navigate_to_target(self, target_point):
        # Placeholder for navigation logic
        self.goal = self.create_nav_goal(target_point, 0.0)
        self.ac.wait_for_server()
        self.start_time = time.time()
        self.goal_future = self.ac.send_goal_async(self.goal)


    def map_movement_planning(self):
        if self.map is not None:
            points = self.find_random_valid_points(number_of_nodes=50, map_msg=self.map)
            self.add_to_points(points)
            current_target = self.points[0] # change
            self.chart_path_to_target(current_target)






def main():
    rclpy.init()
    temp_node = TempNode()
    rclpy.spin(temp_node)
    temp_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# Set up canceling movements
# How to get current position
# implement stack of points stored in path
# Have detour detection in a callback
# Have path navigation handled in time callback
# Recalculate path when new priority target is added
# When victim found change map to say occupied so it is not revisited
# Vision team needs to inform nav node of victim locations
# Work out best way to create path from points and then remove points as they are visited
# Use binary tree to store points for faster access?
# Add in parameters
# Check for syntax and logic errors
# fix path planning callback to properly draw from points
