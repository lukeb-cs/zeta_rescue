#!/usr/bin/env python
"""
Navigation node for ROS2.


Author: Hunter Fauntleroy, Jessica Debes & Joshua Sun
"""
import array
import math
import os
import time
import cv2
from cv_bridge import CvBridge
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action.client import ActionClient
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from functools import total_ordering
from rclpy.task import Future

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils
from sensor_msgs.msg import Image
import tf_transformations
from collections import deque
from std_msgs.msg import Empty
 # from zeta_competition_interfaces.msg import Victim


import numpy as np

class Point:
    def __init__(self, value=0.0, x=0.0, y=0.0, z=0.0, theta=0.0):
        self.value = value
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

    def equals(self, other):
        return self.x == other.x and self.y == other.y

    def __eq__(self, other):
        return (self.x, self.y, self.theta, self.value) == \
               (other.x, other.y, other.theta, other.value)
    def __hash__(self):
        return hash((self.x, self.y))

    def __lt__(self, other):
        return self.value < other.value

class Stack:
    def __init__(self):
        # Initialize an empty deque
        self.stack = deque()

    def push(self, item):
        """Push an item onto the stack."""
        self.stack.append(item)  # O(1) operation

    def pop(self):
        """Pop the top item from the stack."""
        if self.is_empty():
            raise IndexError("Pop from empty stack")
        return self.stack.pop()  # O(1) operation

    def peek(self):
        """Return the top item without removing it."""
        if self.is_empty():
            raise IndexError("Peek from empty stack")
        return self.stack[-1]

    def is_empty(self):
        """Check if the stack is empty."""
        return len(self.stack) == 0

    def size(self):
        """Return the number of items in the stack."""
        return len(self.stack)

    def remove(self, item):
        """Remove a specific item from the stack."""
        try:
            self.stack.remove(item)
        except ValueError:
            raise ValueError("Item not found in stack")


class TempNode(rclpy.node.Node):



    def __init__(self):
        super().__init__('temp_node')
        self.map = None # occupancy grid map
        self.start_time = None
        self.goal_future = None # future for current goal
        self.cancel_future = None # future for goal cancellation
        self.path = Stack() # stack of points to travel to. When empty it draws from points (Use linked list)
        self.points = Stack() # store points in list
        self.point_index = 0 # index for current point in points list
        self.goal = None # current navigation goal for action client
        self.timeout = 60.0  # 60 seconds timeout per goal
        self.future_event = None
        self.priority_point_value = 1000

        self.bridge = CvBridge()
        self.latest_victim_pose = None
        self.latest_image = None
        self.reported_victims = [] # list of victims

        self.at_victim = False
        self.rotating = False
        self.rotation_start = None

        self.save_path = os.path.expanduser('~/victim_images')
        os.makedirs(self.save_path, exist_ok=True)
        self.distance_travelled = 0 # Running odometer
        self.speed = None # Average bot speed: Total distance travelled / time
        self.start_position = None # Return position when finished
        self.position = None # Makes the assumption that the bot starts at 0,0 upon initialization.
        self.max_time = 300 # Time of test, typcially 5 min. Bot expected to return by this time
        self.returning = False # Whether the bot's only objective is to return to start.

        # Create the action client.
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Create publisher and subscriptions
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=latching_qos)
        self.create_subscription(PointStamped, 'nav_point', self.point_callback, qos_profile=qos_profile_sensor_data)
        self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10) #CHECK ROS TOPIC FOR CAMERA

        self.create_subscription(Empty, '/report_requested', self.report_callback, 10)
        # self.create_publisher(Victim, '/victim', 10)
        self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.speed_and_distance_callback, qos_profile=latching_qos)
        # Type unclear. Could be "Pointstamped" or "Point" or something else.

        # Declare TempNode initialized
        self.get_logger().info("TempNode initialized")

        # Create timers for periodic checks
        self.create_timer(0.1, self.rotation_callback)
        self.create_timer(1, self.goal_checker_callback)
        self.create_timer(.1, self.fill_path_and_points_callback)
        self.create_timer(.1, self.check_for_detours)

        # Declare parameters
        self.declare_parameter('node_count', 50)
        self.declare_parameter('threshold', 0.0) # Change to parameters needed



    # -------------------- callbacks --------------------
    def image_callback(self, msg):
        #Add
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def report_callback(self, msg):
        if self.latest_image is None:
            self.get_logger.warn("No image available")
            return
        if self.latest_victim_pose is None:
            self.get_logger.warn("No victim position available")
            return

        for p in self.reported_victims:
            if abs(p.location.pose.position.x - self.latest_victim_pose.pose.position.x) < 0.5:
                self.get_logger().warn("Duplicate!")

        #saving photos in case
        filename = f"victim_{int(time.time())}.jpg"
        filepath = os.path.join(self.save_path, filename)
        cv2.imwrite(filepath, self.latest_image)

        victim = Victim()
        victim.image = self.bridge.cv2_to_imgmsg(self.latest_image, encoding='bgr8')
        victim.location = self.latest_victim_pose

        self.victim_pub.publish(victim)
        self.reported_victims.append(victim)

    def rotation_callback(self):
        #Twisting the robot after it is at a victim to get a good picture, not necessary but good for a good pic
        if not self.rotating:
            return

        twist = Twist()
        twist.angular.z = 0.6
        self.cmd_pub.publish(twist)

        if time.time() - self.rotation_start > 3.0: #arbitrary time for now
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.rotating = False

    def map_callback(self, map_msg):
        """Process the map message for movement planning."""
        self.map = map_utils.Map(map_msg)

    def point_callback(self, point_msg):
        """Process incoming navigation points."""

        x = point_msg.point.x
        y = point_msg.point.y
        theta = point_msg.point.z # Assuming z holds orientation in radians
        new_point = Point(self.priority_point_value, x, y, 0.0, theta)
        self.points.push(new_point)
        self.navigate_to_target(new_point)  # Immediately navigate to new priority point
        self.get_logger().info(f"Received new navigation point at ({x}, {y}, {theta}) with priority value {self.priority_point_value}")


    def goal_checker_callback(self):
        # if self.start_time is None:
        #     self.start_time = time.time()
        # elif (self.returning is False and self.position is not None and self.speed is not None):
        #     # Reccomended by Prof. Molloy: Distance w/ extra
        #     # When to return: time left < distance / speed

        #     if self.max_time - (time.time() - self.start_time) < math.hypot(self.start_position.x - self.position.x, self.start_position.y - self.position.y) / self.speed:
        #         self.returning = True
        #         self.ac.destroy()
        #         create_nav_goal(self, self.start_position, 0) # Assumes starting angle is irrelevant.


        if self.returning is True:
            return

        """Periodically check in on the progress of navigation."""
        if self.goal_future is None:
            return  # No goal has been sent yet.

        if self.goal_future is None:
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.get_logger().info("SERVER HAS ACKNOWLEDGED CANCELLATION")
                # self.ac.destroy()
                self.path.pop()
                self.cancel_future = None
                self.future_event.set_result(True)
        else:

            if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS. EXITING!")
                # if self.goal_future.value == 1000: #check!
                #     self.latest_victim_pose = self.goal.pose
                #     self.at_victim = True
                #     self.rotating = True
                #     self.rotation_start = time.time()
                # else:
                #     self.at_victim = False
                #     self.rotating = False
                popped_point = self.path.peek()
                self.get_logger().info(f"Goal succeeded, popping path point ({popped_point.x}, {popped_point.y})")
                self.path.pop()


                # self.ac.destroy()



            if self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")
                if self.path.is_empty is False:
                    self.path.pop()
                # self.ac.destroy()

            elif time.time() - self.start_time > self.timeout:
                self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
                self.cancel_future = self.goal_future.result().cancel_goal_async()


    def fill_path_and_points_callback(self):
        """Periodically check and add new points."""
        if self.map is None: # Cannot plan without a map
            self.get_logger().info("No map available for path planning.")
            return

        if self.points.is_empty(): # No points currently available
            self.get_logger().info("Generating new points.")
            temp_points = self.find_random_valid_points(number_of_nodes=10)
            time.sleep(1.5) # Give the bot some time to generate points
            for p in temp_points:
                self.get_logger().info(f"New point added to points: ({p.x}, {p.y})")

            return

        if self.path.is_empty(): # No current path, need to plan
            self.path.push(self.points.pop())
            self.navigate_to_target(self.path.peek())
            self.get_logger().info("Selecting new target at ({}, {})".format(self.path.peek().x, self.path.peek().y))
            return



    def check_for_detours(self):
        """Check if there are better points to navigate to en route to current target."""

        if not self.points:
            return  # Not enough points to check for detours
        if self.path.is_empty is False:
            target_point = self.points.peek()


        # current_position_x = 0 # Placeholder for getting current position
        # current_position_y = 0 # change
        # for point in self.points:
        #     if point.equals(target_point):
        #         continue  # Skip the target point itself
        #     distance_to_next = math.hypot(current_position_x - point.x, current_position_y - point.y)
        #     distance_to_target = math.hypot(current_position_x - target_point.x, current_position_y - target_point.y)


        #     if (distance_to_next < distance_to_target / 2.0) and (point.value / self.start_time > 5):  # Detour threshold and value threshold change to parameters
        #         self.get_logger().info(f"Detour detected to point ({point.x}, {point.y})")
        #         self.path.push(point) # change to stack stuff
        #         self.navigate_to_target(point)
        #         break

        pass


    def speed_and_distance_callback(self, msg):
        # self.get_logger().info(f"Message: {msg.pose.pose.position.x}") # If ya ever want to know how to dissect the amcl_pose message
        if self.position is None:
            self.position = msg.pose.pose.position
            self.start_position = Point(x = msg.pose.pose.position.x, y = msg.pose.pose.position.y)
        self.distance_travelled = self.distance_travelled + math.hypot(self.position.x - msg.pose.pose.position.x, self.position.y - msg.pose.pose.position.y)
        if (self.distance_travelled != 0 and self.start_time is not None):
            self.speed = (self.distance_travelled) / (time.time() - self.start_time)
        self.position = Point(x = msg.pose.pose.position.x, y = msg.pose.pose.position.y)

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
            x = np.random.randint(0, self.map.width) # change
            y = np.random.randint(0, self.map.height)
            val = self.map.get_cell(x, y)
            if val == 0:
                points.append(Point(value=0, x=x, y=y))

        self.node_value(points)
        for p in points:
            self.points.push(p)

        return points



    def node_value(self, points, r=4):
        """Assign values to points based on free space around them and spacing."""

        if not points or self.map is None:
            return

        # Reset values before scoring
        for p in points:
            p.value = 0.0

        for point in points:
            # Clamp search window to map bounds
            x_min = max(0, point.x - r)
            x_max = min(50, point.x + r)
            y_min = max(0, point.y - r)
            y_max = min(50 - 1, point.y + r)


            count = 0
            for i in range(x_min, x_max + 1):
                for j in range(y_min, y_max + 1):
                    if self.map.get_cell(i, j) == 0:
                        count += 1

            point.value = count


        # Finally, sort points by value
        points.sort(reverse=True)


    # -------------------- path planning --------------------

    def create_nav_goal(self, point):
        """Create a NavigateToPose Goal from a Point and orientation theta (radians)."""
        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(point.x)
        goal.pose.pose.position.y = float(point.y)
        goal.pose.pose.position.z = 0.0


        # We need to convert theta to a quaternion....
        quaternion = tf_transformations.quaternion_from_euler(0, 0, point.theta, 'rxyz')
        goal.pose.pose.orientation.x = quaternion[0]
        goal.pose.pose.orientation.y = quaternion[1]
        goal.pose.pose.orientation.z = quaternion[2]
        goal.pose.pose.orientation.w = quaternion[3]

        return goal


    def navigate_to_target(self, target_point):
        # Cancel existing goal if present

        self.at_victim = False
        self.rotating = False

        # if self.goal_future is not None:
        #     self.get_logger().info("Cancelling existing goal before sending new one.")
        #     self.cancel_future = self.goal_future.result().cancel_goal_async()

        # Create and send new goal

        self.goal = self.create_nav_goal(target_point)
        self.ac.wait_for_server()
        self.start_time = time.time()
        self.goal_future = self.ac.send_goal_async(self.goal)

        self.future_event = Future()




def main():
    rclpy.init()
    temp_node = TempNode()
    rclpy.spin(temp_node)
    temp_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()




# Add in parameters
# Check for syntax and logic errors
# change main to destroy on future complete
