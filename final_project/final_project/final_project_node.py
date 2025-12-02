# Node setup
import rclpy
import rclpy.node

# Other imports
import cv2
from cv_bridge import CvBridge
import numpy as np
import random
import math

# Message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class Person:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ZetaNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('zeta')
        self.bridge = CvBridge()

        self.subscription_img = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_interp_callback, 10)
        self.subscription_aruco = self.create_subscription(Image, '/aruco_poses', self.aruco_pose_callback, 10)
        self.subscription_pos = self.create_subscription(Odometry, "/odom", self.pos_callback, 10)

        self.thrust_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer = self.create_timer(1.0, self.change_motion_callback)

        self.scanning_code = False

        self.x = 0.0
        self.y = 0.0

        self.person_list = []

    def pos_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def image_interp_callback(self, msg): # may not be used in final version
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pumpkin_location = self.image_helper(img)
        if pumpkin_location[0] == -1 and pumpkin_location[1] == -1:
            self.get_logger().info(f"No pumpkin found")
        else:
            self.get_logger().info("Orange object located in front of robot")
            if pumpkin_location[1] < 100: # checking if the orange pixel is located at the top of the image
                self.get_logger().info("Orange object is close to the robot")

    def image_helper(self, img):
        r = img[:, :, 2].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        b = img[:, :, 0].astype(np.float32)

        orange_score = r - np.abs(g - 0.6 * r) - b * 0.5
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(orange_score)

        if maxVal < 120: # 140:            # pumpkin orange generally sits around maxVal = 180
            return (-1, -1)         # lesser oranges have a lower number, around maxVal = 100
        return maxLoc

    def aruco_pose_callback(self, poses):
        if self.scanning_code:
            return
        self.get_logger().info("Aruco is running the callback")
        self.get_logger().info(f"Poses: {poses}")

        angle = 0 # placeholder
        distance = 1 # placeholder
        scanning_code = True

        # using the distance and angle, with trig functions, get the x and y position
        x_dist = length * math.cos(math.radians(angle_deg))
        y_dist = length * math.sin(math.radians(angle_deg))
        x_loc = x_dist + self.x
        y_loc = y_dist + self.y
        
        # normalize the coords to remove small variance in points
            # if the value is close to that of something we've already scanned, set scanning_code = False and return
        person = Person(x_loc, y_loc)
        for p in self.person_list:
            if abs(p.x - person.x) < 0.5 or abs(p.y - person.y) < 0.5:
                self.scanning_code = False
                return
        self.person_list.append(person)

        # store the robot's starting location
        stating_x = self.x
        starting_y = self.y

        # make sure the robot rotates to travel parallel to the person

        # decide if you can move, so that the robot is faceing the person
            # either move along the x axis until difference in x is zero (check if possible)
            # or you can move along the y-axis until the y is the same as the person

        # turn towards the person once the x or y is the same
        # move within a meter of the person and take a picture where the name is visible
            # (optional) you could using orange detection to prevent the bot from getting too close, but that may not be needed
            # if we are on the side of the robot where there is no code, move around the person until the code is found
            # remember that is the aruco is found, its on one of the sides facing the robot, so we only need to check those sides
        # retrace steps, so that you end back on the random path
        
        self.scanning_code = False

    def change_motion_callback(self):
        if self.scanning_code:
            return
        dumb_twister = Twist()
        linear_vec = Vector3()
        linear_vec.x = 0.2
        dumb_twister.linear = linear_vec
        angular_vec = Vector3()
        angular_vec.z = random.uniform(-1.0, 1.0)
        dumb_twister.angular = angular_vec
        twister = TwistStamped()
        twister.twist = dumb_twister
        
        self.thrust_pub.publish(twister)

        self.get_logger().info("We have changed our motion")

def main():
    rclpy.init()
    zeta_node = ZetaNode()
    rclpy.spin(zeta_node)
    zeta_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
