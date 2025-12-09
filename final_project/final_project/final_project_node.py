# Luke Brenningmeyer

# Node setup
import rclpy
import rclpy.node

# Other imports
import numpy as np
import math
# -- No longer using these imports --
import cv2
from cv_bridge import CvBridge
import random

# Message types
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
# -- No longer using these imports --
from geometry_msgs.msg import TwistStamped, Twist, Vector3
from sensor_msgs.msg import Image

# transformations
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class Person:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ZetaNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('zeta')

        # -- Orange detection not currently in use --
        # self.bridge = CvBridge()
        # self.subscription_img = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_interp_callback, 10)

        # -- Random navigation used earlier, no longer needed --
        # self.thrust_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        # self.timer = self.create_timer(1.0, self.change_motion_callback)

        self.subscription_aruco = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_pose_callback, 10)
        self.subscription_pos = self.create_subscription(Odometry, "/odom", self.pos_callback, 10)
        self.nav_point_pub = self.create_publisher(PointStamped, 'nav_point', 10)

        self.scanning_code = False

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.x = 0.0
        self.y = 0.0

        self.person_list = []

    # def image_interp_callback(self, msg): # may not be used in final version
    #     img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     pumpkin_location = self.image_helper(img)
    #     if pumpkin_location[0] == -1 and pumpkin_location[1] == -1:
    #         self.get_logger().info(f"No pumpkin found")
    #     else:
    #         self.get_logger().info("Orange object located in front of robot")
    #         if pumpkin_location[1] < 100: # checking if the orange pixel is located at the top of the image
    #             self.get_logger().info("Orange object is close to the robot")

    # def image_helper(self, img):
    #     r = img[:, :, 2].astype(np.float32)
    #     g = img[:, :, 1].astype(np.float32)
    #     b = img[:, :, 0].astype(np.float32)

    #     orange_score = r - np.abs(g - 0.6 * r) - b * 0.5
    #     minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(orange_score)

    #     if maxVal < 120: # 140:            # pumpkin orange generally sits around maxVal = 180
    #         return (-1, -1)         # lesser oranges have a lower number, around maxVal = 100
    #     return maxLoc

    def pos_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def aruco_pose_callback(self, poses):
        if self.scanning_code:
            return
        self.get_logger().info("Aruco is running the callback")
        self.get_logger().info(f"Poses: {poses}")

        p = poses.poses[0]
        p1 = PoseStamped()
        p1.pose.position.x = p.position.x
        p1.pose.position.y = p.position.y
        p1.pose.position.z = p.position.z
        p1.pose.orientation.x = p.orientation.x
        p1.pose.orientation.y = p.orientation.y
        p1.pose.orientation.z = p.orientation.z
        p1.pose.orientation.w = p.orientation.w

        try:
            p2 = self.buffer.transform(p1, "map")
            self.get_logger().info('Publishing: "%s"' % p2)
        except Exception as e:
            self.get_logger().warn(str(e))

        self.scanning_code = True

        x_loc = p2.pose.position.x
        y_loc = p2.pose.position.y

        q = p2.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        person = Person(x_loc, y_loc)
        for p in self.person_list:
            if abs(p.x - person.x) < 0.5 or abs(p.y - person.y) < 0.5:
                self.scanning_code = False
                return
        self.person_list.append(person)

        change = 0.5 # 0.5 meters

        front_x = x_loc + change * math.sin(yaw)
        front_y = y_loc + change * math.cos(yaw)
        theta = yaw + math.pi

        msg = PointStamped()

        msg.point.x = front_x
        msg.point.y = front_y
        msg.point.z = theta
        self.nav_point_pub.publish(msg)
        self.get_logger().info(f"Published: x={msg.point.x}, y={msg.point.y}, theta={msg.point.z}")

        self.scanning_code = False

    # def change_motion_callback(self):
    #     if self.scanning_code:
    #         return
    #     dumb_twister = Twist()
    #     linear_vec = Vector3()
    #     linear_vec.x = 0.2
    #     dumb_twister.linear = linear_vec
    #     angular_vec = Vector3()
    #     angular_vec.z = random.uniform(-1.0, 1.0)
    #     dumb_twister.angular = angular_vec
    #     twister = TwistStamped()
    #     twister.twist = dumb_twister

    #     self.thrust_pub.publish(twister)

    #     self.get_logger().info("We have changed our motion")

def main():
    rclpy.init()
    zeta_node = ZetaNode()
    rclpy.spin(zeta_node)
    zeta_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
