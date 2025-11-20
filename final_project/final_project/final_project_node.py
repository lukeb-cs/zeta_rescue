# Node setup
import rclpy
import rclpy.node

# Other imports
import cv2
from cv_bridge import CvBridge
import numpy as np
import random

# Message types
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class ZetaNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('zeta')
        self.bridge = CvBridge()

        self.subscription_img = self.create_subscription(Image, '/oakd/rgb/preview/image_raw', self.image_interp_callback, 10)
        self.subscription_aruco = self.create_subscription(Image, '/aruco_poses', self.aruco_pose_callback, 10)

        self.thrust_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        self.timer = self.create_timer(1.0, self.change_motion_callback)

    def image_interp_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pumpkin_location = self.image_helper(img)
        if pumpkin_location[0] == -1 and pumpkin_location[1] == -1:
            self.get_logger().info(f"No pumpkin found")
        else:
            self.get_logger().info("Orange object located in front of robot")
            if pumpkin_location[1] < 100: # checking if the orange pixel is located at the top of the image
                self.get_logger().info("Orange object is close to the robot")

    def aruco_pose_callback(self, poses):
        self.get_logger().info("Aruco is running the callback")
        self.get_logger().info(f"Poses: {poses}")

    def image_helper(self, img):
        r = img[:, :, 2].astype(np.float32)
        g = img[:, :, 1].astype(np.float32)
        b = img[:, :, 0].astype(np.float32)

        orange_score = r - np.abs(g - 0.6 * r) - b * 0.5
        minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(orange_score)

        if maxVal < 120: # 140:            # pumpkin orange generally sits around maxVal = 180
            return (-1, -1)         # lesser oranges have a lower number, around maxVal = 100
        return maxLoc

    def change_motion_callback(self):
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
