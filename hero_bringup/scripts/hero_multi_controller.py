#!/usr/bin/env python
"""
Skeleton centralized controller for multiple HeRo robots.

- Subscribes to per-robot sensors/topics (encoder, imu, laser, odom, etc.)
- Publishes per-robot actuation (cmd_motor, velocity cmd_vel, led)
- Basic state machine: START -> DEPLOY_CIRCLE -> EXPLORE -> RETURN -> ROTATE_CIRCLE -> DONE

This is a skeleton — replace the TODO sections with your control and mapping logic.
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# Placeholder imports for custom messages (Encoder, Motor) present in this repo
try:
    from hero_common.msg import Encoder, Motor
except Exception:
    # If those msgs aren't built in your workspace yet, we'll use placeholders
    Encoder = None
    Motor = None

import threading
import time

DEFAULT_ROBOTS = ['hero_0', 'hero_1', 'hero_2', 'hero_3', 'hero_4', 'hero_5', 'hero_6', 'hero_7', 'hero_8', 'hero_9']

class RobotState(object):
    def __init__(self, ns):
        self.ns = ns
        self.odom = None
        self.imu = None
        self.encoder = None
        self.laser = None
        self.last_seen = rospy.Time.now()

        # publishers
        self.pub_cmd_vel = rospy.Publisher(ns + '/velocity_controller/cmd_vel', Twist, queue_size=1)
        self.pub_led = rospy.Publisher(ns + '/led', ColorRGBA, queue_size=1)
        # low level motor (optional)
        if Motor is not None:
            self.pub_motor = rospy.Publisher(ns + '/cmd_motor', Motor, queue_size=1)
        else:
            self.pub_motor = None


class MultiController(object):
    def __init__(self, robot_namespaces=None):
        rospy.init_node('hero_multi_controller', anonymous=False)
        if robot_namespaces is None:
            robot_namespaces = rospy.get_param('~robot_namespaces', DEFAULT_ROBOTS)
        self.robot_namespaces = robot_namespaces
        rospy.loginfo('Controller will manage robots: %s', robot_namespaces)

        self.robots = {ns: RobotState(ns) for ns in robot_namespaces}
        self.lock = threading.Lock()

        # subscribers for each robot
        for ns in robot_namespaces:
            rospy.Subscriber(ns + '/odom', Odometry, self._make_odom_cb(ns))
            rospy.Subscriber(ns + '/imu', Imu, self._make_imu_cb(ns))
            if Encoder is not None:
                rospy.Subscriber(ns + '/encoder', Encoder, self._make_encoder_cb(ns))
            # Laser topic could be sensor_msgs/LaserScan or a custom message; we don't import here
            # rospy.Subscriber(ns + '/laser', LaserScan, self._make_laser_cb(ns))

        # global subscribers (optional)
        rospy.Subscriber('/tf', Header, self.tf_cb)  # tf has its own transport; this is placeholder
        rospy.Subscriber('/rosout', Header, self.rosout_cb)

        # State machine
        self.state = 'START'
        self.rate = rospy.Rate(5)

    def _make_odom_cb(self, ns):
        def cb(msg):
            with self.lock:
                self.robots[ns].odom = msg
                self.robots[ns].last_seen = rospy.Time.now()
        return cb

    def _make_imu_cb(self, ns):
        def cb(msg):
            with self.lock:
                self.robots[ns].imu = msg
                self.robots[ns].last_seen = rospy.Time.now()
        return cb

    def _make_encoder_cb(self, ns):
        def cb(msg):
            with self.lock:
                self.robots[ns].encoder = msg
                self.robots[ns].last_seen = rospy.Time.now()
        return cb

    def tf_cb(self, msg):
        # placeholder, TF is usually handled separately by tf listeners
        pass

    def rosout_cb(self, msg):
        # placeholder, useful for monitoring
        pass

    def publish_velocity(self, ns, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.robots[ns].pub_cmd_vel.publish(t)

    def publish_led(self, ns, r=0, g=0, b=0, a=1.0):
        c = ColorRGBA()
        c.r = r
        c.g = g
        c.b = b
        c.a = a
        self.robots[ns].pub_led.publish(c)

    def run(self):
        rospy.loginfo('Starting main loop')
        try:
            while not rospy.is_shutdown():
                if self.state == 'START':
                    rospy.loginfo_once('State: START -> DEPLOY_CIRCLE')
                    self.state = 'DEPLOY_CIRCLE'

                elif self.state == 'DEPLOY_CIRCLE':
                    # TODO: compute initial circle positions and send small movements
                    rospy.loginfo_once('Deploying robots in a circle (skeleton)')
                    for i, ns in enumerate(self.robot_namespaces):
                        # simple example: set small forward velocity to spread out
                        self.publish_velocity(ns, linear=0.05, angular=0.0)
                        self.publish_led(ns, r=0.0, g=1.0, b=0.0, a=1.0)
                    rospy.sleep(2.0)
                    # stop
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, 0.0, 0.0)
                    self.state = 'EXPLORE'

                elif self.state == 'EXPLORE':
                    # TODO: implement obstacle sensing, mapping and exploration behavior
                    rospy.loginfo_once('Exploration: moving out and sensing (skeleton)')
                    # naive random/spiral motion placeholder
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, linear=0.08, angular=0.1)
                    rospy.sleep(5.0)
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, 0.0, 0.0)
                    self.state = 'RETURN'

                elif self.state == 'RETURN':
                    rospy.loginfo_once('Returning to circle center (skeleton)')
                    # TODO: compute and navigate to the circle center
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, linear=-0.05, angular=0.0)
                    rospy.sleep(2.0)
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, 0.0, 0.0)
                    self.state = 'ROTATE_CIRCLE'

                elif self.state == 'ROTATE_CIRCLE':
                    rospy.loginfo_once('Rotate circle a bit (skeleton)')
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, linear=0.0, angular=0.4)
                    rospy.sleep(1.0)
                    for ns in self.robot_namespaces:
                        self.publish_velocity(ns, 0.0, 0.0)
                    # Loop: go back to explore until some termination condition
                    # TODO: replace with real condition
                    self.state = 'EXPLORE'

                else:
                    rospy.logwarn('Unknown state: %s', self.state)
                    self.state = 'START'

                self.rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    try:
        controller = MultiController()
        controller.run()
    except Exception as e:
        rospy.logerr('Controller failed: %s', e)
        raise
