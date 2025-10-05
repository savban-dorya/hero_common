#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

# List of 10 robots
DEFAULT_ROBOTS = ['hero_0', 'hero_1', 'hero_2', 'hero_3', 'hero_4', 
                  'hero_5', 'hero_6', 'hero_7']

class MultiTeleop:
    def __init__(self, robot_names):
        self.robot_names = robot_names
        self.pwm_value = 2300  # default if no joystick

        # One publisher per robot
        self.cmd_publishers = {
            name: rospy.Publisher(f"{name}/cmd_motor", Float32, queue_size=10)
            for name in robot_names
        }

        # Subscribe to joystick
        rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        # Update PWM from joystick
        self.pwm_value = msg.axes[1]  # scale if needed

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            for pub in self.cmd_publishers.values():
                pub.publish(Float32(self.pwm_value))
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("multi_teleop")
    robots = rospy.get_param("~robots", DEFAULT_ROBOTS)
    teleop = MultiTeleop(robots)
    teleop.run()
