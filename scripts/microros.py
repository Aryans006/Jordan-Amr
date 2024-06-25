#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('bot_controller')
        
        self.right_vel = 0.0
        self.left_vel = 0.0
        self.max_pwm_val = 255
        self.min_pwm_val = 0.0
        self.circumference = 0.47
        self.motor_rpm = 30.0
        self.wheel_seperation = 0.45
        self.max_speed = (self.circumference * self.motor_rpm) / 60  
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_esp32', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_pwm(self, right_vel, left_vel):
        lspeedPWM = max(min((left_vel / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)
        rspeedPWM = max(min((right_vel / self.max_speed) * self.max_pwm_val, self.max_pwm_val), self.min_pwm_val)
        return lspeedPWM, rspeedPWM

    def pose_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        self.right_vel = linear_vel + (angular_vel * self.wheel_seperation) / 2
        self.left_vel = linear_vel - (angular_vel * self.wheel_seperation) / 2

        x, y = self.get_pwm(self.right_vel, self.left_vel)

        self.vel = Twist()
        self.vel.linear.x = float(y)
        self.vel.linear.y = float(x)

        self.publish_velocity(self.vel)

    def publish_velocity(self, vel):
        self.publisher_.publish(vel)

    def timer_callback(self):
        # No-op placeholder for the timer callback
        pass

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()

    try:
        rclpy.spin(turtle_controller)
    except KeyboardInterrupt:
        pass
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
