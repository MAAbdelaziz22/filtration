import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
class PIDController (Node):
    def __init__ (self):
        super().__init__('pid')
        self.target_speed =  60
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.05
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = time.time()

        self.cmd_pub = self.create_publisher( Float32 , '/cmd_vel' , 20 )

        self.subscription =  self.create_subscription( Float32 , '/current_speed' , self.speed_callback , 20)

    def speed_callback (self ,  msg):
            current_speed = msg.data
            error = self.target_speed - current_speed
            current_time = time.time()
            dt = current_time - self.last_time
            dt = 0.05 
            p = self.kp * error 
            i = self.ki * self.error_sum
            d = self.kd * (error - self.last_error) / dt
            controller = p + i + d
            controller = max(min(controller,1.0),-1.0)
            cmd_msg = Float32()
            cmd_msg.data = controller
            self.cmd_pub.publish(cmd_msg)
            self.last_error = error
            self.last_time = current_time
            self.get_logger().info(
            f"Target: {self.target_speed:.2f}, Current: {current_speed:.2f}, Control: {controller:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
