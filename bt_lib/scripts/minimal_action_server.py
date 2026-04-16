import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from bt_lib.action import Task

class Server(Node):
    def __init__(self):
        super().__init__('server')

        self.server = ActionServer(
            self,
            Task,
            'my_task',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal")

        goal_handle.succeed()

        result = Task.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = Server()
    rclpy.spin(node)
    rclpy.shutdown()