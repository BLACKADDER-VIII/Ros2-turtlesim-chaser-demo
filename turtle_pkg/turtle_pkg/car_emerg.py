import rclpy
import random
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SetPen

class TurtleEmergController(Node):
	def __init__(self):
		super().__init__('emergency_car')
		self.pos = None
		self.goal_x = 1.0*random.randint(7, 11)
		self.goal_y = 1.0*random.randint(7, 11)
		self.publisher = self.create_publisher(
		Twist, '/turtle3/cmd_vel', 10
		)
		self.sub = self.create_subscription(
		Pose,
		'/turtle3/pose',
		self.pos_callback,
		10
		)
		self.client = self.create_client(Spawn, '/spawn')

		while not self.client.wait_for_service(timeout_sec = 1.0):
			self.get_logger().info('Waiting for /spawn...')
		self.get_logger().info(f"Goal coordinates: {self.goal_x}, {self.goal_y}")
		request = Spawn.Request()
		request.x = 1.0*random.randint(0, 4)
		request.y = 1.0*random.randint(0, 4)
		self.get_logger().info(f"Spawn coordinates: {request.x}, {request.y}")
		request.theta = 0.0
		request.name = 'turtle3'
		future = self.client.call_async(request)
		future.add_done_callback(self.spawn_done)

	def spawn_done(self, future):
		client_w = self.create_client(SetPen, '/turtle3/set_pen')
		while not client_w.wait_for_service(timeout_sec = 1.0):
			self.get_logger().info("Waiting for set pen")

		req = SetPen.Request()
		req.r = 255
		req.b = 0
		req.g = 0
		req.off = False
		req.width = 5
		future = client_w.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		self.time = self.create_timer(0.1, self.move)

	def pos_callback(self, msg):
		self.pos = msg

	def move(self):
		msg = Twist()
		dx = self.goal_x - self.pos.x
		dy = self.goal_y - self.pos.y 
		angle = math.atan2(dy, dx) - self.pos.theta
		hpot = math.hypot(dx, dy)
		msg.angular.z = angle*4.0 if hpot > 0.1 else 0.0
		msg.linear.x = 1.0 if hpot > 0.1 else 0.0
		if msg.linear.x == 0:
			self.get_logger().info("Reached destination. Shutting down...")
			rclpy.shutdown()
		else:
			self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = TurtleEmergController()
	rclpy.spin(node)
	node.destroy_node()
if __name__ == 'main':
	main()






