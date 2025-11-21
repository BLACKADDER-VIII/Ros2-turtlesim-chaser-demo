import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class TurtleAheadController(Node):
	def __init__(self):
		super().__init__("car_ahead")
		self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
		self.pos = None
		self.emerg_pos = None
		self.pos_sub = self.create_subscription(
		Pose,
		'/turtle1/pose',
		self.pose_callback_self,
		10
		)
		self.emerg_sub = self.create_subscription(
		Pose,
		'/turtle3/pose',
		self.pose_callback_emerg,
		10
		)
		req = SetPen.Request()
		req.r = 0
		req.g = 0
		req.b = 255
		req.off = False
		req.width = 5
		client = self.create_client(SetPen, '/turtle1/set_pen')
		while not client.wait_for_service(timeout_sec = 1.0):
			self.get_logger().info("Waiting for set pen...")
		future = client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		self.timer = self.create_timer(0.1, self.move)

	def move(self):
		msg = Twist()
		# Checking emergency collision first
		dist_to_col = 1e10
		if self.emerg_pos != None:
			dx = self.emerg_pos.x - self.pos.x
			dy = self.emerg_pos.y - self.pos.y
			dist_to_col = math.hypot(dx, dy)
		if dist_to_col <= 0.5:
			self.get_logger().info("Near Collision")
		if dist_to_col < 1:
			angle_to_emerg = math.atan2(dy, dx) - self.pos.theta
			angle_to_move = angle_to_emerg + math.pi/2
			msg.linear.x = 1.0
			msg.angular.z = angle_to_move
		else:
			msg.linear.x = 2.0
			msg.angular.z = 1.0
		self.publisher.publish(msg)

	def pose_callback_self(self, msg):
		self.pos = msg

	def pose_callback_emerg(self, msg):
		self.emerg_pos = msg


def main(args=None):
	rclpy.init(args=args)
	node = TurtleAheadController()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
