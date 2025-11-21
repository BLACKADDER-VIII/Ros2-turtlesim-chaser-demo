import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
import random
import math
from turtlesim.msg import Pose
from turtlesim.srv import SetPen

class TurtleBehindController(Node):
	def __init__(self):
		super().__init__('car_behind')
		self.car_ahead_pos = None
		self.pos = None
		self.emerg_pos = None
		self.publisher = None
		self.sub_car_ahead = self.create_subscription(
		Pose,
		'/turtle1/pose',
		self.pose_callback_ahead,
		10
		)
		self.sub_self = self.create_subscription(
		Pose,
		'/turtle2/pose',
		self.pose_callback_self,
		10
		)
		self.sub_emerg = self.create_subscription(
		Pose,
		'/turtle3/pose',
		self.pose_callback_emerg,
		10
		)
		self.client = self.create_client(Spawn, '/spawn')

		while not self.client.wait_for_service(timeout_sec = 1.0):
			self.get_logger().info('Waiting for /spawn...')
		
		request = Spawn.Request()
		request.x = 1.0*random.randint(0, 11)
		request.y = 1.0*random.randint(0, 11)
		request.theta = 0.0
		request.name = 'turtle2'
		future = self.client.call_async(request)
		future.add_done_callback(self.spawn_done)

	def pose_callback_ahead(self, msg):
		self.car_ahead_pos = msg

	def pose_callback_self(self, msg):
		self.pos = msg
		
	def pose_callback_emerg(self, msg):
		self.emerg_pos = msg

	def spawn_done(self, future):
		response = future.result()
		topic_name = f"/{response.name}/cmd_vel"
		self.publisher = self.create_publisher(Twist, topic_name, 10)
		self.timer = self.create_timer(0.1, self.move)
		self.client_w = self.create_client(SetPen, '/turtle2/set_pen')
		while not self.client_w.wait_for_service(timeout_sec=1.0):
			self.get_logger().info("Waiting for set pen")

		req = SetPen.Request()
		req.r = 0
		req.g = 255
		req.b = 0
		req.off = False
		req.width = 5
		future = self.client_w.call_async(req)
		rclpy.spin_until_future_complete(self, future)

	def move(self):
		msg = Twist()
		if self.emerg_pos != None:
			dex = self.emerg_pos.x - self.pos.x
			dey = self.emerg_pos.y - self.pos.y
			de_angle = self.emerg_pos.theta
		# Check if emergency vehicle about to colide, change direction
			angle_to_emerg = math.atan2(dey, dex) - self.pos.theta
			angle_to_move = angle_to_emerg + math.pi/2
			dist_to_emerg = math.hypot(dex, dey)
			if dist_to_emerg <= 0.5:
				self.get_logger().info("Near collision")
			if dist_to_emerg < 1:
				msg.linear.x = 1.0
				msg.angular.z = angle_to_move
				self.publisher.publish(msg)
				return

		dx = self.car_ahead_pos.x - self.pos.x
		dy = self.car_ahead_pos.y - self.pos.y
		angle = 4*(math.atan2(dy, dx) - self.pos.theta)
		msg.angular.z = angle
		hpot = math.hypot(dx, dy)
		msg.linear.x = hpot*1.5 if hpot > 1 else 0.0
		if hpot <= 0.5:
			self.get_logger().info("Near collision")
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = TurtleBehindController()
	rclpy.spin(node)      
	node.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
	main()






