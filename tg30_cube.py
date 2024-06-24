import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time
import numpy as np
import os
os.environ["MAVLINK20"] = "2"
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil


# vehicle = connect("/dev/cube_telem2", baud=921600, wait_ready=False)
vehicle = connect("/dev/ttyAMA0", baud=921600, wait_ready=False)

##### Obstacle distance params #####
# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE

angle_shifted = 180

min_distance = 5
max_distance = 1500
distance_array_length = 72
angle_offset = -angle_shifted
increment_f = (angle_shifted*2)/72

def send_obstacle_distance_message():
	global current_time_us, distances, min_distance, max_distance, angle_offset, increment_f

	if angle_offset is None or increment_f is None:
		print("Please call set_obstacle_distance_params before continue")
	else:
		# print("Send msg")
		msg = vehicle.message_factory.obstacle_distance_encode(
			current_time_us,    # us Timestamp (UNIX time or time since system boot)
			0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
			distances,          # distances,    uint16_t[72],   cm
			0,                  # increment,    uint8_t,        deg
			min_distance,	    # min_distance, uint16_t,       cm
			max_distance,       # max_distance, uint16_t,       cm
			increment_f,	    # increment_f,  float,          deg
			angle_offset,       # angle_offset, float,          deg
			12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
		)

		vehicle.send_mavlink(msg)
		vehicle.flush()


class TG30CubeInterface(Node):

	def __init__(self):

		super().__init__('tg3_cube_interface')

		### Laserscan ###
		self.front_min_scan_ang = -angle_shifted
		self.front_max_scan_ang = angle_shifted
		self.got_scan = False
		self.got_lidar_data = False
		self.distance_array = []

		self.last_send_stamp = time.time()
		self.freq_send = 5.0
		self.period_send = 1.0/self.freq_send

		qos_policy = rclpy.qos.QoSProfile(
			reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, 
			history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=10)
		self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile=qos_policy)

		self.front_scan_pub = self.create_publisher(LaserScan, '/front_scan', qos_profile=qos_policy)

		self.timer = self.create_timer(0.05, self.timer_callback)

	def scan_callback(self, msg):

		if not self.got_scan:
			self.scan_length = len(msg.ranges)
			self.got_scan = True
			self.update_lidar_index()
			print("scan length is {}".format(self.scan_length))
		else:
			# self.distance_array = msg.ranges[self.front_stop_first_idx:self.front_stop_last_idx]
			self.distance_array = msg.ranges
			self.got_lidar_data = True

			# front_stop_scan = LaserScan()
			# front_stop_scan.header.stamp = self.get_clock().now().to_msg()
			# front_stop_scan.header.frame_id = "laser_frame"
			# front_stop_scan.time_increment = msg.time_increment
			# front_stop_scan.angle_increment = msg.angle_increment
			# front_stop_scan.angle_min = np.radians(self.front_min_scan_ang)
			# front_stop_scan.angle_max = np.radians(self.front_max_scan_ang)
			# front_stop_scan.scan_time = msg.scan_time
			# front_stop_scan.range_min = msg.range_min
			# front_stop_scan.range_max = msg.range_max
			# front_stop_scan.ranges = msg.ranges[self.front_stop_first_idx:self.front_stop_last_idx]
			# front_stop_scan.intensities = msg.intensities[self.front_stop_first_idx:self.front_stop_last_idx]

			# print(len(front_stop_scan.ranges))

			# self.front_scan_pub.publish(front_stop_scan)

	def update_lidar_index(self):

		## front stop
		self.front_stop_first_idx = self.lidarAng_to_lidarIdx(self.front_min_scan_ang) 
		self.front_stop_last_idx = self.lidarAng_to_lidarIdx(self.front_max_scan_ang)

	def lidarAng_to_lidarIdx(self, ang):
		return int(self.map_with_limit(ang, -180.0, 180.0, 0.0, (self.scan_length-1)))

	def map_with_limit(self, val, in_min, in_max, out_min, out_max):

		# out = ((val - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min
		## in_min must be the minimum input 
		## in_max must be the maximum input

		## out_min is supposed to map with in_min value
		## out_max is sipposed to map with in_max value
		## out_min can be less/more than out_max, doesn't matter


		m = (out_max - out_min)/(in_max - in_min)
		out = m*(val - in_min) + out_min

		if out_min > out_max:
			if out > out_min:
				out = out_min
			elif out < out_max:
				out = out_max
			else:
				pass
		elif out_max > out_min:
			if out > out_max:
				out = out_max
			elif out < out_min:
				out = out_min
			else:
				pass
		else:
			pass

		# print(m, val, in_min, in_max, out_min, out_max)

		return out

	# def every_nth(array, nth):
		# ## https://www.w3resource.com/python-exercises/list/python-data-type-list-exercise-269.php
		# ## Use list slicing to return elements starting from the (nth-1) index, with a step of 'nth'.
		# return array[nth - 1::nth]



	def timer_callback(self):

		current_time_us = int(round(time.time() * 1000000))

		if self.got_lidar_data:

			dist_array = np.copy(self.distance_array)
			dist_array = dist_array[::-1]*100

			array_len = len(dist_array)
			take_every = int(array_len/72)
			# print(take_every)
			dist_array_small = np.round(dist_array[0::take_every]).astype(int)
			dist_array_small[dist_array_small == 0] = 65535

			distances = dist_array_small.tolist()
			if len(distances) > 72:
				dist_len = len(distances)
				remove_idx = dist_len - 72
				distances = distances[:-remove_idx]
				# print("remove idx", remove_idx)

			# print(distances)
			# print(len(distances))

			if (time.time() - self.last_send_stamp) > self.period_send:
			
				msg = vehicle.message_factory.obstacle_distance_encode(
					current_time_us,    # us Timestamp (UNIX time or time since system boot)
					0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
					distances,          # distances,    uint16_t[72],   cm
					0,                  # increment,    uint8_t,        deg
					min_distance,	    # min_distance, uint16_t,       cm
					max_distance,       # max_distance, uint16_t,       cm
					increment_f,	    # increment_f,  float,          deg
					angle_offset,       # angle_offset, float,          deg
					12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
				)

				vehicle.send_mavlink(msg)
				vehicle.flush()

				self.last_send_stamp = time.time()



def main(args=None):

	rclpy.init(args=args)
	node = TG30CubeInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()