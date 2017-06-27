#!/usr/bin/env python
import rospy
import roslib; #roslib.load_manifest('pico_base_controller')
import serial
import emc_system
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from emc_system.msg import motorCurrent
from math import sin, cos
import re
#from tf.broadcaster import TransformBroadcaster
import tf
from tf.transformations import vector_norm
from tf.transformations import numpy as np

class OmniBase():
	regex = re.compile('(-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+)')
	ser = serial.Serial('/dev/ttyArduino0', 115200)
	max_speed_linear  = 0.5
	max_speed_angular = 1.2
	max_speed_error = 0.01 # speed may be 1% greater than the maximum

	def velocityCallback(self, vel):
		self.vel = vel

	def sendReference(self, vel):
		# cap linear speed
		l = vel.linear
		v = np.array([l.x, l.y, l.z])
		speed = vector_norm(v)
		if speed > self.max_speed_linear * (1 + self.max_speed_error):
			rospy.logwarn('maximum linear speed exceeded with %f m/s, capping to %f m/s' % (speed, self.max_speed_linear))
			v = v / speed * self.max_speed_linear

		# cap angular speed
		a = vel.angular.z
		if np.abs(a) > self.max_speed_angular * (1+ self.max_speed_error):
			rospy.logwarn('maximum angular speed exceeded with %f rad/s, capping to %f rad/s' % (a, self.max_speed_angular))
			a = a / abs(a) * self.max_speed_angular

		self.ser.write("<%f,%f,%f>\n"%(v[0], v[1], a))

	def __init__(self):
		rospy.init_node("omni_base");

		rospy.Subscriber("cmd_vel", Twist, self.velocityCallback)
		r = rospy.Rate(200.0) # 200hz

		odomPub = rospy.Publisher('/pico/odom', Odometry, queue_size=0) #queue_size: This is the size of the outgoing message queue used for asynchronous publishing. Please find more detailed information below in the section "Choosing a good queue_size" 
		currentPub = rospy.Publisher('/pico/current', motorCurrent, queue_size=0) #queue_size: This
		odomBroadcaster = tf.TransformBroadcaster()
		#print "vel callback"
		now = rospy.Time.now()
		x = 0.0
		y = 0.0
		th = 0.0
		x_robot_last = 0.0
		x_robot = 0.0
		y_robot_last = 0.0
		y_robot = 0.0
		th_robot_last = 0.0
		th_robot = 0.0

		self.vel = None

		ref_freq = 10
		ref_dt = rospy.Duration(1.0 / ref_freq)

		ref_t = rospy.Time.now()		

		while not rospy.is_shutdown():
			just = now
			now = rospy.Time.now()
			dt = (now - just).to_sec()

			if self.vel and ref_t + ref_dt < now:
				self.sendReference(self.vel)
				self.vel = None
				ref_t = now

			#print dt
			s = self.ser.readline()
			result = self.regex.match(s)
			if result and dt > 0.0:
				x_robot_last = x_robot
				y_robot_last = y_robot
				th_robot_last = th_robot
				
				x_robot = float(result.group(1))
				y_robot = float(result.group(2))
				th_robot = float(result.group(3))

				I_rightfront = float(result.group(4))
				I_back       = float(result.group(5))
				I_leftfront  = float(result.group(6))
				
				vx = (x_robot - x_robot_last)/dt
				vy = (y_robot - y_robot_last)/dt
				vth = (th_robot - th_robot_last)/dt
				
				dx = (vx * cos(th) - vy * sin(th)) * dt;
				dy = (vx * sin(th) + vy * cos(th)) * dt;
				dth = vth * dt;

				x += dx;
				y += dy;
				th += dth;

				q = tf.transformations.quaternion_from_euler(0.0, 0.0, th)
				quaternion = Quaternion(q[0], q[1], q[2], q[3])
			
				# Create the odometry transform frame broadcaster.
				odomBroadcaster.sendTransform(
					(x, y, 0.0), 
					(q[0], q[1], q[2], q[3]),
					now,
					"/pico/base_link",
					"/pico/odom"
					)
			
				odom = Odometry()
				odom.header.frame_id = "/pico/odom"
				odom.child_frame_id = "/pico/base_link"
				odom.header.stamp = now
				odom.pose.pose.position.x = x
				odom.pose.pose.position.y = y
				odom.pose.pose.position.z = 0.0
				odom.pose.pose.orientation = quaternion
				odom.twist.twist.linear.x = vx
				odom.twist.twist.linear.y = vy
				odom.twist.twist.angular.z = vth
				#print 'odom: %f %f %f' % (x, y, th)
				#print odom
				odomPub.publish(odom)

				currentMsg = motorCurrent()
				odom.header.frame_id = "/pico/current"
				currentMsg.header.stamp = now
				currentMsg.I_rightfront = I_rightfront
				currentMsg.I_back = I_back
				currentMsg.I_leftfront = I_leftfront
				currentPub.publish(currentMsg)
			r.sleep()

if __name__ == '__main__':
	try:
		OmniBase()
	except rospy.ROSInterruptException: pass
