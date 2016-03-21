#!/usr/bin/env python

import rospy	
import roslib
import tf
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist,Pose,PoseStamped
from nav_msgs.msg import Odometry

class movementController:

        def __init__(self): 
		self.get_pose = 0;
                #set up ROS subscriber and publisher
		rospy.Subscriber('vision/detection', Odometry, self.handleRotate)
		#rospy.Subscriber('test/odom',Odometry, self.handleRotate) #test
		rospy.Subscriber('RosAria/pose', Odometry, self.poseCallback)
		#rospy.Subscriber('RosAria/motors_state', Bool, self.handleTest) #test
		
		self.move_started_pub = rospy.Publisher('movement/started',Bool, queue_size=10)
		self.vel_pub =  rospy.Publisher('RosAria/cmd_vel', Twist, queue_size=10)
		self.move_pub = rospy.Publisher('movement/completed',Bool, queue_size=10)
		#self.test_pub = rospy.Publisher('test/odom',Odometry, queue_size=10) #test
		rate = rospy.Rate(10.0) #10hz
		rospy.spin()			

	#gets the current position of the robot
	def poseCallback(self,data):
		self.current_x=data.pose.pose.position.x;
    		self.current_y=data.pose.pose.position.y;

    		quaternion = (
        		data.pose.pose.orientation.x,
        		data.pose.pose.orientation.y,
        		data.pose.pose.orientation.z,
        		data.pose.pose.orientation.w)
    		euler = tf.transformations.euler_from_quaternion(quaternion)

    		#self.current_z = euler[2]/3.14*180;  #roll = euler[0],pitch = euler[1],yaw = euler[2]
		self.current_z = euler[2]; #in rad
		self.get_pose = 1;

		#print("x = %f y= %f z= %f" % (self.current_x, self.current_y, self.current_z))
		#updated every 0.1s
		
	def handleTest(self,msg): #test
		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.pose.pose.position.x = 0.0
		odom.pose.pose.position.y = 0.0
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.x = 0.0
		odom.pose.pose.orientation.y = 0.0
		odom.pose.pose.orientation.z = 0.4
   		odom.pose.pose.orientation.w = 0.6
		while not rospy.is_shutdown():
                        self.test_pub.publish(odom)
	
	def handleRotate(self,data):
		#wait for pose info from robot
		while self.get_pose == 0:
		  pass
		print "Got message from AN"
		twist = Twist()
		count = 0;
		move_time = 2.0;
		clock_speed = 0.5;
		rate = rospy.Rate(clock_speed)
		
		quaternion = (
                        data.pose.pose.orientation.x,
                        data.pose.pose.orientation.y,
                        data.pose.pose.orientation.z,
                        data.pose.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
		target_z = euler[2];

		dz = - target_z+0.2 #self.current_z is in rad
		
		print ("dz =%f"%dz)

		if abs(dz) > 0.07  and abs(dz) < 0.87:
			self.move_started_pub.publish(True)
		while not rospy.is_shutdown() and abs(dz) > 0.07  and abs(dz) < 0.87 and count < move_time/clock_speed + 1:
			if count == 0 or count ==1:
			 	#twist.angular.z = -1*3.14/(move_time/clock_speed)/4; #test: rotate 90 clockwise then stop
				twist.angular.z = -dz/2/int(move_time/clock_speed)
				self.vel_pub.publish(twist)
			count += 1;
			#print ("dz = %f z = %f current_z = %f" %(dz, twist.angular.z, self.current_z)) #test
 			#print ("dz = %f current_z =%f" %(dz,self.current_z))
			rate.sleep()
		
		self.move_pub.publish(True)
		
		#make sure the robot stops
		for i in range(0,2):		
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
			self.vel_pub.publish(twist)
	
	def handleMovement(self,msg):
		#x = target_x-current_x
		while not rospy.is_shutdown():
			twist = Twist()
			twist.linear.x = 0.0;                   # move forward m/s (max:0.8m/s)
  			twist.angular.z = -0.1;   		#rotation rad/s (max:~2.62rad/s) minus is clockwise
			self.vel_pub.publish(twist)
			self.move_pub.publish(True)		

		#print ("Movement completed.")

if __name__ =="__main__":
        rospy.init_node('movementController',anonymous=True)
        try:
                movementController()

        except Exception as e:
                print "Failure: %s" % e

