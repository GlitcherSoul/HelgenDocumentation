#!/usr/bin/env python3
import rclpy
import mysql.connector
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from rclpy.qos import ReliabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState

from datetime import datetime

class sql(Node):
	def __init__(self):
		super().__init__('sql')
		self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
		
		self.subscriber = self.create_subscription(Twist, '/cmd_vel', self.vel_callback,QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
		
		timer_period = 3.0
		self.tmr = self.create_timer(timer_period, self.timer_callback)
        
	def odom_callback(self,msg):
		self.odom = msg
		self.x = self.odom.pose.pose.position.x
		self.y = self.odom.pose.pose.position.y
		#self.get_logger().info("x: "+ str(self.x) + "y: " + str(self.y))
		
	def vel_callback(self,msg):
		self.twist = msg
		self.x_vel = self.twist.linear.x
		self.y_vel = self.twist.linear.y
		self.z_vel = self.twist.angular.z		
		
	def timer_callback(self):
		try:
			connection = mysql.connector.connect(
				host = '***********************************',
				port = *****,
				user = '****************',
				password = '**********',
				database='*************'
			)
			cursor = connection.cursor()	
			time = datetime.now()
			
			# Comandos SQL para insertar valores
			sql_insert = '''
					INSERT INTO HelgenData(x_, y_, vx, vy, vz)
					VALUES(%s, %s, %s, %s, %s);
            			     '''
			
			data = self.x, self.z, self.x_vel, self.y_vel, self.z_vel
			#data = 0.1, 0.2
			
			try:
			    self.get_logger().info("x_pose: "+ str(self.x) + " y_pose: " + str(self.y) + " x_vel: " + str(self.x_vel) + " y_vel: " + str(self.y_vel) + " z_vel: " + str(self.z_vel))
			    cursor.execute(sql_insert, data)
			    connection.commit()
			    
			except mysql.connector.Error as err:
			    self.get_logger().info("Error al enviar datos...")
			    #conn.rollback()

			# Cierra la conexión
			cursor.close()
		except:
			self.get_logger().info("Esperando movimiento del montacargas... Si el montacargas ya esta en movimiento, revisar conectividad")

def main(args=None):
	rclpy.init(args=args)
	node = sql()

	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
if __name__ == '__main__':
  main()
