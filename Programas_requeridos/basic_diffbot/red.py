#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from roboflow import Roboflow
import numpy as np

class red(Node):
  def __init__(self):
    super().__init__('red')
    #Suscripcion topico imagen
    self.subscription = self.create_subscription(
      Image, 
      '/camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription
    self.br = CvBridge()
    
    #Obtencion de red neuronal
    self.model = torch.hub.load('ultralytics/yolov5', 'custom', path = '/home/<user_name>/proyect_ws/best.pt')
    self.classes = self.model.names
    self.model.conf = 0.60
    
    #Publicar resultados
    self.publisher = self.create_publisher(Image, 'red', 10)
    
    timer_period = 0.1  # seconds
    self.timer = self.create_timer(timer_period, self.red_callback)
  
  def class_to_label(self, x):
    return self.classes[int(x)]
   
  def listener_callback(self, data):
    self.frame = self.br.imgmsg_to_cv2(data)
    
    dim = (416, 416)
    self.frame = cv2.resize(self.frame, dim, interpolation = cv2.INTER_AREA)

  def red_callback(self):
    try:
    	self.detect = self.model(self.frame)
    	labels, cord = self.detect.xyxyn[0][:, -1], self.detect.xyxyn[0][:, :-1]
    	n = len(labels)
    	x_shape, y_shape = self.frame.shape[1], self.frame.shape[0]
    	for i in range(n):
    		row = cord[i]
    		if row[4] >= 0.3:
    			x1, y1, x2, y2 = int(row[0]*x_shape), int(row[1]*y_shape), int(row[2]*x_shape), int(row[3]*y_shape)
    			bgr = (0, 255, 0)
    			cv2.rectangle(self.frame, (x1, y1), (x2, y2), bgr, 2)
    			cv2.putText(self.frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)
    	try:
    		self.publisher.publish(self.br.cv2_to_imgmsg(self.frame, "bgr8"))
    	except CvBridgeError as e:
    		self.get_logger().info(e)
    except:
    	self.get_logger().info("Cargando detección...")

def main(args=None):

  rclpy.init(args=args)

  image_subscriber = red()

  rclpy.spin(image_subscriber)
  
  image_subscriber.destroy_node()

  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
