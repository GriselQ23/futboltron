#!/usr/bin/env python
from __future__ import print_function
from sympy import *
import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

abc = Point()
abc1 = Point()
abc2 = Point()
abc3 = Point()
efg = Twist()
###########GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG###############
#colores=Quaternion()
rospy.init_node('Publicador', anonymous=True)
Tiva=rospy.Publisher("Motores",Twist, queue_size=20)


def subscriber(): 
  
  rospy.Subscriber("Tiva2", Point, callback)
  rospy.Subscriber("TivaD", Point, callback2)
  rospy.Subscriber("TivaM", Point, callback3)
  rospy.Subscriber("Tiva4", Point, callback4)
  rospy.spin()
#============================ MASCARAS JUGADORES - PELOTA ======================================


#class image_converter:
#===================== DECLARACION DE SUSCRIPTORES Y PUBLICADORES MAS EL TIPO DE DATO======================

  
def callback(abc):
  a = abc.x #prendido y apagado del motor 
  s = abc.y #Sentido 
  ###############Publicacion 
  efg.linear.x = 1 ##########numero de motor 
  efg.linear.y = a ##########prendido y apagado del motor 
  efg.linear.z = s #########sentido del motor 
  print("Arquero")
  Tiva.publish(efg)

def callback2(abc1):
  a = abc1.x #prendido y apagado del motor 
  s = abc1.y #Sentido 
  ###############Publicacion 
  efg.linear.x = 2 ##########numero de motor 
  efg.linear.y = a ##########prendido y apagado del motor 
  efg.linear.z = s #########sentido del motor 
  print("Defensa")
  Tiva.publish(efg)

def callback3(abc2):
  a = abc2.x #prendido y apagado del motor 
  s = abc2.y #Sentido 
  ###############Publicacion 
  efg.linear.x = 3 ##########numero de motor 
  efg.linear.y = a ##########prendido y apagado del motor 
  efg.linear.z = s #########sentido del motor 
  print("Medio")
  Tiva.publish(efg)

def callback4(abc3):
  a = abc3.x #prendido y apagado del motor 
  s = abc3.y #Sentido 
  ###############Publicacion 
  efg.linear.x = 4 ##########numero de motor 
  efg.linear.y = a ##########prendido y apagado del motor 
  efg.linear.z = s #########sentido del motor 
  print("delantero")
  Tiva.publish(efg)
  


 

    
    

#===================================ENVIO DE DATOS ==============================================

'''def main(args):
  #ic = image_converter()
   # nombre del nodo
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()
'''
if __name__ == '__main__':
    try: 
      subscriber()
    except rospy.ROSInterruptException:
      pass

