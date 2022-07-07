from __future__ import print_function
import cv2
import numpy as np
import time
import serial
import cv2 as cv
import roslib
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
msg2=Quaternion()
msg3=Twist()
msg4=Quaternion()
msg5=Quaternion()
msg6=Quaternion()
msg7=Quaternion()
msg8=Quaternion()

stepPin = 4 
dirPin = 5 
x1=0
y1=0
print ("Entro")
min_speed=0.36 
#board = Arduino("/dev/ttyACM0")
captura = cv2.VideoCapture(2)
muestras=5000

print ("Entro2")
data = np.zeros((muestras, 2))
count=0;
jugadores = np.zeros([20,4])
print(jugadores,("matriz"))

posJugX = 0
posJugY = 0
razon = 1
medio1=0
medio2=0
medio3=0
medio4=0
delantero_Inf_Y=0
delantero_Inf_X=0
delantero_Sup_X=0
delantero_Sup_Y=0
delantero_Med_X=0
delantero_Med_Y=0
w_pelota=0
h_pelota=0
font = cv2.FONT_HERSHEY_SIMPLEX
PubArquero = rospy.Publisher("/Arquero",Quaternion, queue_size=20)
PubDefensas = rospy.Publisher("/Defensas",Twist,queue_size=20)
PubDelanteros = rospy.Publisher("/Delanteros",Quaternion,queue_size=20)
PubMedios = rospy.Publisher("/Medios",Quaternion,queue_size=20)
rospy.init_node('FILAS7', anonymous=True)
############################# prediccion
class KalmanFilter: 
    kf = cv.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def Estimate(self, coordX, coordY):
       measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
       self.kf.correct(measured)
       predicted = self.kf.predict()
       return predicted



while(True):
    ret, imagen = captura.read()
    
    if(ret == True):
        hsv = cv2.cvtColor(imagen, cv2.COLOR_BGR2HSV)
        
        verde_bajos = np.array([74,55,67], dtype=np.uint8)
        verde_altos = np.array([99, 255, 255], dtype=np.uint8)

            
        na_bajos = np.array([0,132,88], dtype=np.uint8)
        na_altos = np.array([255, 255, 255], dtype=np.uint8)
         
        mask = cv2.inRange(hsv, na_bajos, na_altos)
        mask_verdes = cv2.inRange(hsv, verde_bajos, verde_altos)

        #mask = cv2.medianBlur(mask,13)
        mask_verdes = cv2.medianBlur(mask_verdes,13)

        mask = cv2.medianBlur(mask,13)
        mask_verdes = cv2.medianBlur(mask_verdes,13)

        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask,kernel,iterations = 1)
        mask_verdes = cv2.dilate(mask_verdes,kernel,iterations = 1)
             
        _,contours,_ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        _,contours_verdes,_ = cv2.findContours(mask_verdes.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        kfObj = KalmanFilter()
        predictedCoords = np.zeros((2, 1))


        if len(contours_verdes)>0:
            sletter=0.5
            cnt_verdes=max(contours_verdes,key=cv2.contourArea)
            x1,y1,w_pelota,h_pelota = cv2.boundingRect(cnt_verdes) 
            cv2.putText(imagen,"x="+str(x1),(x1,y1+20), font, sletter,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(imagen,"y="+str(y1),(x1,y1+40), font, sletter,(255,255,255),2,cv2.LINE_AA) 
                  
        imagen = cv2.rectangle(imagen,(x1,y1),(x1+w_pelota,y1+h_pelota),(255,0,0),2)
        predictedCoords = kfObj.Estimate(x1, y1)
        imagen = cv2.circle(imagen, (x1, y1), 10, [0,0,255], 2)

        cv2.circle(imagen, (predictedCoords[0], predictedCoords[1]), 10, [0,255,255], 2)
        print ("prediccion en x", predictedCoords[0], "prediccion en y", predictedCoords[1])


        if len(contours) > 0:
            for i in range(len(contours)):
                cnt = contours[i] 
                x,y,w,h = cv2.boundingRect(cnt)
                imagen = cv2.rectangle(imagen,(x,y),(x+w,y+h),(0,255,0),2)
                sletter=0.5
                cv2.putText(imagen,"x="+str(x),(x,y+20), font, sletter,(255,255,255),2,cv2.LINE_AA)
                cv2.putText(imagen,"y="+str(y),(x,y+40), font, sletter,(255,255,255),2,cv2.LINE_AA)
                if(i<20):
                    jugadores[i]= x,y,w,h
                    #print(jugadores,"matriz2")
                    #print(x1,y1, "pelota") 
                    j=0 
                    for k in range(0,20):
                        if x1>=22 and x1 <104: 
                            if jugadores[k][0] > 15 and jugadores[k][0] < 25:
                                #print "Arquero: (",jugadores[k][0],",",jugadores[k][1],")"
                                ArqueroX = jugadores[k][0]
                                ArqueroY = jugadores[k][1]
                                
                                try:
                                    PubArquero.publish(msg2)
                                    msg2.x=float(predictedCoords[0]) #predicion en x
                                    msg2.y=float(predictedCoords[1]) #predicion en y
                                    msg2.z=float(ArqueroX)
                                    msg2.w=float(ArqueroY)
                                    rospy.loginfo(msg2)
                                except CvBridgeError as e: 
                                    print(e)
                        #################################Segunda linea############
                
                        if x1>=104 and x1 <276:
                            PubDefensas.publish(msg3)
                            msg3.linear.x=float(predictedCoords[0])
                            msg3.linear.y=float(predictedCoords[1])
                            if  jugadores[k][1]>=50 and jugadores[k][1]<=210 and jugadores[k][0] > 102 and jugadores[k][0]<110:
                                Defensa_Sup_X = jugadores[k][0]
                                Defensa_Sup_Y = jugadores[k][1]
                                msg3.linear.z=float(Defensa_Sup_X)
                                msg3.angular.x=float(Defensa_Sup_Y)
                            if  jugadores[k][1]>=216 and jugadores[k][1]<=351 and jugadores[k][0] > 102 and jugadores[k][0]<110:
                                Defensa_Inf_X = jugadores[k][0]
                                Defensa_Inf_Y = jugadores[k][1]
                                msg3.angular.y=float(Defensa_Inf_X)
                                msg3.angular.z=float(Defensa_Inf_Y)
                            rospy.loginfo(msg3)
                        ###############################Linea de medios########
                        if x1>=276 and x1 <454: 
                            PubMedios.publish(msg4)
                            PubMedios.publish(msg5)
                            PubMedios.publish(msg6)
                            msg4.x=float(predictedCoords[0])
                            msg4.y=float(predictedCoords[1])

                            if jugadores[k][0]>=279 and jugadores[k][0]<=286 and jugadores[k][1]>=79 and jugadores[k][1]<=148:
                                medio_Inf_X = jugadores[k][0]
                                medio1 = jugadores[k][1]
                                msg4.z=float(medio_Inf_X)
                                msg4.w=float(medio1)
                            
                            if jugadores[k][0]>=279 and jugadores[k][0]<=286 and jugadores[k][1]>=149 and jugadores[k][1]<=223:
                                medio_Inf_X = jugadores[k][0]
                                medio2 = jugadores[k][1]
                                msg5.x=float(medio_Inf_X)
                                msg5.y=float(medio2)
                            #print "medio2", medio2
                            if jugadores[k][0]>=279 and jugadores[k][0]<=286 and jugadores[k][1]>=224 and jugadores[k][1]<=292:
                                medio_Inf_X = jugadores[k][0]
                                medio3 = jugadores[k][1]
                                msg5.z=float(medio_Inf_X)
                                msg5.w=float(medio3)
                            #print "medio3", medio3
                            if jugadores[k][0]>=279 and jugadores[k][0]<=286 and jugadores[k][1]>=293 and jugadores[k][1]<=362:
                                medio_Inf_X = jugadores[k][0]
                                medio4 = jugadores[k][1]
                                msg6.x= float(medio_Inf_X)
                                msg6.y=float(medio4)
                            rospy.loginfo(msg4)
                            rospy.loginfo(msg5)
                            rospy.loginfo(msg6)

                            #print "medio4", medio4
                          

                        ###############DELANTEROS####################
                        if x1>=454 and x1<631:
                            PubDelanteros.publish(msg7)
                            PubDelanteros.publish(msg8) 
                            msg7.x=float(predictedCoords[0])
                            msg7.y=float(predictedCoords[1])
                            if jugadores[k][0]>=454 and jugadores[k][0]<=464 and jugadores[k][1]>=275 and jugadores[k][1]<=369:
                                #print "delantero inferior X", delantero_Inf_X, "inferior Y", delantero_Inf_Y
                                delantero_Inf_X = jugadores[k][0]
                                delantero_Inf_Y = jugadores[k][1]
                                msg7.z=float(delantero_Inf_X)
                                msg7.w=float(delantero_Inf_Y)
                                
                            # DelanteroMedioprint "delantero X", delantero_Inf_X
                            if jugadores[k][0]>=454 and jugadores[k][0]<=464 and jugadores[k][1]>=180 and jugadores[k][1]<=274:
                                #print "delantero medio X", delantero_Med_X, "medio en Y", delantero_Med_Y
                                delantero_Med_X = jugadores[k][0]
                                delantero_Med_Y = jugadores[k][1]
                                msg8.x=float(delantero_Med_X)
                                msg8.y=float(delantero_Med_Y)
                            # DelanteroSuperior
                            if jugadores[k][0]>=454 and jugadores[k][0]<=464 and jugadores[k][1]>=80 and jugadores[k][1]<=178:
                                #print "delantero superior X", delantero_Sup_X, "superior en Y", delantero_Sup_Y
                                delantero_Sup_X = jugadores[k][0]
                                delantero_Sup_Y = jugadores[k][1]
                                msg8.z=float(delantero_Sup_X)
                                msg8.w=float(delantero_Sup_Y)
                            rospy.loginfo(msg7)
                            rospy.loginfo(msg8)

            rangos = np.array([  [0,0,80,480],
                                [100,0,160,480],
                                [270,0,330,480],
                                [380,0,450,480]])
            cv2.imshow("original", imagen)
            try:
                original = identificacion(jugadores,imagen.copy(),x_pelota,y_pelota, rangos)
            except: print('Error en identificacion')
               
            #cv2.imshow('Anaranjado', mask)
            try:
                cv2.imshow('original', original)
            except: print('Error')
            #cv2.imshow('Verde', mask_verdes)
            

        else:
            print ("Sorry No contour Found.")
    

    ########################################################################################
    arqX, arqY, bolX, bolY = posJugX, posJugY, x1, y1
    #################################################################################3

    tecla = cv2.waitKey(5) & 0xFF
    if tecla == 27:
        break
cv2.destroyAllWindows()
captura.release()

