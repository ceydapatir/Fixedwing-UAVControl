from dronekit import LocationGlobalRelative
from scipy.spatial import distance as dist
from Aurornis import aurornis
from pymavlink import mavutil
import scipy.integrate as si
from scipy import linalg
import numpy as np
import argparse
import time
import math
import psutil
import copy
import navpy
import cv2

cap = cv2.VideoCapture(0)
dispW=640
dispH=480
flip=2

flt = 0
total = 0
x = 0
y = 0

camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width= 640 , height= 480 , format=NV12, framerate=20/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
fourcc = cv2.VideoWriter_fourcc(*'MP4V') # XVID algoritmasını tanımlama
kayit_frame = cv2.VideoWriter("KayitFrame.avi",cv2.VideoWriter_fourcc(*"XVID"),20,(640,480))

class missions():

    def __init__(self,aurornis):
        self.aurornis = aurornis
        self.nextwaypoint = self.aurornis.vehicle.commands.next

    def get_target_ned(self,cx,cy): # hedefin piksel koordinatlarının ned framindeki konumunu hesaplar
        T_body_camera=np.array([[0,-1,0], [1,0,0], [0,0,1]])
        T_ned_camera=navpy.angle2dcm(self.aurornis.att_yaw_deg,self.aurornis.att_pitch_deg,self.aurornis.att_roll_deg)*T_body_camera #yaw pitch roll dan cosinüs matrise dönüşüm yapar
        T_camera_ned=linalg.inv(T_ned_camera)

        C=np.array([[self.aurornis.ned_north], [self.aurornis.ned_east], [self.aurornis.ned_down]]) #anlık ned frameleri alır
        #C=np.array([[1],[2],[2]])
        T_camera_ned2=np.array([[T_camera_ned[0,0],T_camera_ned[0,1]], [T_camera_ned[1 , 0],T_camera_ned[1,1]], [T_camera_ned[2,0],T_camera_ned[2,1]]]) # T_camera_ned matrisinin ilk iki sütununu alır
        T_camera_ned3=np.dot(T_camera_ned,C) # T_camera_ned ile C i çarpar

        A=np.array([[1569.75484, 0.0, 798.850457], [0.0, 1566.99991, 637.280372], [0.0, 0.0, 1.0]]) # kalibre edilmiş kamera değerleri
        matris1=np.hstack((T_camera_ned2,T_camera_ned3)) # iki matrisi birleştirir
        matris1=linalg.inv(matris1)

        A=linalg.inv(A)
        matris1=np.dot(A, matris1) # A ile matrisi çarpar
        p=np.array([[cx], [cy], [1]]) # piksel koordinatları, hedef yerde olduğu için z 1 alınır
        p=np.dot(1,p)
        T_pixel_ned=np.dot(matris1,p) # pikseller ile matrisin son halini çarpar ve sonucu verir

        return T_pixel_ned

    def airdrop(self,i,angle):  # atışı gerçekleştirir
        self.aurornis.set_servo(i,angle) # servoyu aç
        self.aurornis.set_servo(i,1000) #eski açıya dön

    def add_airdrop_wp_to_mission(self,x,y,target_ned): # hesaplanan hedef koordinatı ve düşüş mesafesi ile atış konumunu hesaplar
        # ned frame indeki hedef koordinatlarından atış mesafelerini çıkararak airdrop konumunun ned framindeki değerini hesaplar
        airdrop_wp = np.array([target_ned[0,0]-x, target_ned[1,0]-y, target_ned[2,0]])
        #[x,y,z] = navpy.ned2lla(airdrop_wp,39.7022878,32.7571428,0.0,latlon_unit='deg', alt_unit='m',model='wgs84')
        [x,y,z] = navpy.ned2lla(airdrop_wp,self.aurornis.home_lat, self.aurornis.home_lon, 0.0,latlon_unit='deg', alt_unit='m',model='wgs84') # hesaplanan değeri ned framinden lla formatına çevirir
        self.x = self.x * (self.total-1)
        self.y = self.y * (self.total-1)
        self.x = self.x + x
        self.y = self.y + y
        self.x = self.x / self.total
        self.y = self.y / self.total
        self.aurornis.add_waypoint_to_mission(self.x,self.y,30) # koordinatları missiona ekleyerek iki atış için wp leri ekler
        print("Airdrop waypoint: ")
        print([self.x,self.y,z])

    def get_airdrop_wp_with_euler(self,target_ned): # euler metodu ile bırakılan topların düşüş mesafelerini hesaplar
        A=0.0043
        p=1.225
        m=0.14
        g=-9.8
        CD=0.4
        z0=0
        x=0
        y=0
        step=100
        z = self.aurornis.ned_down
        Vx = self.aurornis.ned_vx
        Vy = self.aurornis.ned_vy
        Vz = self.aurornis.ned_vz   # 0 varsayılabilir
        Vr= self.aurornis.airspeed
        wx= self.aurornis.wind_x
        wy= self.aurornis.wind_y
        wz= self.aurornis.wind_z
        h = (z0-z)/step
        while z>0:
            x = x + h * Vx
            y = y + h * Vy
            z = z + h * Vz
            ax=((CD*p*A*(-1))/(2*m))*(Vx-wx)*Vr
            ay=((CD*p*A*(-1))/(2*m))*(Vy-wy)*Vr
            az=(g+(((CD*p*A*(-1))/(2*m))*(Vz-wz)*Vr))
            Vx = Vx + h * ax
            Vy = Vy + h * ay
            Vz = Vz + h * az

        self.add_airdrop_wp_to_mission(x,y,target_ned) # atış konumu hesaplanarak missiona eklenir

    def Flight(self):
        target_detected = False
        xa=0
        
        while True:
            next_wp = self.aurornis.mission.next
            servonum = 7 # ilk servo için pixhawk girişi

            if (target_detected == False and next_wp > 5 and next_wp < 8): # hedef tespit edilmediği müddetçe kamerayı açık tutar ve hedefi arar

                ret, frame = cap.read()
                blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                lower_red = np.array([0, 120, 70])
                upper_red = np.array([10, 255, 255])
                mask1 = cv2.inRange(hsv, lower_red, upper_red)
                lower_red = np.array([170, 120, 70])
                upper_red = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv, lower_red, upper_red)

                mask = mask1 + mask2

                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                merkez_noktasi = cv2.circle(frame, (320,220), 7, (0, 0, 255), -1)
                #cv2.putText(frame, "Center",(320,220),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)


                for contour in contours:
                    if cv2.contourArea(contour) > 20000:  # alan ... den buyukse hedef tespit edildiye döner ve hedefin pikselleri tutulur
                        c = max(contours, key = cv2.contourArea)
                        M = cv2.moments(c)
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        center = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(frame, center, radius, (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1)
                        cv2.putText(frame, "Center "+str(cx)+","+str(cy), (cx - 20, cy - 20),   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(frame,str(self.aurornis.location_current),(10,10),cv2.FONT_HERSHEY_PLAIN,0.8,200) # hedefin anlık lokasyon bilgisi videoya yazdırılır

                        target_detected = True

                kayit_frame.write(frame)


            if (next_wp > 8 ) :
                cap.release()
                cv2.destroyAllWindows()

            #if (target_detected==True and xa==0): # hedef tespit edilirse bir kerelik olmak üzere (xa) bu döngüye girer
            if (target_detected==True):
                #cap.release()
                #cv2.destroyAllWindows() # hedef tespit edildiği için videolar kapanır
                self.total = self.total+1
                target_ned=self.get_target_ned(cx,cy) # hedef ned koordinatları hesaplanır
                self.get_airdrop_wp_with_euler(target_ned)  # düşüş mesafeleri hesaplanarak atış konumu bulunur ve missiona eklenir
                xa=1 # tekrar girmemesi için xa 1 yapılır

                target_detected==False
                #break

            if (cv2.waitKey(15) & 0xFF == ord('q')) :
                break
            if (next_wp==24 ) :
                break
