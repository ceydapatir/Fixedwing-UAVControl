from Aurornis import aurornis
import argparse
import time
from Missions import missions

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyUSB0') #connection string USB olacak '/dev/ttyUSB0'   tcp:127.0.0.1:5762
args = parser.parse_args()

connection_string = args.connect

plane = aurornis(connection_string)
mission = missions(plane)

while True:
	secenek = input("\nLUTFEN SECINIZ\n1- Ucus oncesi kontrol\n2-Gorevi Baslat\n3-Servo Kontrol \n4-Çıkış \n----->")

	if secenek == '1':
		print("ARM KONTROL : {}".format(plane.is_armed()))
		print("OTO PILOT MOD : {}".format(plane.get_ap_mode()))
		print("KONUM : {}  {}".format(plane.pos_lat,plane.pos_lon))
		print("GPS : {}".format(plane.vehicle.gps_0))
		print("YER HIZI : {}".format(plane.vehicle.groundspeed))
		print("HAVA Hizi : {}".format(plane.vehicle.airspeed))
		print("EKF : {}".format(plane.vehicle.ekf_ok))
		print("YUSEKLIK RELATIVE : {}".format(plane.pos_alt_rel))
		print("BATARYA : {}".format(plane.vehicle.battery))
		print("THROTTLE : {}".format(plane.throttle))
		#print("Lidar uzakligi {}".format(plane.distance))
		print("\n")
		time.sleep(1)


	elif secenek == '2':

		print("\n2. Gorev baslatiliyor\n")
		n_WP,missionList = plane.get_current_mission()
		print(n_WP)
		time.sleep(2)

		#if not plane.is_armed():
		#	plane.arm_and_takeoff()#ARM EDIP TAKE OFF KOMUTU VERIR

		mission.Flight()
		break
		
	elif secenek == '3':

		print("\n2. Servo kontrol ...\n")
		i = input("\nServo giriş no:")
		open_servo = input("\nServo açma açısı:")
		#close_servo = input("\nServo kapama açısı:")
		plane.set_servo(i,open_servo) # servoyu aç
		#plane.set_servo(i,close_servo) #eski açıya dön
		"""
		plane.vehicle.set_servo(7,1300) # servoyu aç
		plane.vehicle.set_servo(7,1500) #eski açıya dön
		
		plane.vehicle.set_servo(9,2000) # servoyu aç
		plane.vehicle.set_servo(9,1800) #eski açıya dön
		"""
		break


	elif secenek == '4':
		break
