from dronekit import connect , LocationLocal, LocationGlobalRelative, Command ,Attitude,Battery,VehicleMode, mavutil
import math
#from pymavlink import mavutil
import time
import sys

class aurornis():

	def __init__(self,connection_string = None, vehicle = None):
		#connection string - mavproxy stili bağlanma şekli örneğin tcp:127.0.0.1:5760
		#vehicle - dronekit arac tipi , default kullaniyoruz

		if not vehicle is None:
			self.vehicle = vehicle
			print("Hazir arac kullaniliyor")

		elif not connection_string is None:
			print("Araca baglaniliyor...")
			self._connect(connection_string)
		else:
			raise("HATA : gecerli bir dronekit vehicle yada connection string girilimedi!!!")
			return

		self._setup_listeners()  #telemetri mesajlarini al ve tut, ayrica mavlink ile mesajlar cogaltilabilir

		self.airspeed = 0.0          #hava hızı
		self.groundspeed = 0.0		 #yer hızı
		
		self.pos_lat = 0.0  		 #enlem
		self.pos_lon = 0.0			 #boylam
		self.pos_alt_rel = 0.0	     #yere göre yukseklik
		self.pos_alt_abs = 0.0		 #deniz seviyesi yuksekligi
		
		self.ned_north = 0.0
		self.ned_east = 0.0
		self.ned_down = 0.0
		self.ned_vx = 0.0
		self.ned_vy = 0.0
		self.ned_vz = 0.0
		
		self.ned_north = 0.0
		self.ned_east = 0.0
		self.ned_down = 0.0
		
		self.att_roll_deg = 0.0      #roll acisi
		self.att_pitch_deg = 0.0     #pitch acisi
		self.att_heading_deg = 0.0   #heading acisi,
		self.att_yaw_deg = 0.0
		
		self.wind_x = 0.0
		self.wind_y = 0.0
		self.wind_z = 0.0

		self.wind_speed = 0.0        #rüzgar hizi
		self.wind_dir_from_deg = 0.0 #araca gelen rüzgarin derecesi
		self.wind_dir_to_deg = 0.0   #

		self.climb_rate = 0.0
		self.throttle = 0.0

		self.ap_mode = ''

		self.mission = self.vehicle.commands #görev komutları-görevler

		self.parameters = self.vehicle.parameters  #missionplanner parametreleri

		self.location_home = LocationGlobalRelative(0,0,0)
		self.home_lat = 0.0
		self.home_lon = 0.0
		self.location_current = LocationGlobalRelative(0,0,0)
		
		self.ned_location = LocationLocal(0,0,0)

		self.distance = 0.0   #Lidar uzaklik

		self.mission = self.vehicle.commands

	def _connect(self, connection_string): #Private fonksiyon baska yere import edilemez
		self.vehicle = connect(connection_string , wait_ready = False, baud=921600, heartbeat_timeout = 360)


	def _setup_listeners(self): #private fonksiyon

		if True:
			@self.vehicle.on_message('ATTITUDE')
			def listener(vehicle,name ,message):    
				self.att_roll_deg = math.degrees(message.roll)
				self.att_pitch_deg = math.degrees(message.pitch)
				self.att_heading_deg = math.degrees(message.yaw)%360
				self.att_yaw_deg = math.degrees(message.yaw)

			@self.vehicle.on_message('GLOBAL_POSITION_INT')   #--pozisyon ve sürat
			def listener(vehicle,name ,message):
				self.pos_lat = message.lat*1e-7               #enlem
				self.pos_lon = message.lon*1e-7               #boylam
				self.pos_alt_rel = message.relative_alt*1e-3  #yere gore olan yukseklik
				self.pos_alt_abs = message.alt*1e-3           #deniz seviyesi yuksekligi
				self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel) #anlik konum
				#self.ned_location = LocationLocal(message.location.north,message.location.east,message.location.down)
			
			@self.vehicle.on_message('LOCAL_POSITION_NED')   #--pozisyon ve sürat
			def listener(vehicle,name ,message):
				self.ned_north = message.x
				self.ned_east = message.y
				self.ned_down = message.z
				self.ned_vx = message.vx
				self.ned_vy = message.vy
				self.ned_vz = message.vz
				
			
			@self.vehicle.on_message('VFR_HUD')               #Head up Displa
			def listener(vehicle, name, message):
				self.airspeed = message.airspeed
				self.groundspeed = message.groundspeed
				self.throttle = message.throttle
				self.climb_rate = message.climb 

			@self.vehicle.on_message('WIND')
			def listener(vehicle, name, message):
				self.wind_speed = message.speed
				self.wind_dir_from_deg = message.direction % 360
				self.wind_dir_to_deg = (self.wind_dir_from_deg + 180) % 360  #ruzgar gelme acisi
			
			@self.vehicle.on_message('WIND_COV')   #--pozisyon ve sürat
			def listener(vehicle,name ,message):
				self.wind_x = message.wind_x
				self.wind_y = message.wind_y
				self.wind_z = message.wind_z

			@self.vehicle.on_message('BATTERY')
			def listener(vehicle, name, message):
				self.battery = message.battery

			@self.vehicle.on_message('RAGEFINDER')
			def listener(self, name, message):
				self.distance = message.distance

		print("Baglanti Kuruldu!!!")		
		return (self.vehicle)
		
	def is_armed(self):                              # arm kontrol
		return (self.vehicle.armed)

	def arm(self):                                   # arm etme
		self.vehicle.armed = True

	def disarm(self):                                #disarm etme
		self.vehicle.armed = False

	def set_airspeed(self,speed):                    # havaya göre olan hizi değiştirme
		self.vehicle.airspeed = speed

	def get_ap_mode(self):                           #otopilot mod ogrenme
		self._ap_mode = self.vehicle.mode
		return (self.vehicle.mode)

	def set_ap_mode(self,mode):               #otopilot mod değiştirme
		time_0 = time.time()
		
		try:
			tgt_mode = VehicleMode(mode)
		except:
			return False

		while (self.get_ap_mode() != tgt_mode):
			self.vehicle.mode = tgt_mode
			time.sleep(0.2)
			if time.time() < time_0 + 5:
				return (False)

		return (True)


	def get_current_mission(self):

		print("Gorev indiriliyor!")
		self.download_mission()
		missionList = []
		n_wp = 0
		for wp in self.vehicle.commands:
			missionList.append(wp)
			n_wp += 1

		return n_wp,missionList

	def clear_mission(self):

		cmds = self.vehicle.commands
		self.vehicle.commands.clear()
		self.vehicle.flush()
		# After clearing the mission you MUST re-download the mission from the vehicle
		# before vehicle.commands can be used again
		# (see https://github.com/dronekit/dronekit-python/issues/230)
		self.mission = self.vehicle.commands
		self.mission.download()
		self.mission.wait_ready()
		
		
	def set_servo(self, servo_number, pwm_value):
		pwm_value_int = int(pwm_value)
		msg = self.vehicle.message_factory.command_long_encode(
			0, 0, 
			mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
			0,
			servo_number,
			pwm_value_int,
			0,0,0,0,0
			)
		self.vehicle.send_mavlink(msg)

	def download_mission(self):
		#--- download the mission
		""" Download the current mission from the vehicle.
        
		"""
		while self.pos_lat == 0.0:
			time.sleep(0.5)
			print ("Waiting for good GPS...")
		self.home_lat = self.pos_lat
		self.home_lon = self.pos_lon
        
		print("Home is saved as ")
		print(self.home_lat) 
		print(self.home_lon)
		
		self.vehicle.commands.download()
		self.vehicle.commands.wait_ready() # wait until download is complete.  
		self.mission = self.vehicle.commands


	def add_waypoint_to_mission(        # göreve atış waypointlerini ekler ve pixhawka gönderir
            self,            #--- vehicle object
            wp_Last_Latitude,   #--- [deg]  Target Latitude
            wp_Last_Longitude,  #--- [deg]  Target Longitude
            wp_Last_Altitude):  #--- [m]    Target Altitude
       
        # Get the set of commands from the vehicle
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()

            # Save the vehicle commands to a list
            missionlist=[]
            for cmd in cmds:
                missionlist.append(cmd)

            missionlist[12]=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                               wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
            
            missionlist[20]=Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 
                               wp_Last_Latitude, wp_Last_Longitude, wp_Last_Altitude)
        # Clear the current mission (command is sent when we call upload())
            cmds.clear()

        #Write the modified mission and flush to the vehicle
            for cmd in missionlist:
                cmds.add(cmd)
            cmds.upload()
        
            return (cmds.count)
