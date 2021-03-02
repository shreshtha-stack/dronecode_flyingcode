import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from std_msgs.msg import Header, Int16, Float64
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import os
from mavros_test_common import Mavros_Test_Common 
import serial,struct,binascii

def xyz_to_lla(xyz,ref_lla=[19.134549, 72.912228, 0.0]):
    earth_radius_equator=6378137.00
    earth_radius_polar=6356752.31
    flattening_factor= (earth_radius_equator-earth_radius_polar)/earth_radius_equator
    eccentricity_sq=2*flattening_factor-math.pow(flattening_factor,2)
    radius_curvature_primevertical =earth_radius_equator/(math.sqrt(1-eccentricity_sq*math.pow(math.sin(math.radians(ref_lla[0])),2)))
    radius_curvature_meridian = (radius_curvature_primevertical*(1-eccentricity_sq))/(1-eccentricity_sq*math.pow(math.sin(math.radians(ref_lla[0])),2))

    lat = ref_lla[0]+math.degrees(xyz[0]/radius_curvature_meridian)
    lon = ref_lla[1]+math.degrees(xyz[1]/(radius_curvature_primevertical*math.cos(math.radians(ref_lla[0]))))
    alt = xyz[2]
    return [lat,lon,alt]

def lla_to_xyz(lla,ref_lla=[19.134549, 72.912228, 0.0]):
    home_lla = [19.134549, 72.912228, 0.0]
    earth_radius_equator=6378137.00
    earth_radius_polar=6356752.31
    flattening_factor= (earth_radius_equator-earth_radius_polar)/earth_radius_equator
    eccentricity_sq=2*flattening_factor-math.pow(flattening_factor,2)
    radius_curvature_primevertical =earth_radius_equator/(math.sqrt(1-eccentricity_sq*math.pow(math.sin(math.radians(ref_lla[0])),2)))
    radius_curvature_meridian = (radius_curvature_primevertical*(1-eccentricity_sq))/(1-eccentricity_sq*math.pow(math.sin(math.radians(ref_lla[0])),2))
    x = radius_curvature_meridian*math.radians(lla[0]-ref_lla[0])
    y = radius_curvature_primevertical*math.cos(math.radians(ref_lla[0]))*math.radians(lla[1]-ref_lla[1])
    z = lla[2]
    return [x,y,z]

def obtain_index_list(rx_str,to_be_found):
    list1=[]
    for i in xrange(len(rx_str)-1):
    	if rx_str[i]==to_be_found[0] and rx_str[i+1]==to_be_found[1]:
    		list1.append(i)
    return list1 

def rx_pkt_parser(rx_str):
	rx_pkt_dict = { "cmd_area":[], "mission_start":[], "RTL":[], "mission":[] }	
	cmd_area_pkt_list = []
	mission_start_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('ff0b'))
	mission_start_pkt_list = []
	RTL_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('ff0c'))
	RTL_pkt_list = []
	mission_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('fffc'))
	mission_pkt_list = []
	
	if len(cmd_area_index) > 0:
		for i in cmd_area_index:
			temp=[]
			temp = list(struct.unpack('=BBhhhlllHHB',rx_str[i:i+struct.calcsize('=BBhhhlllHHB')]))
			cmd_area_pkt_list.append(temp)
		rx_pkt_dict["cmd_area"] = cmd_area_pkt_lists
		
	if (len(mission_pkt_index) > 0):
		for i in mission_pkt_index:
			temp=[]
			temp = list(struct.unpack('=BBHlllHB',rx_str[i:i+struct.calcsize('=BBHlllHB')])) #mav_sys_id,lat,lon,alt
			print(temp)
			mission_pkt_list.append(temp)
    	rx_pkt_dict["mission"] = mission_pkt_list
    	
	if (len(RTL_pkt_index) > 0):
		for i in RTL_pkt_index:
			temp=[]
			temp = list(struct.unpack('=BBHB',rx_str[i:i+struct.calcsize('=BBHB')]))
			RTL_pkt_list.append(temp)
		rx_pkt_dict["RTL"] = RTL_pkt_list
		
	if (len(mission_start_pkt_index) > 0):
		for i in mission_start_pkt_index:
			temp=[]
			temp = list(struct.unpack('=BBHB',rx_str[i:i+struct.calcsize('=BBHB')]))
			mission_start_pkt_list.append(temp)
		rx_pkt_dict["mission_start"] = mission_start_pkt_list

	return rx_pkt_dict

class Main_class(Mavros_Test_Common):
    def setUp(self):        
        super(Main_class, self).setUp()
        self.mav_sys_id=1
        self.cluster_number=1
	
	#Setpoint Updation Thread
	self.pos= PoseStamped()
	self.pos_pub=rospy.Publisher(self.uavstring+'setpoint_position/local',PoseStamped,queue_size=10)
	self.pos_thread=Thread(target=self.send_pos,args=())
	self.pos_thread.daemon=True
	self.pos_thread.start()
	self.pos_thread_stop=False
	self.serial_port_status = False
		
	#Communication Thread
	self.serial_port= serial.Serial('/dev/ttyUSB0',57600,timeout=0.1)
	if self.serial_port.isOpen():
		self.serial_port_status==True			
	self.com_thread=Thread(target=self.com_func,args=())
	self.com_thread.daemon=True
	self.com_thread.start()
	
	#Thread maintining formation sync		
	self.formation_sync_thread=Thread(target=self.formation_sync_func,args=())
	self.formation_sync_thread.daemon=True
	self.formation_sync_thread.start()	
	self.node_data=[[0,0,0] for i in range(self.Total_nodes)]
	self.formation_sync_list=[0 for i in range(self.Total_nodes)]
	self.mission_pkt_TX_flag=False
	self.ack1_pkt_TX_flag = False
	self.ack2_pkt_TX_flag = False
	self.tx_retries=3
		
    def com_func(self):
		while True:
			try:
				if self.serial_port_status==True:
					try:
						#Transmit packets
						if self.mission_pkt_TX_flag:
							pkt1_TX_str=struct.pack('=BBHlllHB',0xff,0xfc,self.mav_sys_id,int(round(self.GPS_data.latitude,7)*1E7),int(round(self.GPS_data.longitude,7)*1E7),int(round(self.GPS_data.altitude,3)*1E3),self.formation_sync,0x76)
						#Check NavSatFix message ,why it wasnt used in Swaroop's code
						#If GPS data is not available,local position data from IMU could be converted to global coordinates and transmitted
							for _ in range(self.tx_retries):
								self.ser_port.write(pkt1_TX_str)
							self.mission_pkt_TX_flag = False 
                       				
                       			#RTL packet	
						if self.ack1_pkt_TX_flag:
							pkt2_TX_str=struct.pack('=BBHB',0xff,0xfd,self.mav_sys_id,0x76)
							for _ in range(self.tx_retries):
								self.ser_port.write(pkt2_TX_str)
							self.ack1_pkt_TX_flag = False 
                       				
                       			#Area packet	          
						if self.ack2_pkt_TX_flag:
							pkt3_TX_str=struct.pack('=BBHB', 0xff, 0xfe ,self.mav_sys_id,0x76)
							for _ in range(self.tx_retries):
								self.ser_port.write(pkt3_TX_str)
							self.ack2_pkt_TX_flag = False 

					except rospy.ROSException as e:
						pass	
						#Receive packets
					try:
						if self.serial_port.inWaiting():
							self.rx_str=self.ser_port.read(self.serial_port.inWaiting)
							self.rx_pkt_dict=rx_pkt_parser(self.rx_str)
							self.serial_port.flushInput()
							if self.rx_pkt_dict["cmd_area"]:
								for i in len(self.rx_pkt_dict["cmd_area"]):
									if self.cluster_number == self.rx_pkt_dict["cmd_area"][i][7]:
										self.area_length=self.rx_pkt_dict["cmd_area"][i][2] 
										self.area_breadth=self.rx_pkt_dict["cmd_area"][i][3] 
										self.area_centroid=[(self.rx_pkt_dict["cmd_area"][i][4])/1E7,(self.rx_pkt_dict["cmd_area"][i][5])/1E7 ,(self.rx_pkt_dict["cmd_area"][i][6])/1E3 ]
										self.ack1_pkt_TX_flag=True
									
							if self.rx_pkt_dict["mission"]:
								#parse the mission packet
								for i in len(self.rx_pkt_dict["mission"]):
									node_number=self.rx_pkt_dict["mission"][i][2]
									if node_number != self.mav_sys_id:
										self.node_data[node_number-1][0]=(self.rx_pkt_dict["mission"][i][3])/1E7 #lat
										self.node_data[node_number-1][1]=(self.rx_pkt_dict["mission"][i][4])/1E7 #lon
										self.node_data[node_number-1][2]=(self.rx_pkt_dict["mission"][i][5])/1E7 #alt
										self.formation_sync_list[node_number-1]=self.rx_pkt_dict["mission"][i][6]
									else:
										self.formation_sync_list[self.mav_sys_id-1]=self.formtion_sync
										self.node_data[node_number-1][0]=round(self.GPS_data.latitude,7) #lat
										self.node_data[node_number-1][1]=round(self.GPS_data.longitude,7) #lon
										self.node_data[node_number-1][2]=round(self.GPS_data.altitude,3) #alt
							
							if self.rx_pkt_dict["mission_start"]:
								for i in len(self.rx_pkt_dict["mission_start"]):
									if self.rx_pkt_dict["mission_start"][i][2]==self.mav_sys_id:
										self.ack2_pkt_TX_flag=True
									
							if self.rxt_pkt_dict["RTL"]:
								for i in len(self.rx_pkt_dict["RTL"]):
									if self.rx_pkt_dict["RTL"][i][2]==self.mav_sys_id:
										self.ack2_pkt_TX_flag=True										
							
						self.serial_port.flushInput()
					except rospy.ROSException as e:
						pass		
				else:
					Connection_retries_max=3
					retry_number=1
					while retry_number<=Connection_retries_max:
						self.serial_port=serial.Serial('/dev/ttyUSB0',57600,timeout=0.1)
						if self.serial_port.isOpen():
							self.serial_port_status==True
							rospy.loginfo('Connection Successful on retry {}'.format(retry_number))
							break
						retry_number+=1
					if retry_number==3:
						rospy.loginfo('Connection Unsuccessful.Check the module')

			except rospy.ROSException as e:
				rospy.loginfo('Issue in communication thread.Error encountered:{}'.format(e))
				

    def send_pos(self):
		loop_rate= 20                     					#Setpoint Publishing Rate 20 Hz
		rate=rospy.Rate(loop_rate)
		while self.pos_thread_stop==True :
			try:
				self.pos_data.header.stamp=rospy.Time.now()
				self.pos_pub.publish(self.pos)
				rate.sleep()
			except rospy.ROSException as e:
				rospy.loginfo('Issues sending position setpoints')
				rospy.fail(e)   


    def is_position(self, x, y, z, tolerance):       
        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))        
        l = (desired - pos) 
        for i in l:
            if abs(i) < tolerance:
                pass
            else:
                return False

    def reach_position(self, x, y, z, hdg, timeout):        
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
              
        loop_freq = 5 
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if (self.is_position(self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z,self.local_waypoint_radius)):
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
                
    def formation_sync_func(self,val):
    	for i in self.formation_sync_list:
    		if i!=val:
    			break
    	return False
    			

    def test_lissajou_pos_setpoints(self):
        self.wait_for_topics(60)
        self.wait_for_gps_lock(60)
        self.set_arm(True, 10)
        self.set_mode("AUTO.TAKEOFF", 5) 
        time.sleep(5)
        self.pos.pose.position.x = 0
        self.pos.pose.position.y = 0
        self.pos.pose.position.z = 10
        self.set_mode("OFFBOARD", 5)
        positions = [[0.0, 0.0, 10.0, 0]]   									# Here we will add the traj_planner function's list
        self.reach_position(0, 0 , 10, 0, 30)
        self.formation_sync=1
        while not self.formation_sync_func(1):
			self.pos.pose.position.x = self.local_position.pose.position.x 
        	self.pos.pose.position.y = self.local_position.pose.position.y
        	self.pos.pose.position.z = self.local_position.pose.position.z
        	
        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1], positions[i][2], positions[i][3], 30)   
                     
        self.set_mode("AUTO.LAND",5)
        
        loop_freq = 10 
        rate = rospy.Rate(loop_freq)        
        for i in range(30 * loop_freq):
            if self.extended_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
                  
        self.set_arm(False, 10)
    
if __name__ == '__main__':
    import rostest
    rospy.init_node('collisionnode')
    rostest.rosrun('drone_code', 'collisionnode',Main_class)
    
