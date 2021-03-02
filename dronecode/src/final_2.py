import rospy
from rospy.exceptions import ROSException
from std_msgs.msg import Float64, Int8, UInt32,Header,UInt16
from geometry_msgs.msg import PoseStamped,Point
from geometry_msgs.msg import TwistStamped, Quaternion
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, PositionTarget,GPSRAW
from mavros_msgs.srv import CommandBool, ParamGet, SetMode
from sensor_msgs.msg import NavSatFix, BatteryState
import struct,math,binascii
from threading import Thread,Lock
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import serial
from traj_planner import path_planner
from grid_formation_array import nearest_square_method
import numpy as np
from navpy import ned2lla,lla2ned

uavstring = '/uav2'
setpoint_topic_select=0

def obtain_index_list(rx_str,to_be_found):
	list1=[]
	for i in range(len(rx_str)-1):
		if rx_str[i]==to_be_found[0] and rx_str[i+1]==to_be_found[1]:
			list1.append(i)
	return list1		
    
def rx_pkt_parser(rx_str):
    rx_pkt_dict = {"cmd_area":[],"mission_start":[],"RTL":[],"mission":[],"beacon":[]}
    cmd_area_index = obtain_index_list(rx_str,binascii.unhexlify('ff0a'))
    cmd_area_pkt_list = []
    mission_start_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('ff0b'))
    mission_start_pkt_list = []
    RTL_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('ff0c'))
    RTL_pkt_list = []
    mission_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('fffc'))
    mission_pkt_list = []
    beacon_pkt_index = obtain_index_list(rx_str,binascii.unhexlify('ffee'))
    beacon_pkt_list = []

    if (len(cmd_area_index) > 0):        
        for i in cmd_area_index:
            temp=[]
            temp = list(struct.unpack('=BBhhlllHHHB',rx_str[i:i+struct.calcsize('=BBhhlllHHHB')])) 
            cmd_area_pkt_list.append(temp)
        rx_pkt_dict["cmd_area"] = cmd_area_pkt_list
    rospy.loginfo(cmd_area_pkt_list)
    	
    if (len(mission_pkt_index) > 0):        
        for i in mission_pkt_index:
            temp=[]
            temp = list(struct.unpack('=BBHlllHB',rx_str[i:i+struct.calcsize('=BBHlllHB')])) #mav_sys_id,lat,lon,alt
            mission_pkt_list.append(temp)
        rx_pkt_dict["mission"] = mission_pkt_list    
    	
    if (len(mission_start_pkt_index) > 0):
        for i in mission_start_pkt_index:
            temp=[]
            temp = list(struct.unpack('=BBHB',rx_str[i:i+struct.calcsize('=BBHB')]))
            mission_start_pkt_list.append(temp)
        rx_pkt_dict["mission_start"] = mission_start_pkt_list
     
    if (len(RTL_pkt_index) > 0):
        for i in RTL_pkt_index:
            temp=[]
            temp = list(struct.unpack('=BBHB',rx_str[i:i+struct.calcsize('=BBHB')]))
            RTL_pkt_list.append(temp)
        rx_pkt_dict["RTL"] = RTL_pkt_list

    if (len(beacon_pkt_index) > 0):
        for i in beacon_pkt_index:
            temp=[]
            temp = list(struct.unpack('=BBHB',rx_str[i:i+struct.calcsize('=BBHB')]))
            beacon_pkt_list.append(temp)
        rx_pkt_dict["beacon"] = beacon_pkt_list
    return rx_pkt_dict


class fcu_mode:
    def __init__(self):
        rospy.loginfo('fcu_mode class')
        pass

    def setArm(self):
        rospy.wait_for_service(uavstring+'/mavros/cmd/arming',timeout=5)
        try:
            armService = rospy.ServiceProxy(uavstring+'/mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            rospy.loginfo("Service arming call failed: {}".format(e))

    def setDisarm(self):
        rospy.wait_for_service(uavstring+'/mavros/cmd/arming',timeout=5)
        try:
            armService = rospy.ServiceProxy(uavstring+'/mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            rospy.loginfo("Service Disarming call failed: {}".format(e))

    def setMode(self,mode):
        rospy.wait_for_service(uavstring+'/mavros/set_mode',timeout=5)
        try:
            flightModeService = rospy.ServiceProxy(uavstring+'/mavros/set_mode', SetMode)
            flightModeService(custom_mode=mode)
        except rospy.ServiceException as e:
            rospy.loginfo("service set_mode call failed: {}".format(e))

    def ObtainParam(self,param):
        rospy.wait_for_service(uavstring+'/mavros/param/get',timeout=5)
        try:
            get_param_srv = rospy.ServiceProxy(uavstring+'/mavros/param/get', ParamGet)
            data=get_param_srv(param)
            return data.value.integer
        except rospy.ServiceException as e:
            rospy.loginfo("Couldn't obtain parameters: {}".format(e))

class controller:
    def __init__(self):    
        self.altitude = Altitude()
        self.GPS_data = NavSatFix()
        self.current_state = State()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.local_position.pose.position=Point(0.0,0.0,0.0)
        self.extended_state = ExtendedState()
        self.gps_state=GPSRAW()
        if setpoint_topic_select==1:
            self.sp = PositionTarget()      
            self.sp.type_mask = int('010111111000', 2) # set the flag to use position setpoints and yaw angle     
            self.sp.position=Point(0.0,0.0,0.0)   # LOCAL_NED
            self.sp.coordinate_frame = 1
        else:
            self.sp=PoseStamped()
            self.sp.pose.position=Point(0.0,0.0,0.0)
            self.sp.header.frame_id = "base_footprint"
        
    def altitude_callback(self, data):
        self.altitude = data

    def GPS_data_callback(self, data):
        self.GPS_data = data

    def current_state_callback(self, data):
        self.current_state = data

    def home_position_callback(self, data):
        self.home_position = data

    def local_position_callback(self, data):
        self.local_position = data

    def extended_state_callback(self, data):
        self.extended_state = data

    def gps_state_callback(self, data):
        self.gps_state = data

    def pos_hold(self):
        if setpoint_topic_select==1:
        #PositionTarget
            self.sp.position.x = self.local_position.pose.position.x
            self.sp.position.y = self.local_position.pose.position.y
            self.sp.position.y = self.local_position.pose.position.z
        else:
        #PoseStamped
            self.sp.pose.position.x = self.local_position.pose.position.x
            self.sp.pose.position.y = self.local_position.pose.position.y
            self.sp.pose.position.y = self.local_position.pose.position.z

    def updateSp(self, x, y, z):
        if setpoint_topic_select==1:
            #PositionTarget
            self.sp.position.x = x
            self.sp.position.y = y
            self.sp.position.z = z
        else:
            #PoseStamped
            self.sp.pose.position.x = x
            self.sp.pose.position.y = y
            self.sp.pose.position.z = z


    def set_hdg(self,angle):
        if setpoint_topic_select==1:
            #PositionTarget
            self.sp.yaw = math.radians(angle)
        else:
            #PoseStamped    
            yaw = math.radians(angle)
            quaternion = quaternion_from_euler(0, 0, yaw)
            self.sp.pose.orientation = Quaternion(*quaternion)




class main_class:
    def __init__(self):
        self.cnt = controller()
        self.mode = fcu_mode()
        #rospy.Subscriber('/mavros/mount_control/orientation', Quaternion, cnt.mountOrientCb)
        self.alt_sub = rospy.Subscriber(uavstring+'/mavros/altitude', Altitude, self.cnt.altitude_callback)
        self.GPS_data_sub = rospy.Subscriber(uavstring+'/mavros/global_position/global', NavSatFix, self.cnt.GPS_data_callback)
        self.current_state_sub = rospy.Subscriber(uavstring+'/mavros/state', State, self.cnt.current_state_callback)
        self.home_position_sub = rospy.Subscriber(uavstring+'/mavros/home_position/home', HomePosition, self.cnt.home_position_callback)
        self.local_pos_sub = rospy.Subscriber(uavstring+'/mavros/local_position/pose', PoseStamped, self.cnt.local_position_callback)
        self.ext_state_sub = rospy.Subscriber(uavstring+'/mavros/extended_state', ExtendedState, self.cnt.extended_state_callback)
        self.gps_state_sub = rospy.Subscriber(uavstring+'/mavros/gpsstatus/gps1/raw', GPSRAW, self.cnt.gps_state_callback)
        if setpoint_topic_select==1:
            self.sp_pub = rospy.Publisher(uavstring+'/mavros/setpoint_raw/local',PositionTarget, queue_size=1)
        else:
            self.sp_pub = rospy.Publisher(uavstring+'/mavros/setpoint_position/local',PoseStamped, queue_size=1)

        self.tx_retries=3
        self.local_waypoint_radius=0.25
        self.initial_local_position = PoseStamped()
        self.pos_thread_flag=False
        self.mav_sys_id=0
        self.lock=Lock()
        self.mission_pkt_TX_flag=False
        self.RTL_pkt_rxd=False
        self.cmd_area_pkt_rxd=False
        self.cmd_mission_start_pkt_rxd=False	
        self.area_length=0
        self.area_breadth=0
        self.area_centroid=[0,0,0]
        self.list_generated=[]
        self.offset_xyz=[0,0,0]
        self.total_inspection_sites=0
        self.cluster_mav_sys_list = []

        self.ack2_pkt_TX_flag=False
        self.ack1_pkt_TX_flag=False
        self.mission_pkt_TX_flag=False
        


        self.pos_thread=Thread(target=self.send_pos)
        self.pos_thread.daemon=True
        self.pos_thread.start()
        self.formation_sync=0

        self.serial_port_status=False
        self.cluster_number=1
        self.nodes_per_cluster=1
        self.Vmax=5
        self.serial_port=serial.Serial('/dev/ttyUSB1',57600,timeout=0.1)
        if self.serial_port.isOpen():
            self.serial_port_status=True
        self.com_thread_flag=True
        self.beacon_tx_flag=True
        self.com_thread_TX=Thread(target=self.com_func_TX,args=())
        self.com_thread_TX.daemon=True
        self.com_thread_TX.start()

        self.com_thread_RX=Thread(target=self.com_func_RX,args=())
        self.com_thread_RX.daemon=True
        self.com_thread_RX.start()
        while not self.mav_sys_id:
            self.mav_sys_id=self.mode.ObtainParam('MAV_SYS_ID')
        rospy.loginfo('MAV_SYS_ID :{}'.format(self.mav_sys_id))
        while not self.cnt.GPS_data.latitude:
            rospy.loginfo("Launch lla:{}".format(self.cnt.altitude))
        self.data_dict = {self.mav_sys_id : [round(self.cnt.GPS_data.latitude,7),round(self.cnt.GPS_data.longitude,7),round(self.cnt.GPS_data.altitude,3)]}
        self.home_lla=[round(self.cnt.GPS_data.latitude,7),round(self.cnt.GPS_data.longitude,7),round(self.cnt.GPS_data.altitude,3)]
        self.node_data_list=[]
        rospy.loginfo("Active nodes in the network:{}".format(self.data_dict))
        self.local_id = 1
        self.wait_function(30)
        self.Total_nodes= len(self.node_data_list)
        rospy.loginfo("Waiting for area command")
        rate4=rospy.Rate(0.1)
        while True:
            if self.cmd_area_pkt_rxd:
                break
            rate4.sleep()
        try:
            if len(self.node_data_list)%2 == 0 and len(self.node_data_list)!=1 :
                self.formation_sync_list=[0 for i in range(int(self.Total_nodes/self.total_inspection_sites))]
            elif len(self.node_data_list)==1:
                self.formation_sync_list=[0]
            else:
                self.formation_sync_list=[0 for i in range(int((self.Total_nodes-1)/self.total_inspection_sites))]
        except ROSException as e:
            rospy.logerr(e)

        rospy.loginfo(self.formation_sync_list)
        for i in range(0,len(self.node_data_list),len(self.formation_sync_list)):
            list1 = self.node_data_list[i:i+len(self.formation_sync_list)]
            rospy.loginfo('list1 {}'.format(list1))
            if self.mav_sys_id in list1:
                self.cluster_mav_sys_list = list1
                self.local_id = int(list1.index(self.mav_sys_id)) +1 
            self.cluster_number = i +1
        rospy.loginfo('Total inspection sites {},cluster mav_sys_list: {}, local_id :{}, cluster_number : {} '.format(self.total_inspection_sites,self.cluster_mav_sys_list,self.local_id,self.cluster_number))

    def is_position(self, x, y, z, tolerance): 
        desired = np.array((x, y, z))
        pos = np.array((self.cnt.local_position.pose.position.x,
                        self.cnt.local_position.pose.position.y,
                        self.cnt.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < tolerance
        
    def reach_position(self, x, y, z, hdg, timeout): 
        self.cnt.updateSp(x,y,z)
        self.cnt.set_hdg(hdg)
        #self.cnt.sp.yaw = math.radians(hdg)
        loop_freq = 5 
        rate = rospy.Rate(loop_freq)
        for i in range(timeout * loop_freq):
            if not self.RTL_pkt_rxd:
                if setpoint_topic_select==1:
                    if (self.is_position(self.cnt.sp.position.x,self.cnt.sp.position.y,self.cnt.sp.position.z   ,self.local_waypoint_radius)):
                        rospy.loginfo('Reached position: {0},{1},{2}'.format(self.cnt.sp.position.x,self.cnt.sp.position.y,self.cnt.sp.position.z))
                        break
                else:            
                    if (self.is_position(self.cnt.sp.pose.position.x,self.cnt.sp.pose.position.y,self.cnt.sp.pose.position.z   ,self.local_waypoint_radius)):
                        rospy.loginfo('Reached position: {0},{1},{2}'.format(self.cnt.sp.pose.position.x,self.cnt.sp.pose.position.y,self.cnt.sp.pose.position.z))
                        break
            else:
                break
            
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def wait_function(self,timeout):
        rospy.loginfo("Waiting for {} seconds".format(timeout))
        rate3=rospy.Rate(1.1)
        for i in range(timeout):
            if i%5==0 and i!=0:
                rospy.loginfo("Active nodes in the network:{}".format(self.data_dict))
            rate3.sleep()
        self.beacon_tx_flag=False
        self.node_data_list=sorted(self.data_dict)
        rospy.loginfo('sorted list:'.format(self.node_data_list))
        

    def com_func_TX(self):
        rate=rospy.Rate(1)
        rospy.loginfo('Tx thread starts')
        while (not rospy.is_shutdown() and self.com_thread_flag):
            if self.serial_port_status==True:
            #Transmit packets
                if self.beacon_tx_flag:
                    pkt4_TX_str=struct.pack('=BBHB',0xff,0xee,self.mav_sys_id,0x76)
                    for _ in range(self.tx_retries):
                        self.serial_port.write(pkt4_TX_str)


                if self.mission_pkt_TX_flag:
                    pkt1_TX_str=struct.pack('=BBHlllHB',0xff,0xfc,self.mav_sys_id,int(round(self.cnt.GPS_data.latitude,7)*1E7),int(round(self.cnt.GPS_data.longitude,7)*1E7),int(round(self.cnt.GPS_data.altitude,3)*1E3),self.formation_sync,0x76)
                    for _ in range(self.tx_retries):
                        self.serial_port.write(pkt1_TX_str)
                       				
				#RTL packet	
                if self.ack1_pkt_TX_flag:
                    pkt2_TX_str=struct.pack('=BBHB',0xff,0xfd,self.mav_sys_id,0x76)
                    for _ in range(self.tx_retries):
                        self.serial_port.write(pkt2_TX_str)
                    rospy.loginfo('ACK sent')
                    self.ack1_pkt_TX_flag = False 
                       				
				#Area packet	          
                if self.ack2_pkt_TX_flag:
                    pkt3_TX_str=struct.pack('=BBHB', 0xff, 0xfe ,self.mav_sys_id,0x76)
                    for _ in range(self.tx_retries):
                        self.serial_port.write(pkt3_TX_str)
                    rospy.loginfo('ACK sent')

                    self.ack2_pkt_TX_flag = False
            else:
                retry_number=1
                while retry_number<=3:
                    self.serial_port=serial.Serial('/dev/ttyUSB1',57600,timeout=0.1)
                    if self.serial_port.isOpen():
                        self.serial_port_status==True
                    #rospy.loginfo('Connection Successful on retry {}'.format(retry_number))
                        break
                    retry_number+=1
                if retry_number==3:
                    rospy.loginfo('Connection Unsuccessful.Check the module') 
            rate.sleep()
		
    def com_func_RX(self):	   #Receive packets
        rate=rospy.Rate(1)
        rospy.loginfo('Rx thread starts')
        while (not rospy.is_shutdown() and self.com_thread_flag):
            if self.serial_port_status==True:			
                self.data_len=self.serial_port.inWaiting()
                #rospy.loginfo(' packet data length :{}'.format(self.data_len))

                if self.data_len:
                    self.rx_str=self.serial_port.read(self.data_len)
                    self.serial_port.flushInput()					
                    self.rx_pkt_dict=rx_pkt_parser(self.rx_str)
                    rospy.loginfo('Parsed packet data'.format(self.rx_pkt_dict))

                    if len(self.rx_pkt_dict["cmd_area"])>0:	
                        for i in range(len(self.rx_pkt_dict["cmd_area"])):
                            if self.cluster_number==self.rx_pkt_dict["cmd_area"][i][7]:                    
                                self.area_length=self.rx_pkt_dict["cmd_area"][i][2] 
                                self.area_breadth=self.rx_pkt_dict["cmd_area"][i][3] 
                                self.area_centroid=[(self.rx_pkt_dict["cmd_area"][i][4])/1E7,(self.rx_pkt_dict["cmd_area"][i][5])/1E7 ,(self.rx_pkt_dict["cmd_area"][i][6])/1E3 ]
                                self.total_inspection_sites=self.rx_pkt_dict["cmd_area"][i][8]
                                self.current_mission=self.rx_pkt_dict["cmd_area"][i][9]
                                self.ack1_pkt_TX_flag=True
                                self.cmd_area_pkt_rxd=True
                    		
                    if len(self.rx_pkt_dict["mission"])>0:												#parse the mission packet                    	
                        for i in range(len(self.rx_pkt_dict["mission"])):
                            node_number=self.rx_pkt_dict["mission"][i][2]
                            if node_number != self.mav_sys_id:
                                self.data_dict[node_number] = [(self.rx_pkt_dict["mission"][i][3])/1E7,(self.rx_pkt_dict["mission"][i][4])/1E7,(self.rx_pkt_dict["mission"][i][5])/1E7]
                                if node_number in self.cluster_mav_sys_list:
                                    self.formation_sync_list[self.cluster_mav_sys_list.index(node_number)]=self.rx_pkt_dict["mission"][i][6]
        

                    if len(self.rx_pkt_dict["beacon"])>0 and self.beacon_tx_flag:
                        for i in range(len(self.rx_pkt_dict["beacon"])):
                            if self.rx_pkt_dict["beacon"][i][2]:
                                if self.rx_pkt_dict["beacon"][i][2] not in self.data_dict:
                                    self.data_dict[self.rx_pkt_dict["beacon"][i][2]] = [0,0,0]
								
                    if len(self.rx_pkt_dict["mission_start"])>0:
                        for i in range(len(self.rx_pkt_dict["mission_start"])):
                            if self.rx_pkt_dict["mission_start"][i][2]==self.cluster_number:
                                self.ack2_pkt_TX_flag=True
                                self.cmd_mission_start_pkt_rxd=True
                    									
                    if len(self.rx_pkt_dict["RTL"])>0:
                        for i in range(len(self.rx_pkt_dict["RTL"])):
                            if self.rx_pkt_dict["RTL"][i][2]==self.cluster_number:
                                self.ack2_pkt_TX_flag=True
                                self.RTL_pkt_rxd=True
					
                    rospy.loginfo('{0} and {1}'.format(self.cmd_mission_start_pkt_rxd,self.cmd_area_pkt_rxd))
            else:
                retry_number=1
                while retry_number<=3:
                    self.serial_port=serial.Serial('/dev/ttyUSB1',57600,timeout=0.1)
                    if self.serial_port.isOpen():
                        self.serial_port_status==True
                    #rospy.loginfo('Connection Successful on retry {}'.format(retry_number))
                        break
                    retry_number+=1
                if retry_number==3:
                    rospy.loginfo('Connection Unsuccessful.Check the module') 
            rate.sleep()

    def send_pos(self):
        #self.lock.acquire()
        loop_rate= 20  
        rate=rospy.Rate(loop_rate)
        while self.pos_thread_flag==False :
            try:
                self.cnt.sp.header.stamp=rospy.Time.now()
                #rospy.loginfo('Published data :{}'.format(self.cnt.sp))
                self.sp_pub.publish(self.cnt.sp)                
                rate.sleep()
            except rospy.ROSException as e:
                    rospy.loginfo("Setpoint thread facing issues : {}".format(e))
      # Free lock to release next thread
            #self.lock.release()

    def formation_sync_func(self,val):
        flag=False
        for i in self.formation_sync_list:
            if i!=val:
                flag=True
                break
        if flag==True:
            return False
        else:
            return True

    def main(self):

        self.mission_pkt_TX_flag=True
        self.initial_local_position = self.cnt.local_position
        rate=rospy.Rate(5)

        rate1=rospy.Rate(1.5)
        while self.cnt.gps_state.satellites_visible<6 :
            rospy.loginfo('Waiting for GPS lock')
            rate1.sleep()

        rospy.loginfo('Satellites visible: {}'.format(self.cnt.gps_state.satellites_visible))

        while True:
            rospy.loginfo("wait for command from commander :{}".format(self.cmd_mission_start_pkt_rxd))			
            if self.cmd_mission_start_pkt_rxd:
                rospy.loginfo("mission start cmd rxd")
                break
            rate1.sleep()
        rospy.loginfo('Before initial position : {}'.format(self.cnt.local_position))

        if self.current_mission!=3 or self.local_id==1:
            while not self.cnt.current_state.armed:
                self.mode.setArm()
            rospy.loginfo('ARMING Successful')
            self.takeoff_alt=self.cnt.altitude.local
            rospy.loginfo('Current Mode : {}'.format(self.cnt.current_state.mode))
            rospy.loginfo('Local_position at launch : {}'.format(self.cnt.local_position))

            while self.cnt.current_state.mode!='AUTO.TAKEOFF':
                self.mode.setMode('AUTO.TAKEOFF')
            rospy.loginfo('Current Mode : {}'.format(self.cnt.current_state.mode))

            rospy.loginfo('Vehicle taking off')

            while not (self.cnt.altitude.local-self.takeoff_alt)>9:
                rospy.loginfo('Current Altitude : {}'.format(self.cnt.altitude.local))
                rate.sleep()
            self.cnt.updateSp(0,0,10)

            while self.cnt.current_state.mode!='OFFBOARD':
                self.mode.setMode('OFFBOARD')

            rospy.loginfo('Switched to Offboard Mode')
        
        position=[[0,0,10,0],[0,10,10,0],[10,10,10,0],[0,0,10,0]]

        if self.cnt.current_state.mode=='OFFBOARD' and  self.current_mission==1:
            
            for i in range(len(position)):
                rospy.loginfo('Local_position at launch : {}'.format(self.cnt.local_position))
                self.reach_position(position[i][0],position[i][1],position[i][2],position[i][3],30)
                rospy.loginfo('RTL pkt rxd:{}'.format(self.RTL_pkt_rxd))

                if self.RTL_pkt_rxd:
                    break

        if self.cnt.current_state.mode=='OFFBOARD' and  self.current_mission==2:
            self.reach_position(position[0][0],position[0][1],position[0][2],position[0][3],30)
            self.formation_sync=1
            self.formation_sync_list[self.local_id-1]=self.formation_sync        
            while True:
                rospy.loginfo('Hold position till sync in formation')
                rospy.loginfo('{}'.format(self.formation_sync_list))
                #self.cnt.pos_hold()
                self.reach_position(position[0][0],position[0][1],position[0][2],position[0][3],30)
                if self.formation_sync_func(1)==True:
                    break
                rate.sleep()

                    
            p1=path_planner(self.area_length,self.area_breadth,self.local_id,self.Total_nodes)
            rospy.loginfo('Local id is :{}'.format(self.local_id))
            self.list_generated = p1.get_trajectory_points()
            #rospy.loginfo('{}'.format(self.list_generated))
            rospy.loginfo('Values used for offset:{}{}{}{}{}{}'.format(self.area_centroid[0],self.area_centroid[1],self.area_centroid[2],self.home_lla[0],self.home_lla[1],self.area_centroid[2]))
            
            self.offset_xyz=lla2ned(self.area_centroid[0],self.area_centroid[1],self.area_centroid[2],self.home_lla[0],self.home_lla[1],self.area_centroid[2],latlon_unit='deg',alt_unit='m',model='wgs84')
            #self.offset_xyz=[0,0,0]
            self.reach_position(self.list_generated[0][0] + self.offset_xyz[0], self.list_generated[0][1] + self.offset_xyz[1], self.area_centroid[2], 0,120)
            self.formation_sync=2
            self.formation_sync_list[self.local_id-1]=self.formation_sync
            while True:
                rospy.loginfo('Hold position till sync in formation')
                rospy.loginfo('Current_pos{}'.format(self.cnt.local_position.pose.position.x,self.cnt.local_position.pose.position.y ))
                #self.cnt.pos_hold()
                self.reach_position(self.list_generated[0][0] + self.offset_xyz[0], self.list_generated[0][1] + self.offset_xyz[1], self.area_centroid[2], 0,120)
                rospy.loginfo('sync list :{}'.format(self.formation_sync_func(2)))
                if self.formation_sync_func(2)==True:
                    break
                rate.sleep()

            while not self.RTL_pkt_rxd:		
                if self.local_id==1:
                    for i in range(1,len(self.list_generated)) :
                        if not self.RTL_pkt_rxd:
                            self.reach_position(self.list_generated[i][0] + self.offset_xyz[0],self.list_generated[i][1] + self.offset_xyz[1],self.area_centroid[2],0,30)

                        
                    for i in reversed(range(len(self.list_generated)-1)) :
                        if not self.RTL_pkt_rxd:
                            self.reach_position(self.list_generated[i][0] + self.offset_xyz[0],self.list_generated[i][1] + self.offset_xyz[1],self.area_centroid[2],0,30)

                else:
                    for i in range(len(self.list_generated)) :
                        if not self.RTL_pkt_rxd:
                            #rospy.loginfo("{}".format(self.list_generated))
                            self.reach_position(self.list_generated[i][0] + self.offset_xyz[0],self.list_generated[i][1] + self.offset_xyz[1],self.area_centroid[2],0,30)

                rate.sleep()
        
        if self.current_mission==3:
            if self.local_id==1 and self.cnt.current_state.mode=='OFFBOARD':
                rospy.loginfo('Before reach_position : {}'.format(self.cnt.local_position))
                self.reach_position(position[0][0],position[0][1],position[0][2],position[0][3],30)
                rospy.loginfo('After reach_position : {}'.format(self.cnt.local_position))
                self.formation_sync=1
            else:
                while True:
                    rospy.loginfo('Waiting for previous vehicle to reach defined position')
                    if self.formation_sync_list[self.local_id-2]==1:
                        break

                if self.formation_sync_list[self.local_id-2]==1:
                    while not self.cnt.current_state.armed:
                        self.mode.setArm()
                        rospy.loginfo('ARMING Successful')
                        self.takeoff_alt=self.cnt.altitude.local
                        rospy.loginfo('Current Mode : {}'.format(self.cnt.current_state.mode))

                        while self.cnt.current_state.mode!='AUTO.TAKEOFF':
                            self.mode.setMode('AUTO.TAKEOFF')
                        rospy.loginfo('Current Mode : {}'.format(self.cnt.current_state.mode))

                        rospy.loginfo('Vehicle taking off')

                        rate=rospy.Rate(5)
                        while not (self.cnt.altitude.local-self.takeoff_alt)>6:
                            rospy.loginfo('Current Altitude : {}'.format(self.cnt.altitude.local))
                            rate.sleep()
                        vehicle_1_id = self.cluster_mav_sys_list[0]
                        lat_long_val = self.data_dict[vehicle_1_id]
                        self.offset_xyz=lla2ned(lat_long_val[0],lat_long_val[1],lat_long_val[2],self.cnt.home_position.geo.latitude,self.cnt.home_position.geo.longitude,self.cnt.home_position.geo.altitude,latlon_unit='deg',alt_unit='m',model='wgs84')
                        x,y = nearest_square_method(len(self.formation_sync_list),self.local_id,10,10)
                        self.cnt.updateSp(x+self.offset_xyz[0],y+self.offset_xyz[1],position[0][2])

                        while self.cnt.current_state.mode!='OFFBOARD':
                            self.mode.setMode('OFFBOARD')

                        rospy.loginfo('Switched to Offboard Mode')
            
                        self.reach_position(x+self.offset_xyz[0],y+self.offset_xyz[1],position[0][2],position[0][3],30)
                        self.formation_sync=1
            self.formation_sync_list[self.local_id-1]=self.formation_sync        
            while True:
                rospy.loginfo('Before reach_position : {}'.format(self.cnt.local_position))
                rospy.loginfo('Hold position till sync in formation')
                rospy.loginfo('{}'.format(self.formation_sync_list))
                self.cnt.pos_hold()
                if self.formation_sync_func(1)==True:
                    break
                rate.sleep()
                

            


        rospy.loginfo('Returning to launch')

        while self.cnt.current_state.mode!='AUTO.RTL' or  self.cnt.extended_state.landed_state!=1:
            self.mode.setMode('AUTO.RTL')

        while self.cnt.current_state.armed and self.cnt.extended_state.landed_state==1 :
            self.com_thread_flag=False
            self.pos_thread_flag=True
            self.mode.setDisarm()
        rospy.loginfo('Vehicle disarmed')

if __name__ =="__main__":
    rospy.init_node('offboard_controller', anonymous=True)
    c1=main_class()
    c1.main()
