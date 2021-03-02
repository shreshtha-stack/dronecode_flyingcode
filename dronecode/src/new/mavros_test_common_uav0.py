import unittest
import rospy
import math
from std_msgs.msg import Float64, Int8, UInt32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped, Quaternion 
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, MountControl, GPSRAW
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, WaypointPush, CommandHome
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, BatteryState
from geographic_msgs.msg import GeoPoseStamped 

class MavrosTestCommon(unittest.TestCase):
  
    uavstring = '' 
    def __init__(self, *args):
        super(MavrosTestCommon, self).__init__(*args)
        
    def setUp(self):       
        self.local_position = PoseStamped()        
        self.state = State()        
        self.mav_sys_id = 1       
        self.batt_sts = BatteryState()        
        self.mount_orientation = Quaternion()
        self.gps_nsats = UInt32()             

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'local_pos',
                 'state', 'batt_sts', 'gps_nsats',
            ]
        }

        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service(self.uavstring+'/mavros/param/get', service_timeout)
            rospy.wait_for_service(self.uavstring+'/mavros/cmd/arming', service_timeout)          
            rospy.wait_for_service(self.uavstring+'/mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
            
        self.get_param_srv = rospy.ServiceProxy(self.uavstring+'/mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy(self.uavstring+'/mavros/cmd/arming',CommandBool)
        self.set_mode_srv = rospy.ServiceProxy(self.uavstring+'/mavros/set_mode', SetMode)       
       
        
        # ROS Topics
        self.altitude = Altitude()
        self.alt_sub = rospy.Subscriber(self.uavstring+'/mavros/altitude', Altitude, self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber(self.uavstring+'/mavros/extended_state', ExtendedState, self.extended_state_callback)       
        self.local_pos_sub = rospy.Subscriber(self.uavstring+'/mavros/local_position/pose',PoseStamped,self.local_position_callback)
        self.state_sub = rospy.Subscriber(self.uavstring+'/mavros/state', State,self.state_callback)
        self.batt_sts_sub = rospy.Subscriber(self.uavstring+'/mavros/battery',BatteryState, self.batt_sts_callback)        
        self.gps_nsats_sub = rospy.Subscriber(self.uavstring+'/mavros/global_position/raw/satellites',UInt32,self.gps_nsats_callback)        
     
   

    def gps_nsats_callback(self,data):
        self.gps_nsats = data
        if not self.sub_topics_ready['gps_nsats']:
            self.sub_topics_ready['gps_nsats'] = True
     
    def batt_sts_callback(self,data):
        self.batt_sts = data        
        if not self.sub_topics_ready['batt_sts']:
            self.sub_topics_ready['batt_sts'] = True 

    def altitude_callback(self, data):
        self.altitude = data      
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        self.extended_state = data
        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True


    def local_position_callback(self, data):
        self.local_position = data
        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        self.state = data     
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True
            

    def set_arm(self, arm, timeout):        
        loop_freq = 1  
        rate = rospy.Rate(loop_freq)            
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:                
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)        


    def set_mode(self, mode, timeout):        
        loop_freq = 1
        rate = rospy.Rate(loop_freq)        
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:               
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode) 
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def wait_for_topics(self, timeout):        
        loop_freq = 1 
        rate = rospy.Rate(loop_freq)        
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):                
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def wait_for_mav_sys_id(self, timeout):        
        loop_freq = 1  
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_SYS_ID')
                if res.success:
                    self.mav_sys_id = res.value.integer                    
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

    def wait_for_gps_lock(self, timeout):        
        gps_poll_freq = 1 
        gps_poll_rate = rospy.Rate(gps_poll_freq)        
        for i in xrange(timeout * gps_poll_freq):
            try:
                if (self.gps_nsats > 10):
                    rospy.loginfo("GPS LOCKED")
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)
            
            try:
                gps_poll_rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
