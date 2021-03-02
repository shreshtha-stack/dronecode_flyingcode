PKG = 'drone_code'
nodename = 'uav0'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header, Int16, Float64
from threading import Thread
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import time
import os
from mavros_test_common_uav0 import MavrosTestCommon 


class VehicleNode_uav0(MavrosTestCommon):

    def setUp(self):        
        super(VehicleNode_uav0, self).setUp()
 
        if True:
            self.pos                    = PoseStamped()
            self.local_waypoint_radius  = 1.0
            self.pos_setpoint_pub       = rospy.Publisher(MavrosTestCommon.uavstring+'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
            self.pos_thread             = Thread(target=self.send_pos, args=())
            self.pos_thread.daemon      = True
            self.stop_pos_thread        = False
            self.pos_thread.start()

    def send_pos(self):
        rate = rospy.Rate(10) 
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while (not rospy.is_shutdown() and not self.stop_pos_thread):
            
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, tolerance):       
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
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))
        
        yaw_degrees = hdg 
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)
       
        loop_freq = 5 
        rate = rospy.Rate(loop_freq)
        for i in xrange(timeout * loop_freq):
            if (self.is_at_position(self.pos.pose.position.x,self.pos.pose.position.y,self.pos.pose.position.z,self.local_waypoint_radius)):
                break
            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)
    
    def test_lissajou_pos_setpoints(self):
        self.wait_for_topics(60)
        self.wait_for_gps_lock(60)
        self.set_arm(True, 10)
        self.set_mode("AUTO.TAKEOFF", 5) 
        time.sleep(5) 
        self.set_mode("OFFBOARD", 5)

        positions = [[0.0, 0.0, 10.0, 0], [10.0, 0.0, 10.0, 0],[10.0, 10.0, 10.0, 0],[0.0, 10.0, 10.0, 0],[0.0, 0.0, 10.0, 0]]
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
    rospy.init_node(nodename, anonymous=True)

    rostest.rosrun(PKG, 'vehiclenode_'+nodename,
                   VehicleNode_uav0)
    
