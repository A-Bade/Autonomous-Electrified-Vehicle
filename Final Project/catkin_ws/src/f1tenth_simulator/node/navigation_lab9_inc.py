#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        odom_topic=rospy.get_param('~odom_topic')


        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.CenterOffset=rospy.get_param('~CenterOffset')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.tau=rospy.get_param('~tau')

        self.angle_bl=rospy.get_param('~angle_bl')
        self.angle_al=rospy.get_param('~angle_al')
        self.angle_br=rospy.get_param('~angle_br')
        self.angle_ar=rospy.get_param('~angle_ar')
        self.n_pts_l=rospy.get_param('~n_pts_l')
        self.n_pts_r=rospy.get_param('~n_pts_r')
        self.vehicle_velocity=rospy.get_param('~vehicle_velocity')
        self.turn_velocity=rospy.get_param('~turn_velocity')
        self.turn_angle1 = rospy.get_param('~turn_angle1')
        self.turn_angle2 = rospy.get_param('~turn_angle2')
        self.stop_time1 = rospy.get_param('~stop_time1')
        self.stop_time2 = rospy.get_param('~stop_time2')
        self.scan_beams=rospy.get_param('~scan_beams')
        self.safe_distance=rospy.get_param('~safe_distance')
        self.right_beam_angle=rospy.get_param('~right_beam_angle')
        self.left_beam_angle=rospy.get_param('~left_beam_angle')
        self.heading_beam_angle=rospy.get_param('~heading_beam_angle')
        self.stop_distance=rospy.get_param('~stop_distance')
        self.stop_distance_decay=rospy.get_param('~stop_distance_decay')
        self.velocity_zero=rospy.get_param('~velocity_zero')
        self.optim_mode=rospy.get_param('~optim_mode')

 

        #Subscriptions,Publishers
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        self.marker_pub = rospy.Publisher("wall_markers", Marker, queue_size = 2)
        self.marker = Marker()

        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.vel = 0

        self.ls_ang_inc=2*math.pi/self.scan_beams

        # Define field of view (FOV) to search for obstacles 

        self.ls_str=int(round(self.scan_beams*self.right_beam_angle/(2*math.pi)))
        self.ls_end=int(round(self.scan_beams*self.left_beam_angle/(2*math.pi)))
        self.ls_len_mod=self.ls_end-self.ls_str+1
        self.ls_fov=self.ls_len_mod*self.ls_ang_inc
        self.angle_cen=self.ls_fov/2
        self.ls_len_mod2=0
        self.ls_data=[]

        self.drive_state="normal"
        self.stopped_time =0.0
        self.yaw0 =0.0 
        self.dtheta = 0.0 
        self.yaw = 0.0
        t = rospy.Time.from_sec(time.time())
        self.current_time = t.to_sec()
        self.prev_time = self.current_time
        self.time_ref = 0.0
        self.wl0 = np.array([0.0, -1.0])
        self.wr0 = np.array([0.0, 1.0])


    # Pre-process LiDAR data    
    def preprocess_lidar(self, ranges):
        
        data=[]
        data2=[]     

        for i in range(self.ls_len_mod):
            if ranges[self.ls_str+i]<=self.safe_distance:
               data.append ([0,i*self.ls_ang_inc-self.angle_cen])
            elif ranges[self.ls_str+i]<=self.max_lidar_range:
                data.append ([ranges[self.ls_str+i],i*self.ls_ang_inc-self.angle_cen])
            else: 
                data.append ([self.max_lidar_range,i*self.ls_ang_inc-self.angle_cen])
        
        k1 = 100
        k2 = 40

        for i in range(k1):
            
            s_range = 0

            for j in range(k2):
                index1 = int(i*len(ranges)/k1+j)
                if index1 >= len(ranges):
                    index1 = index1-len(ranges)
                    
                index2 = int(i*len(ranges)/k1-j)
                if index2 < 0:
                    index2= index2 + len(ranges)    
                
                s_range = s_range + min(ranges[index1],self.max_lidar_range)+ min(ranges[index2],self.max_lidar_range)

            data2.append(s_range)
             
             
        return np.array(data), np.array(data2)



    # Return the start and end indices of the maximum gap in free_space_ranges
    def find_max_gap(self, proc_ranges):
        
        j=0
        str_indx=0;end_indx=0
        str_indx2=0;end_indx2=0
        
        range_sum = 0
        range_sum_new = 0

        for i in range (self.ls_len_mod):

            if proc_ranges[i,0]!=0:
                if j==0:
                    str_indx=i
                    range_sum_new = 0
                    j=1
                range_sum_new = range_sum_new + proc_ranges[i,0]
                end_indx=i
                
            if  j==1 and (proc_ranges[i,0]==0 or i==self.ls_len_mod-1):
               
                j=0

                if  range_sum_new > range_sum: 
                        end_indx2= ...
                        str_indx2= ...
                        range_sum = ...

        return str_indx2, end_indx2

    
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):

        ...
        ...
        ...      

        return best_heading


 
    def getWalls(self, left_obstacles, right_obstacles, wl0, wr0, alpha):

        
        
        if self.optim_mode == 0:
            
            Pr = ...
            Pl = ...

            bl = ...
            br = ...
        
            Cl= ...
            Cr= ...
            al = (1-alpha)*wl0 
            ar = (1-alpha)*wr0 
        

            wl = solve_qp(Pl.astype(np.float), al.astype(np.float), Cl.astype(np.float),  bl.astype(np.float), 0)[0]
            wr = solve_qp(Pr.astype(np.float), ar.astype(np.float), Cr.astype(np.float),  br.astype(np.float), 0)[0]

        else: 

            P = np.array([[1.0,0,0],[0,1.0,0],[0,0,0.0001]])

            bl = ...
            br = ...        
            b = np.concatenate((br,bl,np.array([-0.99, -0.99])))

            Cl= ...
            Cr= ...
            C1 = np.vstack((-Cr,br))
            C2 = np.vstack((Cl,- bl))
            C =np.hstack((C1,C2))
            C =np.hstack((C,np.array([[0,0],[0,0],[1.0,-1.0]])))
        
            a = ...

            ws  = solve_qp(P.astype(np.float), a.astype(np.float), C.astype(np.float),  b.astype(np.float), 0)[0]
        
            wr = ...
            wl = ...
        
        return  wl, wr 



    def lidar_callback(self, data):      

        ranges = data.ranges

        t = rospy.Time.from_sec(time.time())
        self.current_time = t.to_sec()
        dt = self.current_time - self.prev_time
    
        self.prev_time = self.current_time
        sec_len= int(self.heading_beam_angle/data.angle_increment)
        proc_ranges, mod_ranges = self.preprocess_lidar(ranges)     


        if self.drive_state == "normal":

            str_indx,end_indx=self.find_max_gap(proc_ranges)           
            heading_angle =self.find_best_point(str_indx, end_indx, proc_ranges)
        

            index_l=int(round((self.angle_bl-self.angle_al)/(data.angle_increment*self.n_pts_l)))
            index_r=int(round((self.angle_ar-self.angle_br)/(data.angle_increment*self.n_pts_r)))

            mod_angle_al = self.angle_al + heading_angle

            if mod_angle_al > 2*math.pi:
                mod_angle_al = mod_angle_al - 2*math.pi
            elif mod_angle_al < 0:
                mod_angle_al = mod_angle_al + 2*math.pi
        
            mod_angle_br = self.angle_br + heading_angle

            if mod_angle_br > 2*math.pi:
                mod_angle_br = mod_angle_br - 2*math.pi
            
            elif mod_angle_br < 0:
                mod_angle_br = mod_angle_br + 2*math.pi
 

            start_indx_l=int(round(mod_angle_al/data.angle_increment))
            start_indx_r=int(round(mod_angle_br/data.angle_increment))

            obstacle_points_l=np.zeros((self.n_pts_l,2))
            obstacle_points_r=np.zeros((self.n_pts_r,2))


            for k in range(0, self.n_pts_l):

                obs_index = (start_indx_l+k*index_l) % self.scan_beams
                obs_range= data.ranges[obs_index]
                if obs_range >=self.max_lidar_range:
                    obs_range = self.max_lidar_range
                
                
                obstacle_points_l[k][0]= - obs_range*math.cos(mod_angle_al+k*index_l*data.angle_increment)
                obstacle_points_l[k][1]= - obs_range*math.sin(mod_angle_al+k*index_l*data.angle_increment)

            for k in range(0,self.n_pts_r):
            
                obs_index = (start_indx_r+k*index_r) % self.scan_beams
                obs_range= data.ranges[obs_index]
                if obs_range >=self.max_lidar_range:
                    obs_range = self.max_lidar_range
                
                obstacle_points_r[k][0]= - obs_range*math.cos(mod_angle_br+k*index_r*data.angle_increment)
                obstacle_points_r[k][1]= - obs_range*math.sin(mod_angle_br+k*index_r*data.angle_increment)
        

            alpha = 1-math.exp(-dt/self.tau)
            
            wl, wr = self.getWalls(obstacle_points_l, obstacle_points_r, self.wl0, self.wr0, alpha)

            self.wl0 = wl
            self.wr0 = wr 
            
            dl = ...
            dr = ...

            wl_h = ... 
            wr_h = ... 

            self.marker.header.frame_id = "base_link"
            self.marker.header.stamp = rospy.Time.now() 
            self.marker.type = Marker.LINE_LIST
            self.marker.id = 0
            self.marker.action= Marker.ADD
            self.marker.scale.x = 0.1
            self.marker.color.a = 1.0
            self.marker.color.r = 0.5
            self.marker.color.g = 0.5
            self.marker.color.b = 0.0
            self.marker.pose.orientation.w = 1

            self.marker.lifetime=rospy.Duration(0.1)
    

            self.marker.points = []
        
            line_len = 1
            self.marker.points.append(Point(dl*(-wl_h[0]-line_len*wl_h[1]), dl*(-wl_h[1]+line_len*wl_h[0]) , 0))
            self.marker.points.append(Point(dl*(-wl_h[0]+line_len*wl_h[1]), dl*(-wl_h[1]-line_len*wl_h[0]) , 0))
            self.marker.points.append(Point(dr*(-wr_h[0]-line_len*wr_h[1]), dr*(-wr_h[1]+line_len*wr_h[0]) , 0))
            self.marker.points.append(Point(dr*(-wr_h[0]+line_len*wr_h[1]), dr*(-wr_h[1]-line_len*wr_h[0]) , 0))
            self.marker.points.append(Point(0, 0 , 0))
            self.marker.points.append(Point(line_len*math.cos(heading_angle), line_len*math.sin(heading_angle), 0))
   
            self.marker_pub.publish(self.marker)

        
            if self.vel >= 0.01 or self.vel <= -0.01:

                d_tilde= dl-dr - self.CenterOffset
                d_tilde_dot=self.vel*(wl_h[0]-wr_h[0])
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde+self.k_d*d_tilde_dot))/((self.vel**2)*(-wl_h[1]+wr_h[1])))
        
            else:
                delta_d = 0
            

            if delta_d >=self.max_steering_angle:
                delta_d=self.max_steering_angle
            elif delta_d<=-self.max_steering_angle:
                delta_d =-self.max_steering_angle
        
            min_distance = min(data.ranges[-sec_len+int(self.scan_beams/2):sec_len+int(self.scan_beams/2)])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)
            
            velocity=velocity_scale*self.vehicle_velocity

            if velocity <= self.velocity_zero:

                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()

                t = rospy.Time.from_sec(time.time())
                self.stopped_time = t.to_sec() - self.time_ref

                if self.stopped_time >= self.stop_time1:
                    self.drive_state = "backup"
                    self.time_ref = 0.0
                    self.yaw0 = self.yaw
                    self.turn_angle = np.argmax(mod_ranges)*(2*math.pi/len(mod_ranges)) - math.pi

            else:
                self.time_ref = 0.0
        
        elif self.drive_state == "backup":

            self.dtheta =  self.yaw-self.yaw0
            
            if abs(self.dtheta) >1.0:

                if np.sign(self.dtheta) != np.sign(self.turn_angle):
                    if self.dtheta < 0:
                        self.dtheta = self.dtheta + 4*math.pi
                    else:
                        self.dtheta = self.dtheta - 4*math.pi       

            min_distance = min(data.ranges[0:sec_len])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)

            delta_d = - np.sign(self.turn_angle)*self.max_steering_angle
            velocity= - velocity_scale*self.turn_velocity


            if abs(self.dtheta) >= abs(self.turn_angle/2.0): 
                self.drive_state = "turn"
                self.time_ref = 0.0
 

            elif - velocity <= self.velocity_zero:
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                    if self.stopped_time >= self.stop_time2: 
                        self.drive_state = "turn"
                        self.time_ref = 0.0
            else:
                 self.time_ref = 0            

        else:

            min_distance = min(data.ranges[-sec_len+int(self.scan_beams/2):sec_len+int(self.scan_beams/2)])    
            velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)

            delta_d =  np.sign(self.turn_angle)*self.max_steering_angle
            velocity = velocity_scale*self.turn_velocity

            self.dtheta =  self.yaw-self.yaw0
            
            if abs(self.dtheta) > 1.0:

                if np.sign(self.dtheta) != np.sign(self.turn_angle):
                    if self.dtheta < 0:
                        self.dtheta = self.dtheta + 4*math.pi
                    else:
                        self.dtheta = self.dtheta - 4*math.pi

            if abs(self.dtheta) >= abs(self.turn_angle):
                delta_d = 0.0
                velocity = 0.0 
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                    self.stopped_time = 0.0
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                if self.stopped_time >= self.stop_time2: 
                    self.drive_state = "normal"
                    self.time_ref = 0.0
                    self.wl0 = np.array([0.0, -1.0])
                    self.wr0 = np.array([0.0, 1.0])

            
            elif velocity <= self.velocity_zero:
                if self.time_ref == 0.0:
                    t = rospy.Time.from_sec(time.time())
                    self.time_ref = t.to_sec()
                else:
                    t = rospy.Time.from_sec(time.time())
                    self.stopped_time = t.to_sec() - self.time_ref

                    if self.stopped_time >= 1.0: 
                        self.drive_state = "backup"
                        self.time_ref = 0.0
                        self.yaw0 = self.yaw
                        self.turn_angle = np.argmax(mod_ranges)*(2*math.pi/len(mod_ranges)) - math.pi

            else:
                self.time_ref = 0.0
                          
            
        # Publish to driver topic
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = delta_d
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x
        #self.yaw_old = self.yaw
        self.yaw = 2*np.arctan2(odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w)


def main(args):
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
