#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry




class WallFollow:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        odom_topic=rospy.get_param('~odom_topic')

        self.t_prev=rospy.get_time()
        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.CenterOffset=rospy.get_param('~CenterOffset')
        self.DistanceLeft=rospy.get_param('~DistanceLeft')
        self.DistanceRight=rospy.get_param('~DistanceRight')
        self.TrackWall=rospy.get_param('~TrackWall')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.angle_bl=rospy.get_param('~angle_bl')
        self.angle_al=rospy.get_param('~angle_al')
        self.angle_br=rospy.get_param('~angle_br')
        self.angle_ar=rospy.get_param('~angle_ar')

        self.vehicle_velocity=rospy.get_param('~vehicle_velocity')
        self.heading_beam_angle=rospy.get_param('~heading_beam_angle')
        self.stop_distance=rospy.get_param('~stop_distance')
        self.stop_distance_decay=rospy.get_param('~stop_distance_decay')
        self.scan_beams=rospy.get_param('~scan_beams')

    
        #Subscriptions,Publishers
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.vel = 0

        self.thetal= self.angle_bl - self.angle_al
        self.thetar= self.angle_ar - self.angle_br

    def getRange(self, data, angle):
        
        index=int(round(angle/data.angle_increment))
        data2=data.ranges[index]

        if data2>=self.max_lidar_range:
            data2=self.max_lidar_range
        return data2



    def lidar_callback(self, data):      

        dis_lsr_al=self.getRange(data, self.angle_al)
        dis_lsr_bl=self.getRange(data, self.angle_bl)

        dis_lsr_ar=self.getRange(data, self.angle_ar)
        dis_lsr_br=self.getRange(data, self.angle_br)

        betal= math.atan(dis_lsr_al*math.cos(self.thetal) - dis_lsr_bl)/(dis_lsr_al*math.sin(self.thetal))
        betar= math.atan(dis_lsr_ar*math.cos(self.thetar) - dis_lsr_br)/(dis_lsr_ar*math.sin(self.thetar))

        alphal = -betal -self.angle_bl + 3*math.pi/2
        alphar = betar - self.angle_br + math.pi/2

 
        dl=dis_lsr_bl*math.cos(betal)
        dr=dis_lsr_br*math.cos(betar)

        sec_len= int(self.heading_beam_angle/data.angle_increment)

        
        if self.vel >= 0.01 or self.vel <= -0.01:

            if self.TrackWall == 1:
                d_tilde = self.DistanceLeft-dl
                d_dot=-self.vel*math.sin(alphal)
                delta_d = math.atan(-(self.wheelbase*(self.k_p*d_tilde-self.k_d*d_dot))/((self.vel**2)*math.cos(alphal)))
            
            elif self.TrackWall == 2:
                d_tilde = self.DistanceRight-dr
                d_dot= self.vel*math.sin(alphar)
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde-self.k_d*d_dot))/((self.vel**2)*math.cos(alphar)))
            
            else :
                d_tilde= dl-dr-self.CenterOffset
                d_tilde_dot= -self.vel*math.sin(alphal)-self.vel*math.sin(alphar)
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde+self.k_d*d_tilde_dot))/((self.vel**2)*(math.cos(alphal)+math.cos(alphar))))
        
        else:
            delta_d = 0
                    

        if delta_d >=self.max_steering_angle:
            delta_d=self.max_steering_angle
        elif delta_d<=-self.max_steering_angle:
            delta_d =-self.max_steering_angle
        
        min_distance = min(data.ranges[-sec_len+int(self.scan_beams/2):sec_len+int(self.scan_beams/2)])    
        velocity_scale = 1-math.exp(-max(min_distance-self.stop_distance,0)/self.stop_distance_decay)
           
        velocity=velocity_scale*self.vehicle_velocity



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


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
