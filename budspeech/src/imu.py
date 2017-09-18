#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: June 2017
import rospy
import time
import rospkg
from std_srvs.srv import Empty
from budspeech.msg import imu_data
from budspeech.msg import imu_event
from mpu6050 import mpu6050
import math

class imu_node(object):
    
    def __init__(self):
        rospy.init_node('imu')
        self.publish_data=True
        self.sensor = mpu6050(0x68)
        self.imu_data=imu_data()
        self.imu_event=imu_event()
        self.checking_acc=False
        self.acc_counter=0
        self.acc_thres=0

        self.rate=100
        # Init publisher
        self.pub_data = rospy.Publisher('imu/data', imu_data,queue_size=100)
        self.pub_event =  rospy.Publisher('imu/event', imu_event,queue_size=100)

        # Init services
        s_en = rospy.Service('publish_imu_data', Empty, self.enable_publishing)
        s_dis = rospy.Service('disable_publish_imu_data', Empty, self.disable_publishing)

        
    def spin(self):
        r = rospy.Rate(self.rate) 
        while not rospy.is_shutdown():
            self.read_acc_gyro()
            self.process_acc_gyro()
            r.sleep()

    def read_acc_gyro(self):
        acc_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()
        imu = imu_data()
        now = rospy.Time.now()
        imu.header.stamp = now
        imu.acceleration.x = acc_data["x"]
        imu.acceleration.y = acc_data["y"]
        imu.acceleration.z = acc_data["z"]
        imu.acc_mag=math.sqrt(self.imu_data.acceleration.x**2+ self.imu_data.acceleration.y**2 + self.imu_data.acceleration.z**2)

        imu.angular_velocity.x = gyro_data["x"]
        imu.angular_velocity.y = gyro_data["y"]
        imu.angular_velocity.z = gyro_data["z"]
        imu.angular_mag=math.sqrt(self.imu_data.angular_velocity.x**2+ self.imu_data.angular_velocity.y**2 + self.imu_data.angular_velocity.z**2)

        self.imu_data=imu
        if (self.publish_data):
            self.pub_data.publish(self.imu_data)

    def process_acc_gyro(self):
        if (self.imu_data.acc_mag>(9.81*2)):
            self.imu_event=imu_event.TYPE_SHAKE
            self.pub_event.publish(self.imu_event)
            if (self.checking_acc==False):
                self.checking_acc=True
                self.acc_counter=0
                self.acc_thres=1
                self.check_acc_5S()
        if (self.checking_acc==True):
            self.check_acc_5S()
            
        if (self.imu_data.angular_mag>(120)):
            self.imu_event=imu_event.TYPE_TURN
            self.pub_event.publish(self.imu_event)

    def check_acc_5S(self):
        # compute average acc magnitude over 5s
        self.acc_counter+=1

        if (self.imu_data.acc_mag>(10.5)):
            self.acc_thres+=1
        if (self.acc_counter>self.rate*5):
            if (self.acc_thres>10):
                self.imu_event=imu_event.TYPE_SHAKE_5S
                self.pub_event.publish(self.imu_event)
            self.checking_acc=False
        print(self.acc_thres)
            
    def enable_publishing(self,state):
        self.publish_data=True
        return []

    def disable_publishing(self,state):
        self.publish_data=False
        return []

if __name__ == '__main__':
    i=imu_node()
    i.spin()


