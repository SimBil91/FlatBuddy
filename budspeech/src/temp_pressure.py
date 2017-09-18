#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: June 2017
import rospy
import time
import rospkg
from std_srvs.srv import Empty
from budspeech.msg import temp_pressure_data
from budspeech.msg import pressure_event
import Adafruit_BMP.BMP085 as BMP085
import math
from collections import deque

class temp_press_node(object):

    def __init__(self):
        rospy.init_node('temp_press')
        self.publish_data=True
        self.sensor = BMP085.BMP085()
        self.tp_data=temp_pressure_data()
        self.p_event=pressure_event()
        self.press_samples=40
        self.press_buffer=deque([0] * self.press_samples) # buffer size
        self.mean_buffer=deque([0] * 10) # buffer size
        self.press_counter=0
        self.rate=100
        self.pub_rate=0.05
        self.sent_event=False
        self.pub_counter=0
        # Init publisher
        self.pub_data = rospy.Publisher('temp_pressure/data', temp_pressure_data,queue_size=100)
        self.pub_event =  rospy.Publisher('pressure/event', pressure_event,queue_size=100)

        # Init services
        s_en = rospy.Service('publish_temp_pressure_data', Empty, self.enable_publishing)
        s_dis = rospy.Service('disable_publish_temp_pressure_data', Empty, self.disable_publishing)


    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.read_temp_pressure()
            self.process_pressure()
            r.sleep()

    def read_temp_pressure(self):
        tp_data = temp_pressure_data()
        tp_data.temp= self.sensor.read_temperature()
        tp_data.pressure= self.sensor.read_pressure()
        tp_data.altitude= self.sensor.read_altitude()
        self.tp_data=tp_data
        self.pub_counter+=1
        if (self.publish_data and self.pub_counter>(self.rate/self.pub_rate)):
            self.pub_counter=0
            self.pub_data.publish(self.tp_data)

    def process_pressure(self):
        # put new measurement in buffer
        self.press_buffer.popleft()
        self.press_buffer.append(self.tp_data.pressure)
        mean=sum(self.press_buffer)/len(self.press_buffer)

        loc_var=sum([(x - mean)**2 for x in list(self.press_buffer)])/(len(self.press_buffer)-1)
        #print(loc_var)
        self.press_counter+=1
        self.tp_data.pressure=0
        if (self.press_counter>self.press_samples):
            if(loc_var>35):

                #check if positive or negative
                positive=sum([(x-mean)>0 for x in list(self.press_buffer)])>self.press_samples/2

                if (not self.sent_event):
                    self.press_counter=0
                    if (positive):
                        self.pub_event.publish(pressure_event.TYPE_PRESSURE_RISE)
                    else:
                        self.pub_event.publish(pressure_event.TYPE_PRESSURE_DROP)
                    self.sent_event=True
            else:
                self.sent_event=False

        self.mean_buffer.popleft()
        self.mean_buffer.append(mean)

    def enable_publishing(self,state):
        self.publish_data=True
        return []

    def disable_publishing(self,state):
        self.publish_data=False
        return []

if __name__ == '__main__':
    tp=temp_press_node()
    tp.spin()
