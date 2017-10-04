#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: June 2017
import rospy
import time
import rospkg
from std_msgs.msg import String
from budspeech.msg import disp_emotion
from budspeech.msg import disp_action
from std_srvs.srv import Empty
from luma.led_matrix.device import max7219
from luma.core.interface.serial import spi, noop
from luma.core.render import canvas
from luma.core.virtual import viewport
from luma.core.legacy import text, show_message
from luma.core.legacy.font import proportional, CP437_FONT, TINY_FONT, SINCLAIR_FONT, LCD_FONT

import math
import random

class display_node(object):

    def __init__(self):
        rospy.init_node('display')
        self.rate=100
        self.serial = spi(port=0, device=0, gpio=noop())
        self.device = max7219(self.serial, cascaded=1, block_orientation=0)
        self.shift_counter=0
        self.peak_counter=3
        self.processing_count=0
        self.show_grec=True
        print("Created device")
        # Init Subscribers
        rospy.Subscriber("disp/text", String, self.show_text_message)
        rospy.Subscriber("disp/emotion", disp_emotion, self.emotion)
        rospy.Subscriber("disp/action", disp_action, self.action)

        # Init services
        s_dis = rospy.Service('stop_disp', Empty, self.stop_disp)
        i_proc = rospy.Service('inc_proc', Empty, self.increase_processing_count)
        dec_proc = rospy.Service('dec_proc', Empty, self.decrease_processing_count)
        show_grec = rospy.Service('show_grec', Empty, self.show_grec)
        show_lrec = rospy.Service('show_lrec', Empty, self.show_lrec)

        self.display_anim=False

    def stop_disp(self,state):
        self.display_anim=False
        return []

    def show_grec(self):
        self.show_grec=True

    def show_lrec(self):
        self.show_lrec=False

    def decrease_processing_count(self,state):
        self.processing_count=self.processing_count-1
        self.draw_processing_count()
        return []


    def increase_processing_count(self,state):
        self.processing_count=self.processing_count+1
        self.draw_processing_count()
        return []


    def draw_processing_count(self):
        if not self.display_anim:
            with canvas(self.device) as draw:
                for i in range(0,self.processing_count):
                    if self.show_grec:
                        draw.point((7,7-i), fill="white")
                    else:
                        draw.point((0,i), fill="white")


    def show_text_message(self,data):
         msg = data.data
         print(msg)
         show_message(self.device, msg, fill="white", font=proportional(LCD_FONT), scroll_delay=0.1)

    def sin_func(self,shift,peak):
        val=[round(peak*math.sin((x-shift)*(2*math.pi)/8))+4 for x in range(1,8)]
        return val

    def action(self,data):
        action_type=data.type
        self.stop_disp(False)
        if (action_type==disp_action.MUSIC):
            with canvas(self.device) as draw:
                text(draw, (0, 0), chr(14), fill="white")
        if (action_type==disp_action.SPEAKING):
            self.display_anim=True
            while(self.display_anim):
                with canvas(self.device) as draw:
                    val=self.sin_func(self.shift_counter,self.peak_counter)
                    self.shift_counter+=1
                    if self.shift_counter%2 is 0:
                        self.peak_counter=random.randint(1,3)
                    for i in range(0,7):
                        draw.point((i,int(val[i])), fill="white")
                    rospy.sleep(0.1)
            self.disable_disp()


        if (action_type==disp_action.PROCESSING):
            self.display_anim=True
            while(self.display_anim):
                for image in [1,2,3,2]:
                    with canvas(self.device) as draw:
                        self.shift_counter+=1
                        for shift in range(1,image+1):
                            for i in range(shift,8-shift):
                                draw.point((i,shift-1), fill="white")

                                draw.point((i,8-shift), fill="white")
                                draw.point((shift-1,i), fill="white")
                                draw.point((8-shift,i), fill="white")
                            draw.point((shift,shift), fill="white")
                            draw.point((7-shift,7-shift), fill="white")
                            draw.point((shift,7-shift), fill="white")
                            draw.point((7-shift,shift), fill="white")
                        self.device.contrast(int(round(255*(math.sin(self.shift_counter%6/5.0*2*math.pi)+1))/2))
                        rospy.sleep(0.12*(math.sin(self.shift_counter%6/5.0*2*math.pi)+1)/2+0.10)
            self.disable_disp()

    def disable_disp(self):
        self.device.contrast(255)
        with canvas(self.device) as draw:
            draw.rectangle(self.device.bounding_box, outline="black", fill="black")

    def emotion(self,data):
        emo_type = data.type
        if (emo_type==disp_emotion.POSITIVE):
            with canvas(self.device) as draw:
                text(draw, (0, 0), chr(1), fill="white")
                draw.point((3,4), fill="black")
                draw.point((4,4), fill="black")
        if (emo_type==disp_emotion.NEGATIVE):
            with canvas(self.device) as draw:
                text(draw, (0, 0), chr(1), fill="white")
                draw.point((2,4), fill="black")
                draw.point((5,4), fill="black")
                draw.point((3,5), fill="black")
                draw.point((4,5), fill="black")
                draw.point((2,5), fill="white")
                draw.point((5,5), fill="white")
        if (emo_type==disp_emotion.HAPPY):
            with canvas(self.device) as draw:
                text(draw, (0, 0), chr(1), fill="white")
        rospy.sleep(data.duration)
        self.disable_disp()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':
    disp=display_node()
    disp.spin()
