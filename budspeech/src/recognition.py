#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# This node uses Pyaudio, the speech recognition library (uses google speech api)
# currently: no own key used
# private key: AIzaSyD_RN5bb7QvX5YEHg5nWT2Q2D3PWA9dWwE
import speech_recognition as sr
import rospy
import time
from budspeech.msg import speech
from std_srvs.srv import Empty


class recognition_node(object):
    def __init__(self):
        self.rec_state=1
        self.hot_word="buddy"
        # Init node
        rospy.init_node('recognition')
        # init Publisher
        self.pub = rospy.Publisher('speech', speech,queue_size=100)
        # init Service
        s_en = rospy.Service('enable_recognition', Empty, self.enable_recognition)
        s_dis = rospy.Service('disable_recognition', Empty, self.disable_recognition)
        self.inc_proc=rospy.ServiceProxy('inc_proc', Empty)
        self.dec_proc=rospy.ServiceProxy('dec_proc', Empty)
        self.grec=rospy.ServiceProxy('show_grec', Empty)
        self.lrec=rospy.ServiceProxy('show_lrec', Empty)
        self.init_recognition()
        self.rec_local=True
        self.recognition_time=time.time()-100000
        self.time_without_detection=60


    def init_recognition(self):
        r = sr.Recognizer()
        r.dynamic_energy_threshold = False
        m = sr.Microphone()
        with m as source:
            r.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening
            #r.energy_threshold = 100 # microphone dependent
            #perform listining and recognizing
        # start recognition
        self.stop_listen=r.listen_in_background(m, self.callback)
    def enable_recognition(self,state):
        self.rec_state=1
        return []

    def disable_recognition(self,state):
        self.rec_state=0
        return []

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


    def callback(self,recognizer, audio):                          # this is called from the background thread
        if self.rec_state==1:
            try:
                # listen for the first phrase and extract it into audio data
                print("processing data..")
                if (time.time()-self.recognition_time<self.time_without_detection):
                    self.rec_local=False
                    self.lrec()
                else:
                    self.rec_local=True
                    self.grec()
                if (self.rec_local):
                    self.inc_proc()
                    hot_word_res=recognizer.recognize_sphinx(audio, keyword_entries=[(self.hot_word, 1)])
                    self.dec_proc()
                    print("local recognition")
                else:
                    hot_word_res=self.hot_word
                if any(self.hot_word in s for s in hot_word_res.split()):
                    try:
                        result = recognizer.recognize_google(audio)
                        message = speech()
                        message.text=str(result)
                        message.confidence=1
                        message.interface_id=1
                        self.pub.publish(message)
                        rospy.loginfo(result)
                        if any(self.hot_word in u for u in message.text.split()):
                            self.recognition_time=time.time()
                    except sr.UnknownValueError:
                        print("Google Speech Recognition could not understand audio")
                    except sr.RequestError as e:
                        print("Could not request results from Google Speech Recognition service; {0}".format(e))

            except sr.UnknownValueError:
                self.dec_proc()
                print("Hot word not detected!")
            except sr.RequestError as e:
                print("Could not request results from Google Speech Recognition service; {0}".format(e))

        else:
            print('Recognition disabled')




if __name__ == '__main__':
    rec=recognition_node()
    rec.spin()
