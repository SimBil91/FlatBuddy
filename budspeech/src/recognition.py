#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# This node uses Pyaudio, the speech recognition library (uses google speech api)
# currently: no own key used
# private key: AIzaSyD_RN5bb7QvX5YEHg5nWT2Q2D3PWA9dWwE
import speech_recognition as sr
import rospy
from budspeech.msg import speech

# Edit parameters
#r = sr.Recognizer(language = "en-US", key = "AIzaSyD_RN5bb7QvX5YEHg5nWT2Q2D3PWA9dWwE")
#r.pause_threshold = 0.8
#r.record(source, duration = None)
#r.listen(source, timeout = None)
#r.recognize(audio_data, show_all = False)
from std_srvs.srv import Empty
rec_state=0

def enable_recognition(state):
    global rec_state
    rec_state=1
    return []

def disable_recognition(state):
    global rec_state
    rec_state=0
    return []


def callback(recognizer, audio):                          # this is called from the background thread
    global rec_state
    try:
            # listen for the first phrase and extract it into audio data
        if rec_state==1:
            message = speech()
            result = recognizer.recognize(audio, True)
            message.text=str(result[0]['text'])
            message.confidence=result[0]['confidence']
            message.interface_id=1
            pub.publish(message)
            rospy.loginfo(result) 
        else:
            print('Recognition disabled')
    except LookupError as e:
        rospy.loginfo("Could not understand sentence!") 


if __name__ == '__main__':
    # Init node
    rospy.init_node('recognition')
    # init Publisher
    pub = rospy.Publisher('speech', speech)
    # init Service
    s_en = rospy.Service('enable_recognition', Empty, enable_recognition)
    s_dis = rospy.Service('disable_recognition', Empty, disable_recognition)
    r = sr.Recognizer()
    r.energy_threshold = 2000 # microphone dependent
    # perform listining and recognizing
    r.listen_in_background(sr.Microphone(), callback)


