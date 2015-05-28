#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# This node uses Pyaudio, the speech recognition library (uses google speech api)
# currently: no own key used
# private key: AIzaSyD_RN5bb7QvX5YEHg5nWT2Q2D3PWA9dWwE
import speech_recognition as sr
import rospy
import pyTenvis
import io
import MySQLdb
from budspeech.msg import speech
from std_srvs.srv import Empty
import rospkg
import yaml

rec_state=0
rospack = rospkg.RosPack()
path=rospack.get_path('budspeech')

def load_yaml():
    with open(path+'/src/master.yml', 'r') as readfile:
        data=yaml.load(readfile)
        try:
            return [data['IP'],data['usr'],data['pwd']]
        except:
            return ['','','']
        
[master_IP,master_usr,master_pwd]=load_yaml()

def enable_recognition(state):
    global rec_state
    rec_state=1
    return []

def disable_recognition(state):
    global rec_state
    rec_state=0
    return []

class PipeStream(sr.AudioSource):
    def __init__(self, taudio):
        self.stream = None
        #taudio.start_stream()
        self.pipe=taudio.pipe

    def __enter__(self):
        self.SAMPLE_WIDTH = 2
        self.RATE = 16000 # sampling rate in Hertz
        self.CHANNELS = 1 # mono audio
        self.CHUNK = 1024 # number of frames s
        self.stream = self.pipe.stdout
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        #self.stream = None
        self.pipe.stdout.flush()
        
        
def callback(recognizer, audio):                          # this is called from the background thread
    global rec_state
    try:
            # listen for the first phrase and extract it into audio data
        if rec_state==0:
            result = recognizer.recognize(audio, True)
            pub.publish(speech)
            rospy.loginfo(result) 
        else:
            print('Reconition disabled')
    except:
        rospy.loginfo("Could not understand sentence!") 
        
if __name__ == '__main__':
    # Init node
    rospy.init_node('recognition_IP')
    # read IPCAM information
    db = MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    # TYPE 2 == IPCAM
    IPCAM_iter=0
    cursor.execute('SELECT id, IP, usr, pwd FROM interfaces WHERE inter_type = 2')
    sender_data=cursor.fetchall()[IPCAM_iter] # id of IPCAM, can be given by launchfile
    db.close()
    # Init IP audio
    #try:
    IPCAM=pyTenvis.pyTenvis(sender_data[1],sender_data[2],sender_data[3])
    taudio=IPCAM.audio
    #sleep(0.1)
#except:
    rospy.loginfo('IPCAM not reached. Check your settings')
    #exit()
    
    # init Publisher
    pub = rospy.Publisher('speech', speech)
    # init Service
    s_en = rospy.Service('enable_recognition', Empty, enable_recognition)
    s_dis = rospy.Service('disable_recognition', Empty, disable_recognition)
    r = sr.Recognizer()
    r.energy_threshold = 1200 # microphone dependent
    r.pause_threshold = 1.2
    r.quiet_duration =0.8
    r.dynamic_energy_threshold = False
    # perform listining and recognizing
    while 1:
        with PipeStream(taudio) as source:                # use the default microphone as the audio source
            audio = r.listen(source) 
            print(r.energy_threshold)                  # listen for the first phrase and extract it into audio data
        try:
            # listen for the first phrase and extract it into audio data
            if rec_state==1:
                message = speech()
                result = r.recognize(audio, True)
                message.text=str(result[0]['text'])
                message.confidence=result[0]['confidence']
                message.interface_id=sender_data[0]
                pub.publish(message)
                rospy.loginfo(result) 
            else:
                print('Recognition disabled')
        except LookupError as err:
            rospy.loginfo("Could not understand sentence!") 


