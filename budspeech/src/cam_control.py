#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# This node controls the camera, shakes head, follows face, takes pictures...
import speech_recognition as sr
import rospy
import pyTenvis
import io
import MySQLdb
import rospkg
import yaml
from threading import Thread
from std_msgs.msg import String

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

def cam_command(com):
    com=com.data
    IPCAM.motor.stop=False
    IPCAM.video.stop=False
    t3 = Thread(target=IPCAM.video.stream, args=('/var/www/pics/',))
    if com=='shake':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.shake_head(0.4,2)
    elif com=='nod':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.nod_head(0.3,2)
    elif com=='up':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.send_command('up','1')
    elif com=='down':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.send_command('down','1')
    elif com=='left':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.send_command('left','1')
    elif com=='right':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
        IPCAM.motor.send_command('right','1')
    elif com=='follow':
        t1 = Thread(target=IPCAM.video.pull_frames, args=())
        t2 = Thread(target=IPCAM.video.follow, args=(False,))
        t1.start()
        t2.start()
        #t2.join()
    elif com=='start_stream':
        IPCAM.video.show_stream=True
        t3.start()
    elif com=='stop_stream':
        IPCAM.video.show_stream=False
    elif com=='stop':
        IPCAM.motor.stop=True
        IPCAM.video.stop=True
    else:
        rospy.loginfo('Command not known')
            


if __name__ == '__main__':
    # Init node
    rospy.init_node('cam_control')
    # read IPCAM information
    db = MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    # TYPE 2 == IPCAM
    IPCAM_iter=0
    cursor.execute('SELECT id, IP, usr, pwd FROM interfaces WHERE inter_type = 2')
    sender_data=cursor.fetchall()[IPCAM_iter] # id of IPCAM, can be given by launchfile
    db.close()
    #Init IP audio
    try:
        IPCAM=pyTenvis.pyTenvis(sender_data[1],sender_data[2],sender_data[3])
    except:
        rospy.loginfo('IPCAM not reached. Check your settings')
        exit()
    
    # init Publisher
    rospy.Subscriber("budCAM", String, cam_command)
    rospy.spin()
    