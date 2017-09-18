#!/usr/bin/env python
# author: Simon Bilgeri, Munich, Germany
# date: March 2015
# Input: Natural language sentences in text
# Output:

import rospy
import nltk
import MySQLdb
import urllib2
import rospkg
#import pygame
import os
import pywapi
import random
import wolframalpha
import yaml
from std_srvs.srv import Empty
from std_msgs.msg import String
from budspeech.msg import speech
from budspeech.msg import disp_emotion
from budspeech.msg import disp_action
from budspeech.msg import temp_pressure_data
from budspeech.msg import pressure_event
from budspeech.msg import imu_event
from spotipy.oauth2 import SpotifyClientCredentials
import spotipy
import pprint
import subprocess


import rospkg
import time

class brain_node(object):

    def __init__(self):
        # Variables
        self.shout='buddy'
        self.aw_response=''
        self.num_response=['',0]
        self.POWER=1
        self.current_temp=0
        self.current_altitude=0
        self.sender={}
        self.wa_client = wolframalpha.Client('ETRTW6-QUVU77J24J')
        rospack = rospkg.RosPack()
        self.spotify_pid=0
        self.path=rospack.get_path('budspeech')
        self.urllib_resp_timeout=0.2
        [self.master_IP,self.master_usr,self.master_pwd]=self.load_yaml()
        client_credentials_manager2 = SpotifyClientCredentials('ee9205213e0e439ca670beac2745aeb3','ddac124af2da4c42ab30c91944c92339')
        self.sp = spotipy.Spotify(client_credentials_manager=client_credentials_manager2)

        #Init
        rospy.init_node('brain')
        reached=self.connect_to_master()
        if reached:
            # Init music player
            #pygame.mixer.init()
            # Init Service
            self.enable_rec=rospy.ServiceProxy('enable_recognition', Empty)
            self.disable_rec=rospy.ServiceProxy('disable_recognition', Empty)
            self.stop_disp=rospy.ServiceProxy('stop_disp', Empty)

            try:
                self.enable_rec()
            except:
                rospy.loginfo('no recognition node found')
            # Init Subscribers
            rospy.Subscriber("speech", speech, self.process_speech)
            rospy.Subscriber("temp_pressure/data", temp_pressure_data, self.get_temp_pressure)
            rospy.Subscriber("pressure/event", pressure_event, self.process_pressure_event)
            rospy.Subscriber("imu/event", imu_event, self.process_imu_event)

            # Init CAM publisher
            self.cam_pub = rospy.Publisher('budCAM', String,queue_size=100)
            # Reply publisher
            self.rep_pub = rospy.Publisher('budrep', String,queue_size=100)
            self.disp_emo_pub=rospy.Publisher('disp/emotion', disp_emotion,queue_size=100)
            self.disp_action_pub=rospy.Publisher('disp/action', disp_action,queue_size=100)
            # Welcome greeting
            self.sender={'id':1,'type':'MASTER', 'IP':None, 'room':None, 'usr':None, 'pwd':None}
            self.speak("I am ready!")

    def spin(self):
        rospy.spin()

    def process_imu_event(self,data):
        self.sender={'id':1,'type':'MASTER', 'IP':None, 'room':None, 'usr':None, 'pwd':None}
        if (data.type==imu_event.TYPE_SHAKE_5S):
            rospy.loginfo("Attention! There might be a criminal!")
        if (data.type==imu_event.TYPE_TURN):
            rospy.loginfo("Alarm! Somebody is stealing me!")

    def process_pressure_event(self,data):
        self.sender={'id':1,'type':'MASTER', 'IP':None, 'room':None, 'usr':None, 'pwd':None}

        if (data.type==pressure_event.TYPE_PRESSURE_DROP):
            rospy.loginfo("Door has been opened?")
	    #self.speak("A door has been openend")
        if (data.type==pressure_event.TYPE_PRESSURE_RISE):
            rospy.loginfo("Door has been closed?")
	    #self.speak("A door has been closed")

    def get_temp_pressure(self,data):
        self.current_temp=data.temp
        self.current_altitude=data.altitude
        # Write data to database
        db = MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        cursor.execute('''INSERT INTO data(type,value) VALUES(%s,%s)''', (1,data.temp))
        cursor.execute('''INSERT INTO data(type,value) VALUES(%s,%s)''', (2,data.altitude))
        db.commit();
        db.close();

    def load_yaml(self):
        with open(self.path+'/src/master.yml', 'r') as readfile:
            data=yaml.load(readfile)
            try:
                return [data['IP'],data['usr'],data['pwd']]
            except:
                return ['','','']

    def connect_to_master(self):
        #try to open database connection and look for users
        try:
            db = MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
            cursor = db.cursor()
            cursor.execute("SELECT type FROM objects")
            db.commit();
            db.close();
            return 1
        except MySQLdb.OperationalError as err:
            print('Could not connect to server! Error code '+err[0])
            return 0

    def check_interface(self,interface_id):
        db = MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        try:
            cursor.execute("SELECT inter_type,room,IP,usr,pwd FROM interfaces WHERE id=%s", (interface_id,))
            interface_data=cursor.fetchall()
            cursor.execute("SELECT type FROM interface_types WHERE id=%s", (interface_data[0][0],))
            type_name=cursor.fetchall()[0][0]
            try:
                cursor.execute("SELECT name FROM rooms WHERE id=%s", (interface_data[0][1],))
                room_name=cursor.fetchall()[0][0]
            except:
                room_name=None
            self.sender={'id':interface_id,'type':type_name,'room':room_name, 'IP':interface_data[0][2], 'usr':interface_data[0][3],'pwd':interface_data[0][4]}
        except:
            self.sender={'id':None,'type':None, 'IP':None, 'room':None, 'usr':None, 'pwd':None}
        db.close()

        return self.sender

    def pre_parse(self,token):
        # pre parse tokens for commands
        # give speech to wolfram alpha
        if 'check' in token:
            #try:
                question=' '.join(map(str, token[token.index('check')+1:]))
                replies=['one second', 'one moment', 'let me think', 'okay']
                self.speak(random.choice(replies),1)
                try:
                    data=self.gather_information(question)
                    #answer=next(self.gather_information(question).results).text.split('(')[0]

                    for pod in data.pods:
                        if 'Input interpretation' not in pod.title:
                            answer=pod.text.split('(')[0].split('\n')[0].strip('$').strip('|')
                            #self.speak('The '+pod.title+' is '+answer)
                            self.speak(answer)
                            break
                    if not answer:
                        self.speak('Sorry, I don\'t know')
                except:
                    self.speak('Sorry, I don\'t know')
                return 1
        elif 'cancel' in token:
            self.aw_response=''
            self.num_response=['',0]
            return 1
        else:
            return -1

    def find_objects(self,what,column,string):
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        query="SELECT %s FROM objects WHERE %s = " % (what,column)
        cursor.execute(query+"%s",(string,))
        all_rows = cursor.fetchall()
        # close DB
        db.close()
        return all_rows

    def find_objects_id(self,what,column,ids):
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        query="SELECT %s FROM objects WHERE %s IN " % (what,column)
        cursor.execute(query+"(%s)" % (', '.join(map(str, ids)),))
        all_rows = cursor.fetchall()
        # close DB
        db.close()
        return all_rows

    def find_mods(self,what,column,string):
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        query="SELECT %s FROM mods WHERE %s = " % (what,column)
        cursor.execute(query+"%s" ,(string,))
        all_rows = cursor.fetchall()
        # close DB
        db.close()
        return all_rows

    def check_shout(self,text):
        # checks for shouts and cuts it from the sentence for further processing
        if len(self.aw_response)>0:
            text=self.aw_response+' '+text
            self.aw_response=''
            return text
        if self.shout in text.lower():
            print(text)
            return text[text.lower().find(self.shout)+len(self.shout)+1:]
        else:
            return False

    def process_speech(self,data):
        # called if speech is recognized
        text=False
        # consider that a string was send directly
        self.sender=self.check_interface(data.interface_id)
        text=self.check_shout(data.text)
        if text!=False:
            if text=='':
                if self.sender['type']=='MASTER':
                    self.speak('Yes?')
                if self.sender['type']=='IPCAM':
                    cam_pub.publish('follow')
                self.aw_response=' '
            else:
                token=nltk.word_tokenize(text)
                #tagged=nltk.pos_tag(token)
                if (self.pre_parse(token)!=1):
                    self.handle_input(token,data.interface_id)

    def speak(self,sentence,background=0):
        self.rep_pub.publish(sentence)
        print('Says: '+sentence)
        if (self.sender['type']=='MASTER'):
            if background:
                os.system("pico2wave -w out.wav \""+sentence+"\" && aplay -D hw out.wav &")
            else:
                try:
                    self.disable_rec()
                except:
                    rospy.loginfo('recognition node not found!')
                # Show animation
                msg=disp_action()
                msg.type=disp_action.SPEAKING
                self.disp_action_pub.publish(msg)
                os.system("pico2wave -w out.wav \""+sentence+"\" && aplay -D hw out.wav")
                try:
                    self.stop_disp()
                except:
                    rospy.loginfo('Display node not available')
                rospy.sleep(0.7)
                try:
                    self.enable_rec()
                except:
                    rospy.loginfo('recognition node not found!')

        elif self.sender['type']=='IPCAM':
            if '?' in sentence:
                self.cam_pub.publish('shake')
                self.aw_response=' '
        #time.sleep(0.2)


    def check_keys(self,token,keys):
        # checks which keys are mentioned in sentence, returns list
        keys_found=[]
        for x in keys:
            if x in token:
                keys_found.append(x)
        return keys_found


    def read_poss_tasks(self):
        # collects all possible tasks on objects
        #connect to DB
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT modification FROM mods")
        all_rows = cursor.fetchall()
        all_rows_nt=[]
        for row in all_rows:
            all_rows_nt.append(row[0])
        # close DB
        db.close()
        # delete identical entries
        return list(set(all_rows_nt))

    def handle_input(self,token,interface_id):
        # perform tasks
        # Mandatory: What kind of TASK? (Verb), optional: WHAT, WHERE, WHEN? # asks back if necessary but missing
        # check if there is an modifiable object mentioned and a possible task
        #connect to DB
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        self.sender=self.check_interface(interface_id)
        print(self.sender)
        tasks=self.read_poss_tasks() # all possible tasks
        # check which tasks are found in NL
        tasks_found=[]
        # add 'yes' to tokens to avoid quesetion when using an ipcam
        if self.sender['type']=='IPCAM':
            token.append('yes')
        print(tasks)
        for task in tasks:
            if list(set(self.check_keys(token,nltk.word_tokenize(task))))==nltk.word_tokenize(task):
                tasks_found.append(task)
        if tasks_found:
            # check if there is mentioned an object which supports this task
            for task in tasks_found:
                # generate possible object names:
                obj_id_poss=list(sum(self.find_mods('objectID','modification',task), ()))
                obj_poss=list(set(list(sum(self.find_objects_id('type','id',obj_id_poss), ()))))
                obj_poss_pl=[object+'s' for object in obj_poss]
                obj_found=self.check_keys(token,obj_poss+obj_poss_pl)
                #print(str(obj_id_poss))
                result=0
                if len(obj_found)>0:
                    for object in obj_found:
                        obj_pl=0
                        object_name=object
                        if object in obj_poss_pl:
                            obj_pl=1
                            object=object[:-1]
                        # modify all objects if plural and 'all' is said
                        if obj_pl and 'all' in token:
                            query="SELECT id FROM objects WHERE id IN (%s) AND type = " % (', '.join(map(str, obj_id_poss),),)
                            cursor.execute(query+"%s" , (object,))
                            ids=list(sum(cursor.fetchall(), ()))
                            result=self.perform_mod(token,object,ids,task,self.sender)
                            if result==-1:
                                self.speak('I can\'t '+task+' the '+object_name)
                            elif result==0:
                                self.speak('I couldn\'t '+task+' all of the '+object_name)
                        else:
                            query="SELECT room,location,how FROM objects WHERE id IN (%s) AND type = " % (', '.join(map(str, obj_id_poss),),)
                            cursor.execute(query+"%s" , (object,))
                            comb_poss=cursor.fetchall()  # generate from object list
                            if len(comb_poss)==1:
                                #Only one object found
                                query="SELECT id FROM objects WHERE id IN (%s) AND type = " % (', '.join(map(str, obj_id_poss),),)
                                cursor.execute(query+"%s",(object,))
                                id=list(sum(cursor.fetchall(), ()))
                                result=self.perform_mod(token,object,id,task,self.sender)
                                if result==-1:
                                        self.speak('I can\'t '+task+' the '+object_name)
                                break
                            else:
                                # more object found, look for ways to distinguish
                                loc_poss=[loc[1] for loc in comb_poss]
                                rooms_poss=[room[0] for room in comb_poss]
                                rooms_poss_names=[]
                                for room in rooms_poss:
                                    # get rooms name
                                    cursor.execute("SELECT name FROM rooms WHERE id=%s",(room,))
                                    rooms_poss_names.append(nltk.word_tokenize(cursor.fetchall()[0][0])[0])
                                how_poss=[how[2] for how in comb_poss]
                                loc_found=self.check_keys(token,loc_poss)
                                rooms_found=self.check_keys(token,rooms_poss_names)
                                how_found=self.check_keys(token,how_poss)
                                more_info=-1

                                if len(loc_found)+len(rooms_found)+len(how_found)>0:
                                    # check the objects features
                                    query="SELECT id FROM objects WHERE id IN (%s)"
                                    query_var=[', '.join(map(str, obj_id_poss),)]
                                    if (len(rooms_found)>0):
                                        query=query+" AND room IN (%s)"
                                        query_var.append(', '.join(map(str, ['\''+str(rooms_poss[rooms_poss_names.index(room)])+'\'' for room in rooms_found ]),))
                                        self.aw_response=self.aw_response + ' ' + ' '.join(map(str, rooms_found))
                                    if (len(loc_found)>0):
                                        query=query+" AND location IN (%s)"
                                        query_var.append(', '.join(map(str, ['\''+loc+'\'' for loc in loc_found]),))
                                        self.aw_response=self.aw_response + ' ' + ' '.join(map(str, loc_found))
                                    if (len(how_found)>0):
                                        query=query+" AND how IN (%s)"
                                        query_var.append(', '.join(map(str, ['\''+how+'\'' for how in how_found]),))
                                        self.aw_response=self.aw_response + ' ' + ' '.join(map(str, how_found))
                                    query=query % tuple(query_var)
                                    cursor.execute(query+" AND type = %s",(object,))
                                    id=list(sum(cursor.fetchall(), ()))
                                    if obj_pl==1 or len(id)==1:
                                        # turn on all the object with the feature
                                        result=self.perform_mod(token,object,id,task,self.sender)
                                        self.aw_response=''
                                        self.num_response=['',0]
                                        if result==-1:
                                            self.speak('I can\'t '+task+' the '+object_name)
                                    else:
                                        more_info=len(loc_found)+len(rooms_found)+len(how_found)
                                if (len(loc_found)+len(rooms_found)+len(how_found)==0) or more_info>-1:

                                    if self.num_response[0]=='feat' and self.num_response[1]>1:
                                        self.num_response=['',0]
                                        self.aw_response=''
                                        self.speak('Sorry, I don\'t know that '+object_name+'!')
                                    else:
                                        if self.num_response[1]==1 and self.num_response[0]=='feat':
                                            self.speak('Could you repeat?')
                                        else:
                                            if more_info>=1:
                                                self.speak('Which '+object_name +' do you mean exactly?')
                                            else:
                                                self.speak('Which '+object_name +' do you mean?')

                                        self.aw_response=self.aw_response+' '+task+' '+object_name
                                        self.num_response=['feat',self.num_response[1]+1]
                else:
                    if self.num_response[0]=='obj' and self.num_response[1]>1:
                        self.num_response=['',0]
                        self.aw_response=''
                        self.speak('Sorry, I don\'t know this object!')
                    else:
                        if self.num_response[1]==1 and self.num_response[0]=='obj':
                            self.speak('Could you repeat?')
                        else:
                            self.speak('Which object shall I ' + task + '?')
                        self.aw_response=self.aw_response+' '+task
                        self.num_response=['obj',self.num_response[1]+1]
        else:
            # Give an answer
            self.speak('Sorry, I don\'t understand!')
            if self.sender['type']=='IPCAM':
                cam_pub.publish('shake')
        # give answers
        db.close()

    def play_confirm(self):
        # Play confirm sound
        print("Not implemented")
        #pygame.mixer.music.load(self.path+"/src/confirm.ogg")
        #pygame.mixer.music.set_volume(0.7)
        #pygame.mixer.music.play()

    def perform_mod(self,token,object,objectIDs,task,sender):
        result=1
        ## TURN ON, TURN OFF OBJECT or POWER
        if task=='turn on':
            if object!='power':
                result=self.turn_on_off(objectIDs,'on')
            else:
                if self.POWER==0:
                    try:
                        self.enable_rec()
                    except:
                        rospy.loginfo('recognition node not found!')
                    result= self.switch_power('on')
                else:
                    result= -1
        elif task=='turn off':
            if object!='power':
                result= self.turn_on_off(objectIDs,'off')
            else:
                if self.POWER==1:
                    try:
                        self.disable_rec()
                    except:
                        rospy.loginfo('recognition node not found!')
                    result= self.switch_power('off')
                else:
                    result= -1
        ## FLASH THE LIGHT
        elif task=='flash' and object=='light':
            result= self.flash_light(objectIDs,0.2,8)
        ## Say something
        elif task=='say' and object=='you':
            self.sender={'id':1,'type':'MASTER', 'IP':None, 'room':None, 'usr':None, 'pwd':None}
            self.speak(' '.join(token[token.index('say')+1:]))
        ## SET TIMER // ALARM
        elif task=='set':
            if object=='timer':
                duration=self.get_time(token,task,object,object,['For how long?'])
                if duration>0:
                    rospy.Timer(rospy.Duration(duration), self.result_timer, True)
            elif object=='alarm':
                duration=self.get_time(token,task,object,object,['When?'])
                if duration>0:
                    self.speak('Set alarm to %d hours and %d minutes.' % ((duration/3600),(duration/60)%60))
                    rospy.Timer(rospy.Duration(duration), self.result_alarm, True)
        ## PLAY,STOP,CHANGE MUSIC # only if master
        elif task=='play':
            try:
                song=' '.join(token[token.index('song')+1:])
                print(song)
            except:
                song="don't worry be happy"
                pass
            result = self.sp.search(song,type='track',limit=1)
            track_id=result['tracks']['items'][0]['id']
            print('Playing '+song)
            self.spotify_pid = subprocess.Popen([self.path+"/src/librespot", track_id])
        elif task=='stop':
            if self.spotify_pid is not 0:
                self.spotify_pid.kill()
                self.spotify_pid=0
            else:
                result= -1
        ## WEATHER
        elif (task=='is' or task=='tell' or task=='\'s') and object=='weather' and sender['type']!='IPCAM':
            result= self.check_weather('Munich, Germany',token,1)
        ## GO TO BED
        elif task=='go to' or task=='sleep' and sender['type']!='IPCAM':
            self.prepare_sleep_mode(token,task,object)
        ## MOVE CAMERA
        elif task=='move' and object=='camera':
            result=self.move_camera(token)
        ## LEAVE THE HOUSE
        elif task=='leave' and object=='I':
            result=self.leave_house()
        ## GET TEMPERATUR
        elif (task=='is' or task=='\'s') and object=='temperature':
            result=self.give_temperature()
        ## GET ALTITUDE
        elif (task=='is' or task=='\'s') and object=='altitude':
            result=self.give_altitude()
        ## BACK HOME
        elif task=='\'m back' or 'am back' and object=='I':
            result=self.back_home()
        else:
            result=-1
            self.speak('I don''t know how to '+ task +' the '+object)
        if result==1:
            if sender['type']=='IPCAM':
                cam_pub.publish('nod')
            elif self.sender['type']=='MASTER':
                self.play_confirm()
            else:
                self.rep_pub.publish('done!')
        else:
            if self.sender['type']=='IPCAM':
                cam_pub.publish('shake')
        return result

    def give_temperature(self):
        self.speak("Currently it has "+str(self.current_temp)+" degrees!")

    def give_altitude(self):
        self.speak("We're at a height of "+str(int(round(self.current_altitude)))+" meters!")

    def leave_house(self):
        result=self.switch_power('off')
        # turn off music
        if radio_pid>=0:
            radio.kill_mplayer(radio_pid)
            radio_pid=-1
        else:
            result= -1
        # activate camera stream
        cam_pub.publish('start_stream')
        return result

    def back_home(self):
        result=self.switch_power('on')
        # deactivate camera stream
        cam_pub.publish('stop_stream')
        return result

    def gather_information(self,question):
        # ask wolfram alpha for answer
        msg=disp_action()
        msg.type=disp_action.PROCESSING
        self.disp_action_pub.publish(msg)
        result=self.wa_client.query(question)
        try:
            self.stop_disp()
        except:
            rospy.loginfo('Display node not available')
        return result

    def check_genre(self,token):
        love=['Radio Center Love','JazzRadio Budapest']
        mainstream=['SWR3 Radio','CampusFM Essen Germany']
        channel=-1
        if 'love' in token:
            channel=random.choice(love)
        elif 'mainstream' in token:
            channel=random.choice(mainstream)
        elif 'rock' in token:
            channel='Rocky FM'
        elif 'country' in token:
            channel='Country 108'
        elif 'hop' in token and 'hip' in token:
            channel='105HipHop'
        return channel

    def prepare_sleep_mode(self,token,task,object):
        duration=self.get_time(token,task,object,'alarm',['Okay! When do you want to get up?'])
        if duration>0:
            self.speak('Enjoy your %d hours and %d minutes of sleep. Good night!' % ((duration/3600),(duration/60)%60))
            rospy.Timer(rospy.Duration(duration), self.wakeup_mode, True)
            self.switch_power('off')
            self.check_interface()

    def result_alarm(self,event):
        self.speak('Alarm!')
        #self.turn_on_off([1,2],'on')

    def wakeup_mode(self,event):
        # Play wakeup music:
        #pygame.mixer.music.load(self.path+"/music/wakeup1.mp3")
        #pygame.mixer.music.play()
        #fade in music
        for vol in range(0,100):
            #pygame.mixer.music.set_volume(float(vol)/100)
            rospy.sleep(0.15)
        rospy.sleep(70)
        #pygame.mixer.music.fadeout(10000)
        #while pygame.mixer.music.get_busy() == True:
        #    continue
        rospy.sleep(1)
        message=speech()
        message.confidence=1
        message.text='data: '+self.shout+' '+'turn on the light at the bed'
        self.process_speech(message)
        self.speak('Good morning!')
        self.check_weather('Munich, Germany', [],1)

    def get_time(self,token,task,object,real_obj,questions):
        # gets the time for setting the timer or alarm (real_obj)
        # task and object describe the makro task and object (can be the same)
        global aw_response
        global num_response
        keys=['second','minute','hour','day']
        keys2=['o\'clock','a.m','p.m']
        keys_plural=[key+'s' for key in keys]
        time_found=[]
        duration=0
        for tok in token:
        # check for :
            if ':' in tok:
                time_found.append([tok,''])
        for x in keys+keys_plural+keys2:
            if x in token:
                try:
                    number=int(token[token.index(x)-1])
                    time_found.append([number,x])
                except:
                    pass
        if time_found==[]:
            if self.num_response[0]=='time' and self.num_response[1]>1:
                self.num_response=['',0]
                self.aw_response=''
                self.speak('Sorry, I don\'t understand!')
                return -1
            else:
                if self.num_response[1]==1 and self.num_response[0]=='time':
                    self.speak('Could you repeat?')
                    self.aw_response=self.aw_response+' '+task+' '+object
                    self.num_response=['time',self.num_response[1]+1]
                else:
                    self.speak(questions[0])
                    self.aw_response=self.aw_response+' '+task+' '+object
                    self.num_response=['time',self.num_response[1]+1]
                return 0
        else:
            self.num_response=['',0]
            time_string=''
            for time in time_found:
                if time[1]!='':
                    time_string=time_string+' '+str(time[0])+' '+time[1]
                else:
                    time_string=time_string+' '+str(time[0])
            decision=self.check_keys([tok.lower() for tok in token],['yes','no'])
            if decision==[]:
                self.speak('set'+' the '+real_obj+' to'+time_string+'?')
                self.aw_response=self.aw_response+' '+task+' '+object+' '+time_string
            elif decision[0]=='yes':
                duration=self.convert_time_to_sec(time_found)
            else:
                self.process_speech('data: '+self.shout+' '+task+' '+object)
            return duration

    def convert_time_to_sec(self,time_list):
        duration=0
        # get current time:
        now = rospy.Time.now().to_sec()
        hours= int(((now/3600)+2)%24)
        minutes=int((now/60)%60)
        seconds=int(now%60)
        for time in time_list:
            if time[1]=='a.m' or time[1]=='o\'clock':
                time[0]=str(time[0])+':0'
                time[1]=''
            elif time[1]=='p.m':
                time[0]=str(time[0]+12)+':0'
                time[1]=''
        for time in time_list:
            if time[1]=='second' or time[1]=='seconds':
                duration=duration+time[0]
            elif time[1]=='minute' or time[1]=='minutes':
                duration=duration+time[0]*60
            elif time[1]=='hour' or time[1]=='hours':
                duration=duration+time[0]*60*60
            elif time[1]=='day' or time[1]=='days':
                duration=duration+time[0]*60*60*60
            elif time[1]=='':
                hours_set=int(time[0].split(':')[0])
                minutes_set=int(time[0].split(':')[1])
                if (hours-(hours_set))>0:
                    hours_togo=24-hours+hours_set
                else:
                    hours_togo=hours_set-hours
                if (minutes-minutes_set)>0:
                    minutes_togo=60-minutes+minutes_set
                    hours_togo=hours_togo-1
                else:
                    minutes_togo=minutes_set-minutes
                if (seconds-0)>0:
                    seconds_togo=60-seconds-1
                    minutes_togo=minutes_togo-1
                duration=hours_togo*3600+minutes_togo*60+seconds_togo
        return duration

    def move_camera(self,token):
        if 'up' in token:
            cam_pub.publish('up')
        elif 'down' in token:
            cam_pub.publish('down')
        elif 'left' in token:
            cam_pub.publish('left')
        elif 'right' in token:
            cam_pub.publish('right')
        else:
            return -1

    def check_weather(self,location,token,current):
        # current=True: give current weather
        day=0
        if 'tomorrow' in token:
            day=1
            current=0
            token.remove('tomorrow')
        if 'in' in token:
            try:
                location=' '.join(token[token.index('in')+1:])
            except:
                pass
        location_id=pywapi.get_loc_id_from_weather_com(location)
        city=location.split(',')[0]
        if type(location_id) is dict:
            try:
                city=location_id[0][1].split(',')[0]
                location_id=location_id[0][0]
            except:
                pass

        try:
            data=pywapi.get_weather_from_weather_com( location_id , units = 'metric' )
            rain=data['forecasts'][day]['day']['chance_precip']
            uv=data['current_conditions']['uv']['index']
            if current:
                self.speak('Currently it has ' + data['current_conditions']['temperature'] + ' degrees in ' + city + ' and it\'s ' + data['current_conditions']['text'])
                if int(uv)>=5:
                    self.speak('Attention! The current UV index is '+uv+', so don\'t forget the sunscreen.')
            if day==1:
                msg='Tomorrow\'s high in ' +city+' is ' + data['forecasts'][day]['high'] + ' degrees, low temperature is ' + data['forecasts'][day]['low']
                if (data['forecasts'][day]['day']['text']!=""):
                    msg+=', and it will be ' +data['forecasts'][day]['day']['text']
                self.speak(msg)
            else:
                msg='Today\'s high is ' + data['forecasts'][day]['high'] + ' degrees, low temperature is ' + data['forecasts'][day]['low']
                if (data['forecasts'][day]['day']['text']!=""):
                    msg+=', and it will be ' +data['forecasts'][day]['day']['text']
                self.speak(msg)
            if int(rain)>=50:
                self.speak('Attention! The probability of rain is '+rain+'%, so better take an umbrella.')
        except:
            self.speak('Sorry, I do not know the weather in '+city)

    def result_timer(self,event):
        self.speak('Timer exceeded!')
        ## flash the lights
        ids_t=self.find_objects('id','type','light')
        ids=[id[0] for id in ids_t]
        self.flash_light(ids,0.3,4)

    def flash_light(self,objectIDs,delay,iterations):
        result=self.turn_on_off(objectIDs,'state')
        if result!=-1:
            state=[]
            for id in objectIDs:
                state.append(self.turn_on_off([id],'state'))
            for i in range (0,iterations):
                rospy.sleep(delay)
                self.turn_on_off(objectIDs,'on')
                rospy.sleep(delay)
                self.turn_on_off(objectIDs,'off')
            rospy.sleep(delay)
            for light in range(0,len(objectIDs)):
                if state[light]==2:
                    self.turn_on_off([objectIDs[light]],'off')
                elif state[light]==3:
                    self.turn_on_off([objectIDs[light]],'on')
        return result

    def switch_power(self,on_off):
        #switches power on_off, returns to previous state
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id,IP,state FROM sockets")
        IP=cursor.fetchall()  # generate from object list
        global POWER
        for ip in IP:
            if on_off=='on':
                try:
                    if int(ip[2])==1:
                        urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?on', timeout=self.urllib_resp_timeout)
                    else:
                        urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?off', timeout=self.urllib_resp_timeout)
                    self.POWER=1
                except:
                    print('cannot switch the power on')
            elif on_off=='off':
                try:
                    state=urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?state', timeout=self.urllib_resp_timeout).read()
                    if int(state)==1:
                        urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?off', timeout=self.urllib_resp_timeout)
                    cursor.execute("UPDATE sockets SET state = %s WHERE id = %s", (int(state),ip[0],))
                    self.POWER=0
                except:
                    print('cannot switch the power off')
        db.commit()
        db.close()

    def turn_on_off(self,objectIDs, on_off):
        #switch state of socket
        db= MySQLdb.connect(self.master_IP,self.master_usr,self.master_pwd,'FB')
        cursor = db.cursor()
        result=-1
        error=0
        for objectID in objectIDs:
            cursor.execute("SELECT IP FROM sockets WHERE objectID = %s",(objectID,))
            IP=list(sum(cursor.fetchall(), ()))  # generate from object list
            try:
                if on_off=='on':
                    urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?on', timeout=self.urllib_resp_timeout)
                    result=1
                elif on_off=='state':
                    result=int(urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?state', timeout=self.urllib_resp_timeout).read())+2
                else:
                    urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?off', timeout=self.urllib_resp_timeout)
                    result=1
            except:
                error=1
        db.close()
        if error==1 and result==1:
            return 0
        else:
            return result

if __name__ == '__main__':
        brain=brain_node()
        brain.spin()
