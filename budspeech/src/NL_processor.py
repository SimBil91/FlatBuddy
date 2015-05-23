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
import radiopy
import pygame
import os
import pywapi
import random
import wolframalpha
import yaml
#from nltk.corpus import wordnet as wn
from std_msgs.msg import String
from std_srvs.srv import Empty

# GLOBAL variables
conf_thres=0.5
shout='hey buddy'
aw_response=''
num_response=['',0]
path=rospkg.RosPack().get_path('budspeech')
radio=radiopy.Player()
radio_pid=-1
POWER=1
client = wolframalpha.Client('ETRTW6-QUVU77J24J')

def load_yaml():
    with open('master.yml', 'r') as readfile:
        data=yaml.load(readfile)
        try:
            return [data['IP'],data['usr'],data['pwd']]
        except:
            return ['','','']

[master_IP,master_usr,master_pwd]=load_yaml()

def connect_to_master():
    #try to open database connection and look for users
    try: 
        db = MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT type FROM objects")
        db.commit();
        db.close();
        return 1
    except MySQLdb.OperationalError as err:
        print('Could not connect to server! Error code '+err[0])
        return 0
    
def pre_parse(token):
    global aw_response
    global num_response
    # pre parse tokens for commands
    # give speech to wolfram alpha
    if 'check' in token:
        #try:
            question=' '.join(map(str, token[token.index('check')+1:]))
            replies=['one second', 'one moment', 'let me think', 'okay']
            speak(random.choice(replies),1)
            try:
                data=gather_information(question)
                #answer=next(gather_information(question).results).text.split('(')[0]

                for pod in data.pods:
                    if 'Input interpretation' not in pod.title:
                        answer=pod.text.split('(')[0].split('\n')[0].strip('$').strip('|')
                        #speak('The '+pod.title+' is '+answer)
                        speak(answer)
                        break
                if not answer:
                    speak('Sorry, I don\'t know')
            except:
                speak('Sorry, I don\'t know')
            return 1
    elif 'cancel' in token:
        aw_response=''
        num_response=['',0]
        return 1
    else:
        return -1
def find_objects(what,column,string):
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    cursor.execute("SELECT %s FROM objects WHERE %s = %s" ,(what,column,string,))
    all_rows = cursor.fetchall() 
    # close DB
    db.close()   
    return all_rows

def find_objects_id(what,column,ids):
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    cursor.execute("SELECT %s FROM objects WHERE %s IN (%s)" , (what,column,', '.join(map(str, ids))))
    all_rows = cursor.fetchall() 
    # close DB
    db.close()   
    return all_rows

def find_mods(what,column,string):
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    cursor.execute("SELECT %s FROM mods WHERE %s = %s" , (what,column,string,))
    all_rows = cursor.fetchall() 
    # close DB
    db.close()   
    return all_rows
def check_shout(text): 
    # checks for shouts and cuts it from the sentence for further processing
    global aw_response
    if len(aw_response)>0:
        text=aw_response+' '+text
        aw_response=''
        return text
    if shout in text.lower():
        return text[text.lower().find(shout)+len(shout)+1:]
    else: 
        return False
    
def process_speech(data):
    # called if speech is recognized
    global aw_response
    text=''
    try:
        sentence=eval(str(data)[5:]) # dict is read
    except:
        # consider that a string was send directly
        sentence={'text':str(data)[5:],'confidence':0}
    if sentence['confidence']==0: 
        sentence['confidence']=1
    if sentence['confidence'] > conf_thres:
        text=check_shout(sentence['text'])#
    if text!=False:
        if text=='':
            speak('Yes?')
            aw_response=' '
        else:
            token=nltk.word_tokenize(text)
            #tagged=nltk.pos_tag(token)
            tagged=[]
            if (pre_parse(token)!=1):
                handle_input(tagged,token)

        

def speak(sentence,background=0):
    #soundhandle.voiceSound(sentence).play()
    #engine.say(sentence)
    #engine.runAndWait()
    print('Says: '+sentence)
    if background:
        os.system('echo \"'+sentence+'\" | festival --tts &')
    else:
        disable_rec()
        os.system('echo \"'+sentence+'\" | festival --tts')
        rospy.sleep(0.7)
        enable_rec()
    #rospy.sleep(0.5*len(sentence))

def check_keys(token,keys):
    # checks which keys are mentioned in sentence, returns list
    keys_found=[]
    for x in keys:
        if x in token:
            keys_found.append(x)
    return keys_found
	

def read_poss_tasks():
    # collects all possible tasks on objects
    #connect to DB
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
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
    

def handle_input(tagged,token):
    # perform tasks
    # Mandatory: What kind of TASK? (Verb), optional: WHAT, WHERE, WHEN? # asks back if necessary but missing
    # check if there is an modifiable object mentioned and a possible task
    #connect to DB
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    global aw_response
    global num_response
    tasks=read_poss_tasks() # all possible tasks
    # check which tasks are found in NL
    tasks_found=[]
    for task in tasks:
        if list(set(check_keys(token,nltk.word_tokenize(task))))==nltk.word_tokenize(task):
            tasks_found.append(task)
    if tasks_found:
        # check if there is mentioned an object which supports this task
        for task in tasks_found:   
            # generate possible object names:  
            obj_id_poss=list(sum(find_mods('objectID', 'modification',task), ()))
            obj_poss=list(set(list(sum(find_objects_id('type','id',obj_id_poss), ()))))
            obj_poss_pl=[object+'s' for object in obj_poss]
            obj_found=check_keys(token,obj_poss+obj_poss_pl)
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
                        cursor.execute("SELECT id FROM objects WHERE id IN (%s) AND type = %s" % (', '.join(map(str, obj_id_poss),),),(object,))
                        ids=list(sum(cursor.fetchall(), ())) 
                        result=perform_mod(token,object,ids,task)
                        if result==-1:
                            speak('I can\'t '+task+' the '+object_name) 
                        elif result==0:
                            speak('I couldn\'t '+task+' all of the '+object_name) 
                    else:
                        cursor.execute("SELECT room,location,how FROM objects WHERE id IN (%s) AND type = %s" % (', '.join(map(str, obj_id_poss),),),(object,))
                        comb_poss=cursor.fetchall()  # generate from object list
                        if len(comb_poss)==1:
                            #Only one object found
                            cursor.execute("SELECT id FROM objects WHERE id IN (%s) AND type = %s" % (', '.join(map(str, obj_id_poss),),),(object,))
                            id=list(sum(cursor.fetchall(), ())) 
                            result=perform_mod(token,object,id,task)
                            if result==-1:
                                    speak('I can\'t '+task+' the '+object_name) 
                            break
                        else: 
                            # more object found, look for ways to distinguish
                            loc_poss=[loc[1] for loc in comb_poss]
                            rooms_poss=[room[0] for room in comb_poss]
                            how_poss=[how[2] for how in comb_poss]
                            loc_found=check_keys(token,loc_poss)
                            rooms_found=check_keys(token,rooms_poss)
                            how_found=check_keys(token,how_poss)
                            more_info=-1
                            if len(loc_found)+len(rooms_found)+len(how_found)>0:
                                # check the objects features
                                query="SELECT id FROM objects WHERE id IN (%s)"
                                query_var=[', '.join(map(str, obj_id_poss),)]
                                if (len(rooms_found)>0): 
                                    query=query+" AND room IN (%s)"
                                    query_var.append(', '.join(map(str, ['\''+room+'\'' for room in rooms_found]),))
                                    aw_response=aw_response + ' ' + ' '.join(map(str, rooms_found))
                                if (len(loc_found)>0): 
                                    query=query+" AND location IN (%s)"
                                    query_var.append(', '.join(map(str, ['\''+loc+'\'' for loc in loc_found]),))
                                    aw_response=aw_response + ' ' + ' '.join(map(str, loc_found))
                                if (len(how_found)>0): 
                                    query=query+" AND how IN (%s)"
                                    query_var.append(', '.join(map(str, ['\''+how+'\'' for how in how_found]),))
                                    aw_response=aw_response + ' ' + ' '.join(map(str, how_found))
                                query=query+" AND type = %s"
                                cursor.execute(query % tuple(query_var),(object,))
                                id=list(sum(cursor.fetchall(), ())) 
                                if obj_pl==1 or len(id)==1:
                                    # turn on all the object with the feature
                                    result=perform_mod(token,object,id,task)
                                    aw_response=''
                                    num_response=['',0]
                                    if result==-1:
                                        speak('I can\'t '+task+' the '+object_name+' at the '+ location) 
                                else:
                                    more_info=len(loc_found)+len(rooms_found)+len(how_found)
                            if (len(loc_found)+len(rooms_found)+len(how_found)==0) or more_info>-1:
                                
                                if num_response[0]=='feat' and num_response[1]>1:
                                    num_response=['',0]
                                    aw_response=''
                                    speak('Sorry, I don\'t know that '+object_name+'!')
                                else:
                                    if num_response[1]==1 and num_response[0]=='feat':
                                        speak('Could you repeat?')
                                    else:
                                        if more_info>=1:
                                            speak('Which '+object_name +' do you mean exactly?')
                                        else:
                                            speak('Which '+object_name +' do you mean?')
                        
                                    aw_response=aw_response+' '+task+' '+object_name
                                    num_response=['feat',num_response[1]+1]
            else:
                if num_response[0]=='obj' and num_response[1]>1:
                    num_response=['',0]
                    aw_response=''
                    speak('Sorry, I don\'t know this object!')
                else:
                    if num_response[1]==1 and num_response[0]=='obj':
                        speak('Could you repeat?')
                    else:
                        speak('Which object shall I ' + task + '?')
                    aw_response=aw_response+' '+task
                    num_response=['obj',num_response[1]+1]
    else:
        # Give an answer
        speak('Sorry, I don\'t understand!')
    # give answers
    db.close()
def play_confirm():
    # Play confirm sound   
    pygame.mixer.music.load(path+"/src/confirm.ogg")
    pygame.mixer.music.set_volume(0.7)
    pygame.mixer.music.play()
    
def perform_mod(token,object,objectIDs,task):
    global radio
    global radio_pid
    play_confirm()
    ## TURN ON, TURN OFF OBJECT or POWER
    if task=='turn on':
        if object!='power':
            return turn_on_off(objectIDs,'on')
        else:
            if POWER==0:
                enable_rec()
                return switch_power('on')
            else:
                return -1
    elif task=='turn off':
        if object!='power':
            return turn_on_off(objectIDs,'off')
        else:
            if POWER==1:
                disable_rec()
                return switch_power('off')
            else:
                return -1
    ## PLAY,STOP,CHANGE MUSIC
    elif task=='play':
        if radio_pid==-1:
            channel=check_genre(token)
            if channel==-1:
                radio_pid=radio.play_random()
            else:
                radio_pid=radio.play(channel)
        else: 
            return -1
    elif task=='stop':
        if radio_pid>=0:
            radio.kill_mplayer(radio_pid)
            radio_pid=-1
        else:
            return -1
    elif task=='change':
        if radio_pid>=0:
            radio.kill_mplayer(radio_pid)
            radio_pid=radio.play_random()
        else:
            return -1
    ## WEATHER
    elif (task=='is' or task=='tell' or task=='\'s') and object=='weather':
        return check_weather('Munich, Germany',token,1)
    ## FLASH THE LIGHT
    elif task=='flash' and object=='light':
        return flash_light(objectIDs,0.2,8)
    ## SET TIMER // ALARM
    elif task=='set':
        if object=='timer':
            duration=get_time(token,task,object,object,['For how long?'])
            if duration>0:
                rospy.Timer(rospy.Duration(duration), result_timer, True)
        elif object=='alarm':
            duration=get_time(token,task,object,object,['When?'])
            if duration>0:
                speak('Set alarm to %d hours and %d minutes.' % ((duration/3600),(duration/60)%60)) 
                rospy.Timer(rospy.Duration(duration), result_alarm, True)
    ## GO TO BED
    elif task=='go to' or task=='sleep':
        prepare_sleep_mode(token,task,object)
    else:
        speak('I don''t know how to '+ task +' the '+object)

def gather_information(question):
    global client
    # ask wolfram alpha for answer
    return client.query(question)

def check_genre(token):
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

def prepare_sleep_mode(token,task,object):
    duration=get_time(token,task,object,'alarm',['Okay! When do you want to get up?'])
    if duration>0:
        speak('Enjoy your %d hours and %d minutes of sleep. Good night!' % ((duration/3600),(duration/60)%60)) 
        rospy.Timer(rospy.Duration(duration), wakeup_mode, True)
        switch_power('off')
        disable_rec()
def result_alarm(event):
    speak('Alarm!')
    #turn_on_off([1,2],'on')
def wakeup_mode(event):
    global shout
    # Play wakeup music:      
    pygame.mixer.music.load(path+"/music/wakeup1.mp3")
    pygame.mixer.music.play()
    #fade in music
    for vol in range(0,100):
        pygame.mixer.music.set_volume(float(vol)/100)
        rospy.sleep(0.15)
    rospy.sleep(70)
    pygame.mixer.music.fadeout(10000)
    while pygame.mixer.music.get_busy() == True:
        continue
    rospy.sleep(1)
    process_speech('data: '+shout+' '+'turn on the light at the bed')
    speak('Good morning!')
    check_weather('Munich, Germany', [],1)
def get_time(token,task,object,real_obj,questions):
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
        if num_response[0]=='time' and num_response[1]>1:
            num_response=['',0]
            aw_response=''
            speak('Sorry, I don\'t understand!')
            return -1
        else:
            if num_response[1]==1 and num_response[0]=='time':
                speak('Could you repeat?')
                aw_response=aw_response+' '+task+' '+object
                num_response=['time',num_response[1]+1]  
            else:
                speak(questions[0])
                aw_response=aw_response+' '+task+' '+object
                num_response=['time',num_response[1]+1]  
            return 0
    else:
        num_response=['',0]
        time_string=''
        for time in time_found:
            if time[1]!='':
                time_string=time_string+' '+str(time[0])+' '+time[1]
            else:
                time_string=time_string+' '+str(time[0])
        decision=check_keys([tok.lower() for tok in token],['yes','no'])
        if decision==[]:
            speak('set'+' the '+real_obj+' to'+time_string+'?')
            aw_response=aw_response+' '+task+' '+object+' '+time_string
        elif decision[0]=='yes':
            duration=convert_time_to_sec(time_found)
        else:
            process_speech('data: '+shout+' '+task+' '+object)
        return duration
    
def convert_time_to_sec(time_list):
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

def check_weather(location,token,current):
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
            speak('Currently it has ' + data['current_conditions']['temperature'] + ' degrees in ' + city + ' and it\'s ' + data['current_conditions']['text'])
            if int(uv)>=5:
                speak('Attention! The current UV index is '+uv+', so don\'t forget the sunscreen.')
        if day==1:
            speak('Tomorrow\'s high in ' +city+' is ' + data['forecasts'][day]['high'] + ' degrees, low temperature is ' + data['forecasts'][day]['low'] + ', and it will be ' + data['forecasts'][day]['day']['text'])
        else:
            speak('Today\'s high is ' + data['forecasts'][day]['high'] + ' degrees, low temperature is ' + data['forecasts'][day]['low'] + ', and it will be ' + data['forecasts'][day]['day']['text'])
        if int(rain)>=50:
            speak('Attention! The probability of rain is '+rain+'%, so better take an umbrella.')
    except:
        speak('Sorry, I do not know the weather in '+city)
    
def result_timer(event):
    speak('Timer exceeded!')
    ## flash the lights
    ids_t=find_objects('id','type','light')
    ids=[id[0] for id in ids_t]
    flash_light(ids,0.3,4)

def flash_light(objectIDs,delay,iterations):
    result=turn_on_off(objectIDs,'state')
    if result!=-1:
        state=[]
        for id in objectIDs:
            state.append(turn_on_off([id],'state'))
        for i in range (0,iterations):
            rospy.sleep(delay)
            turn_on_off(objectIDs,'on')
            rospy.sleep(delay)
            turn_on_off(objectIDs,'off')
        rospy.sleep(delay)
        for light in range(0,len(objectIDs)):
            if state[light]==2:
                turn_on_off([objectIDs[light]],'off')
            elif state[light]==3:
                turn_on_off([objectIDs[light]],'on')
    return result
def switch_power(on_off):
    #switches power on_off, returns to previous state
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    cursor.execute("SELECT id,IP,state FROM sockets")
    IP=cursor.fetchall()  # generate from object list
    global POWER
    for ip in IP:
        if on_off=='on':
            if int(ip[2])==1:
                urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?on')
            else:
                urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?off')
            POWER=1
        elif on_off=='off':
            try:
                state=urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?state').read()
                if int(state)==1:
                    urllib2.urlopen('http://'+ip[1]+'/cgi-bin/turn.cgi?off')
                cursor.execute("UPDATE sockets SET state = %s WHERE id = %s", (int(state),ip[0],))
                POWER=0
            except:
                print('cannot switch the power off')
    db.commit()
    db.close() 
def turn_on_off(objectIDs, on_off):
    #switch state of socket
    db= MySQLdb.connect(master_IP,master_usr,master_pwd,'FB')
    cursor = db.cursor()
    result=-1
    error=0
    for objectID in objectIDs:
        cursor.execute("SELECT IP FROM sockets WHERE objectID = %s",(objectID,))
        IP=list(sum(cursor.fetchall(), ()))  # generate from object list
        try:
            if on_off=='on':
                urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?on')
                result=1
            elif on_off=='state':
                result=int(urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?state').read())+2
            else:
                urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?off')
                result=1
        except:
            error=1
    db.close() 
    if error==1 and result==1:
        return 0
    else:
        return result         
    
if __name__ == '__main__':
    rospy.init_node('NL_processor', anonymous=True)
    reached=connect_to_master()
    if reached:
        # init music player
        pygame.mixer.init()
        # enable recognition
        enable_rec=rospy.ServiceProxy('enable_recognition', Empty)
        disable_rec=rospy.ServiceProxy('disable_recognition', Empty)
        enable_rec()
        #print(next(gather_information('What is your name').results).text.split('(')[0])
        # listen to speech topic
        rospy.Subscriber("speech", String, process_speech)
        rospy.spin()
