#!/usr/bin/python


import sys
import urllib2
import urllib3
import MySQLdb
import urllib
import platform
import PySide
from threading import Thread
from PySide import QtGui
from layout import *
from change_obj import *
from new_obj import *
from new_soc import *
from change_soc import *
from change_interface import *
from change_room import *
from new_room import *
from new_interface import *
from master_ip import *
from webstream import *
import yaml
import rospkg
import cv2
import time

rospack = rospkg.RosPack()
path=rospack.get_path('budspeech')
__version__ = '1.1.0'

class ControlMainWindow(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        # start in local mode
        self.remote=False
        # try to connect to master
        [self.IP,self.usr,self.pwd,self.authusr,self.authpwd]=self.load_yaml()
        reached=self.connect_to_master()
        if not reached:
            self.ask_for_master()
        else:
            self.init_main()
    def init_main(self):
        self.ui =  Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.label_center.setText('FlatBUDDY - Control V' + __version__)
        # define Button events
        self.ui.but_send_com.clicked.connect(lambda: self.send_speech(self.ui.line_speech.text()))
        self.ui.line_speech.returnPressed.connect(lambda: self.send_speech(self.ui.line_speech.text()))
        self.ui.but_open_stream.clicked.connect(self.show_stream) 
        if not self.remote:
            self.ui.but_new_object.clicked.connect(self.new_object) 
            self.ui.but_new_socket.clicked.connect(self.new_socket) 
            self.ui.but_new_room.clicked.connect(self.new_room) 
            self.ui.but_new_interface.clicked.connect(self.new_inter) 
            # Check if camera defined
            Inter_types=self.get_all_inter_types()
            ipcam_id=[item[0] for item in Inter_types if item[1] == 'IPCAM'][0]
            IPCAMS=[item for item in self.get_all_inter() if item[0] == ipcam_id]
            if IPCAMS:
                self.ui.but_open_stream.setEnabled(True)
        # Menu actions:
        self.ui.actionAbout.setShortcut('Ctrl+A')
        self.ui.actionAbout.setStatusTip('Show About Popup')
        self.ui.actionAbout.triggered.connect(self.show_about)
        self.ui.actionExit.setShortcut('Ctrl+Q')
        self.ui.actionExit.setStatusTip('Exit Program')
        self.ui.actionExit.triggered.connect(self.destroy)
        # don't show the socket buttons in remote mode
        if not self.remote:
            # generate socket buttons, which indicate the state and if reachable  
            self.ui.layout_buttons.addStretch(1) 
            all_socs=self.get_all_sockets()
            self.ui.button_group=QtGui.QButtonGroup(self.ui.layout_buttons)
            buttons=[]
            states=[]
            ids=[]
            i=0

            # display buttons for all sockets:
            for socket in all_socs:
                type=self.get_obj_type(socket[1])
                button = QtGui.QPushButton(str(socket[0])+': '+type)
                state=self.get_state_socket(socket[0])
                if state != -1:
                    if state==0:
                        button.setStyleSheet('QPushButton {background-color: red; color: white;}')
                    else:
                        button.setStyleSheet('QPushButton {background-color: green; color: white;}')
                else:
                    button.setStyleSheet('QPushButton {background-color: grey; color: white;}')
                self.ui.button_group.addButton(button,socket[0])
                buttons.append(button) 
                states.append(state)
                ids.append(socket[0])
                i=i+1
                self.ui.layout_buttons.addWidget(button)
            # set action 
            self.ui.button_group.buttonClicked.connect(self.switch_socket) 
            self.ui.layout_buttons.addStretch(1)  
            # add current objects to listWidget
            # read all
            self.update_obj_items()
            self.ui.list_objects.itemDoubleClicked.connect(self.change_object)
            self.update_soc_items()
            self.ui.list_sockets.itemDoubleClicked.connect(self.change_socket)
            self.update_room_items()
            self.ui.list_rooms.itemDoubleClicked.connect(self.change_room)
            self.update_inter_items()
            self.ui.list_interfaces.itemDoubleClicked.connect(self.change_inter)
            # if remote mode: remove config tab
        else:
            self.ui.but_open_stream.setEnabled(True)
            self.ui.tab_control.removeTab(self.ui.tab_control.indexOf(self.ui.tab_4))
        self.show()  
        
    class Stream_Widget( QtGui.QWidget):

        def __init__(self,IP,pool):
            QtGui.QWidget.__init__(self)
            self.IP=IP
            self.pool=pool
        def open_website(self,url):
            return self.pool.request('GET',url)

        def send_speech(self,speech):
            try:
                fullurl = 'http://'+self.IP+'/cgi-bin/send.py?speech='+str(speech)
                fullurl = urllib.quote(fullurl, safe="%/:=&?~#+!$,;'@()*[]")
                Thread(target=self.open_website, args=[fullurl]).start()
            except: 
                print('Cannot move camera')
            
        def keyPressEvent(self, event):

            key = event.key()
            
            if key == QtCore.Qt.Key_Left:
                self.send_speech('move camera left')
            elif key == QtCore.Qt.Key_Right:
                self.send_speech('move camera right')
            elif key == QtCore.Qt.Key_Down:
                self.send_speech('move camera down')
            elif key == QtCore.Qt.Key_Up:
                self.send_speech('move camera up')
            else:
                QtGui.QWidget.keyPressEvent(self, event)
    
    def stream_image(self):
        while 1:
            h=self.pool.request('GET','http://'+self.IP+'/pics/stream.jpg')
            self.img=h.data
            time.sleep(0.1)
          
    def show_stream(self):
         # init new window:
        uiw=Ui_webstream()
        self.uiw = self.Stream_Widget(self.IP,self.pool)
        uiw.setupUi(self.uiw)
        key=0  
        Thread(target=self.stream_image, args=[]).start()
        self.uiw.show()
        while key!='q':
            try:
                new_image=QtGui.QPixmap()
                if new_image.loadFromData(self.img,'jpg'):
                    image=new_image
                uiw.img_stream.setPixmap(image)  
            except:
                print('couldnt load frame')       
            key=cv2.waitKey(300)
        
    def connect_to_master(self):
        #try to open database connection and look for users
        result=1
        if not self.remote:
            try: 
                db = MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
                cursor = db.cursor()
                cursor.execute("SELECT usr FROM interfaces")
                db.commit();
                db.close();
                result = 1
            except (MySQLdb.OperationalError, MySQLdb.ProgrammingError) as err:
                if err[0]==1045:
                    QtGui.QMessageBox.warning(self, 'Server problem', 'Mysql Username or password invalid!')
                elif err[0]==1049:
                    QtGui.QMessageBox.warning(self, 'Server problem', 'No database "FB" found!')
                elif err[0]==1054 or err[0]==1146:
                    self.init_database()
                    result= 1
                else:
                    QtGui.QMessageBox.warning(self, 'Server problem', 'Server could not be reached!')
                result = 2
        # try to connect and get data from the server
        # try to authenticate
        #self.auth_http(self.authusr,self.authpwd)
        try: 
            #timeout = urllib3.util.timeout.Timeout(connect=0.5, read=0.1)
            self.pool = urllib3.PoolManager(headers=urllib3.util.make_headers(keep_alive=True, accept_encoding=None, user_agent=None, basic_auth=self.authusr+':'+self.authpwd))
            domain='http://'+self.IP+'/index.html'
            self.open_website(domain)
            if result==2:
                result=0
            else:
                result=1
        except:
            QtGui.QMessageBox.warning(self, 'Server', 'Authentification failure!')
            result = 0
        return result
        
    def auth_http(self,user, pwd):
        passman = urllib2.HTTPPasswordMgrWithDefaultRealm()
        # this creates a password manager
        passman.add_password(None, 'http://'+self.IP, user, pwd)
        authhandler = urllib2.HTTPBasicAuthHandler(passman)
        # create the AuthHandler
        opener = urllib2.build_opener(authhandler)
        urllib2.install_opener(opener)
        
    def init_database(self):
    # create all necessary tables
        db = MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute('''CREATE TABLE IF NOT EXISTS objects(id INT PRIMARY KEY AUTO_INCREMENT, type TEXT,room INT, location TEXT, how TEXT)''')
        cursor.execute('''CREATE TABLE IF NOT EXISTS sockets(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, IP TEXT, state INT)''')
        cursor.execute('''CREATE TABLE IF NOT EXISTS mods(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, modification TEXT)''')
        cursor.execute('''CREATE TABLE IF NOT EXISTS rooms(id INT PRIMARY KEY AUTO_INCREMENT, name TEXT)''')
        cursor.execute('''CREATE TABLE IF NOT EXISTS interface_types(id INT PRIMARY KEY, type TEXT)''')
        cursor.execute('''CREATE TABLE IF NOT EXISTS interfaces(id INT PRIMARY KEY AUTO_INCREMENT, IP TEXT, inter_type INT, room INT, usr TEXT, pwd TEXT)''')
        db.commit();
    # insert hardcoded entries
        cursor.execute('''INSERT IGNORE INTO interface_types(id,type) VALUES(%s,%s)''', (1,'MASTER'))
        cursor.execute('''INSERT IGNORE INTO interface_types(id,type) VALUES(%s,%s)''', (2,'IPCAM'))
        cursor.execute('''INSERT IGNORE INTO interfaces(id,inter_type) VALUES(%s,%s)''', (1,1))
        db.commit();
        db.close();
        
    def ask_for_master(self):
        self.hide()
        uim=Ui_master_ip()
        self.uim = QtGui.QWidget()
        uim.setupUi(self.uim)
        uim.line_master_pwd.setEchoMode(QtGui.QLineEdit.Password)
        uim.line_auth_pwd.setEchoMode(QtGui.QLineEdit.Password)
        uim.line_master_pwd.setText(self.pwd)
        uim.line_master_usr.setText(self.usr)
        uim.line_master_IP.setText(self.IP)
        uim.line_auth_pwd.setText(self.authpwd)
        uim.line_auth_usr.setText(self.authusr)
        uim.check_remote.stateChanged.connect(lambda: self.switch_remote([uim.check_remote.isChecked()]))
        # button actions 
        uim.but_quit_master.clicked.connect(self.uim.destroy)
        uim.but_save_master.clicked.connect(lambda: self.save_yaml([uim.line_master_IP.text(),uim.line_auth_usr.text(),uim.line_auth_pwd.text(),uim.line_master_usr.text(),uim.line_master_pwd.text()]))
        self.uim.show()
        
    def switch_remote (self,state):
        if state[0]:
            self.remote=True
        else:
            self.remote=False
        
    def save_yaml(self,data):
        self.IP=data[0]
        self.authusr=data[1]
        self.authpwd=data[2]
        self.usr=data[3]
        self.pwd=data[4]
        write_data={'IP':self.IP,'authusr':self.authusr,'authpwd':self.authpwd,'usr':self.usr,'pwd':self.pwd}
        reached=self.connect_to_master()
        if reached:
            # save to yaml file
            with open(path+'/src/master.yml', 'w') as outfile:
                outfile.write( yaml.dump(write_data, default_flow_style=True) )
            self.uim.close()
            self.init_main()
    
    def load_yaml(self):
            try:
                with open(path+'/src/master.yml', 'r') as readfile:
                    data=yaml.load(readfile)
                    return [data['IP'],data['usr'],data['pwd'],data['authusr'],data['authpwd']]
            except:
                return ['','','','','']
            
    def get_obj_type(self, id):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # get values for item
        try:        
            cursor.execute("SELECT type FROM objects WHERE id = %s", (str(id),))
            type = cursor.fetchall()[0][0]
            db.close()
            return type
        except:
            return 'error'
    
    ## Update/Refresh TAB items
    def update_obj_items(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()  
        self.ui.list_objects.clear()
        all_objects,all_tasks=self.get_all_objects()
        for row in all_objects:
            tasks=''
            cursor.execute("SELECT name FROM rooms WHERE id=%s", (str(row[2]),))
            room_name = cursor.fetchall() 
            for task in all_tasks:
                if task[1]==row[0]:
                    tasks=tasks+task[2]+', '
            QtGui.QListWidgetItem('|'+str(row[0])+'| '+str(row[1])+', '+room_name[0][0]+', '+str(row[3])+', '+str(row[4])+' | tasks: '+tasks[:-2], self.ui.list_objects)
        # close DB
        db.close()

    def update_soc_items(self):
        self.ui.list_sockets.clear()
        all_sockets=self.get_all_sockets()
        for row in all_sockets:
            QtGui.QListWidgetItem('|'+str(row[0])+'| object: '+str(row[1])+', IP: '+str(row[2]), self.ui.list_sockets)
    
    def update_room_items(self):
        self.ui.list_rooms.clear()
        all_rooms=self.get_all_rooms()
        for row in all_rooms:
            QtGui.QListWidgetItem('|'+str(row[0])+'| '+str(row[1]), self.ui.list_rooms)
    
    def update_inter_items(self):
        # get room and interface_types names
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()         
        self.ui.list_interfaces.clear()
        all_inter=self.get_all_inter()
        for row in all_inter:
            cursor.execute("SELECT type FROM interface_types WHERE id=%s", (str(row[2]),))
            inter_type = cursor.fetchall() 
            try:
                cursor.execute("SELECT name FROM rooms WHERE id=%s", (str(row[3]),))
                room_name = cursor.fetchall() [0][0]
            except:
                room_name = ''
            QtGui.QListWidgetItem('|'+str(row[0])+'| type: '+inter_type[0][0]+' | room: '+room_name+' | IP: '+str(row[1]), self.ui.list_interfaces)
        # close DB
        db.close()

    ## GET all items of TABs            
    def get_all_objects(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id, type,room, location, how FROM objects")
        all_objects = cursor.fetchall() 
        cursor.execute("SELECT id, objectID, modification FROM mods")
        all_tasks = cursor.fetchall() 
        # close DB
        db.close()   
        return all_objects,all_tasks
    
    def get_all_sockets(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id, objectID, IP FROM sockets")
        all_sockets = cursor.fetchall() 
        # close DB
        db.close()   
        return all_sockets

    def get_all_rooms(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id, name FROM rooms")
        all_rooms = cursor.fetchall() 
        # close DB
        db.close()   
        return all_rooms

    def get_all_inter(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id, IP, inter_type, room,usr,pwd FROM interfaces")
        all_inter = cursor.fetchall() 
        # close DB
        db.close()   
        return all_inter

    def get_all_inter_types(self):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT id, type FROM interface_types")
        all_inter_types = cursor.fetchall() 
        # close DB
        db.close()   
        return all_inter_types

    # Create new item:
    def new_object(self):
        # init new window:
        uin=Ui_new_object()
        self.uin = QtGui.QWidget()
        uin.setupUi(self.uin)
        # fill combo boxes
        self.fill_combo_inter_room(uin.com_obj_room)
        # button actions
        uin.but_cancel_obj.clicked.connect(self.uin.close)
        uin.but_create_obj.clicked.connect(lambda: self.create_new_object([uin.line_obj_type.text(),self.read_combo_id(uin.com_obj_room),uin.line_obj_loc.text(),uin.line_obj_how.text(),uin.line_obj_tasks.text()]))
        self.uin.show()

    def new_socket(self):
        # init new window:
        uins=Ui_new_socket()
        self.uins = QtGui.QWidget()
        uins.setupUi(self.uins)
        # fill combo box
        self.fill_combo_soc(uins.combo_soc_new)
        # button actions
        uins.but_cancel_soc.clicked.connect(self.uins.close)
        uins.but_create_soc.clicked.connect(lambda: self.create_new_socket([self.read_combo_id(uins.combo_soc_new),uins.line_soc_IP.text()]))
        self.uins.show()
    
    def new_room(self):
        # init new window:
        uinr=Ui_new_room()
        self.uinr = QtGui.QWidget()
        uinr.setupUi(self.uinr)
        # button actions
        uinr.but_cancel_room.clicked.connect(self.uinr.close)
        uinr.but_create_room.clicked.connect(lambda: self.create_new_room([uinr.line_room_name.text()]))
        self.uinr.show()
        
    def new_inter(self):
        # init new window:
        uini=Ui_new_interface()
        self.uini = QtGui.QWidget()
        uini.setupUi(self.uini)
        uini.lin_inter_pwd.setEchoMode(QtGui.QLineEdit.Password)
        # fill combo boxes
        self.fill_combo_inter_room(uini.com_inter_room)
        self.fill_combo_inter_type(uini.com_inter_type)
        # button actions
        uini.but_cancel_inter.clicked.connect(self.uini.close)
        uini.but_create_inter.clicked.connect(lambda: self.create_new_inter([self.read_combo_id(uini.com_inter_type),self.read_combo_id(uini.com_inter_room),uini.line_inter_IP.text(),uini.lin_inter_usr.text(),uini.lin_inter_pwd.text()]))
        self.uini.show()

    def create_new_object(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("INSERT INTO objects(type,room,location,how) VALUES(%s,%s,%s,%s)" ,(data[0],data[1],data[2],data[3]))
        db.commit()
        id=cursor.lastrowid
        for task in data[4].split(', '):
            cursor.execute("INSERT INTO mods(objectID,modification) VALUES(%s,%s)", (id,task))
        db.commit()
        # close DB
        db.close()
        self.update_obj_items()
        self.uin.close()
   
    def create_new_socket(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("INSERT INTO sockets(objectID,IP) VALUES(%s,%s)", (data[0],data[1]))
        db.commit()
        # close DB
        db.close()
        self.update_soc_items()
        self.uins.close()

    def create_new_room(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("INSERT INTO rooms(name) VALUES(%s)", (data[0],))
        db.commit()
        # close DB
        db.close()
        self.update_room_items()
        self.uinr.close()
    
    def create_new_inter(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("INSERT INTO interfaces(inter_type,room,IP,usr,pwd) VALUES(%s,%s,%s,%s,%s)", (data[0],data[1],data[2],data[3],data[4]))
        db.commit()
        # close DB
        db.close()
        self.update_inter_items()
        self.uini.close()

    ## Change TAB items and update result
    def change_object(self,curr):
        # init new window:
        id=int(curr.text().split()[0][1:-1]) # extract id from string
        uic=Ui_change_object()
        self.uicw = QtGui.QWidget()
        uic.setupUi(self.uicw)
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # fill combo box
        self.fill_combo_inter_room(uic.com_obj_room)
        # get values for item
        cursor.execute("SELECT  type, room, location, how FROM objects WHERE id = %s", (str(id),))
        obj_data = cursor.fetchall()[0]
        cursor.execute("SELECT modification FROM mods WHERE objectID=%s", (str(id),))
        all_tasks = cursor.fetchall()
        tasks=''
        for task in all_tasks:
            tasks=tasks+task[0]+', '
        # fill in current data
        uic.label_obj_id.setText(str(id))
        uic.line_obj_type.setText(obj_data[0])
        uic.line_obj_loc.setText(obj_data[2])
        uic.line_obj_how.setText(obj_data[3])
        uic.line_obj_tasks.setText(tasks[:-2])
        # button actions
        uic.but_cancel_obj.clicked.connect(self.uicw.close)
        uic.but_upd_obj.clicked.connect(lambda: self.update_object([id,uic.line_obj_type.text(),self.read_combo_id(uic.com_obj_room),uic.line_obj_loc.text(),uic.line_obj_how.text(),uic.line_obj_tasks.text()]))
        uic.but_del_obj.clicked.connect(lambda: self.delete_object(id))
        # close DB
        db.close()   
        self.uicw.show()

    def change_socket(self,curr):
        # init new window:
        id=int(curr.text().split()[0][1:-1]) # extract id from string
        uis=Ui_change_socket()
        self.uis = QtGui.QWidget()
        uis.setupUi(self.uis)
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # get values for item
        cursor.execute("SELECT  objectID, IP FROM sockets WHERE id = %s", (str(id),))
        soc_data = cursor.fetchall()[0]
        # fill in current data
        uis.label_soc_id.setText(str(id))
        uis.line_soc_IP.setText(soc_data[1])
        # fill combo box
        self.fill_combo_soc(uis.combo_soc)
        # button actions
        uis.but_cancel_soc.clicked.connect(self.uis.close)
        uis.but_update_soc.clicked.connect(lambda: self.update_socket([id,self.read_combo_id(uis.combo_soc),uis.line_soc_IP.text()]))
        uis.but_del_soc.clicked.connect(lambda: self.delete_socket(id))
        # close DB
        db.close()   
        self.uis.show()

    def change_room(self,curr):
        # init new window:
        id=int(curr.text().split()[0][1:-1]) # extract id from string
        uir=Ui_change_room()
        self.uir = QtGui.QWidget()
        uir.setupUi(self.uir)
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # get values for item
        cursor.execute("SELECT name FROM rooms WHERE id = %s", (str(id),))
        room_data = cursor.fetchall()[0]
        # fill in current data
        uir.label_room_id.setText(str(id))
        uir.line_room_name.setText(room_data[0])
        # button actions
        uir.but_cancel_room.clicked.connect(self.uir.close)
        uir.but_upd_room.clicked.connect(lambda: self.update_room([id,uir.line_room_name.text()]))
        uir.but_del_room.clicked.connect(lambda: self.delete_room(id))
        # close DB
        db.close()   
        self.uir.show()
    
    def change_inter(self,curr):
        # init new window:
        id=int(curr.text().split()[0][1:-1]) # extract id from string
        uii=Ui_change_interface()
        self.uii = QtGui.QWidget()
        uii.setupUi(self.uii)
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # get values for item
        cursor.execute("SELECT  inter_type, room, IP,usr,pwd FROM interfaces WHERE id = %s", (str(id),))
        inter_data = cursor.fetchall()[0]
        uii.lin_inter_pwd.setEchoMode(QtGui.QLineEdit.Password)
        # fill in current data
        uii.label_inter_id.setText(str(id))
        uii.line_inter_IP.setText(inter_data[2])
        uii.lin_inter_usr.setText(inter_data[3])
        uii.lin_inter_pwd.setText(inter_data[4])
        # fill combo box
        self.fill_combo_inter_room(uii.com_inter_room)
        if (inter_data[0]==1):
            all_inter_types=self.get_all_inter_types()
            for row in all_inter_types:
                if (str(row[1])=='MASTER'):
                    uii.com_inter_type.addItem('|'+str(row[0])+'| '+str(row[1]))
            uii.but_del_inter.setVisible(False);
        else:
            self.fill_combo_inter_type(uii.com_inter_type)
        # button actions
        uii.but_cancel_inter.clicked.connect(self.uii.close)
        uii.but_upd_inter.clicked.connect(lambda: self.update_inter([id,self.read_combo_id(uii.com_inter_room),self.read_combo_id(uii.com_inter_type),uii.line_inter_IP.text(),uii.lin_inter_usr.text(),uii.lin_inter_pwd.text()]))
        uii.but_del_inter.clicked.connect(lambda: self.delete_inter(id))
        # close DB
        db.close()   
        self.uii.show()

    def update_object(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("UPDATE objects SET type = %s, room = %s, location = %s, how = %s WHERE id = %s", (data[1],data[2],data[3],data[4],data[0]))
        db.commit()
        # drop all tasks 
        cursor.execute("DELETE FROM mods WHERE objectID = %s", (data[0],))
        db.commit()
        for task in data[5].split(', '):
            cursor.execute("INSERT INTO mods(objectID,modification) VALUES(%s,%s)", (data[0],task))
        db.commit()
        # close DB
        db.close()
        self.update_obj_items()
        self.uicw.close()

    def update_socket(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("UPDATE sockets SET objectID = %s, IP = %s WHERE id = %s", (data[1],data[2],data[0]))
        db.commit()
        # close DB
        db.close()
        self.update_soc_items()
        self.uis.close()
        
    def update_room(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("UPDATE rooms SET name = %s WHERE id = %s", (data[1],data[0]))
        db.commit()
        # close DB
        db.close()
        self.update_room_items()
        self.uir.close()
        
    def update_inter(self,data):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("UPDATE interfaces SET room = %s, inter_type = %s, IP = %s, usr = %s, pwd = %s WHERE id = %s", (data[1],data[2],data[3],data[4],data[5],data[0]))
        db.commit()
        # close DB
        db.close()
        self.update_inter_items()
        self.uii.close()
          
    ## delete item:
    def delete_object(self,id):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("DELETE FROM objects WHERE id = %s", (id,))
        cursor.execute("DELETE FROM mods WHERE objectID = %s", (id,))
        db.commit()
        self.update_obj_items()
        db.close()
        self.uicw.close()

    def delete_socket(self,id):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("DELETE FROM sockets WHERE id = %s", (id,))
        db.commit()
        self.update_soc_items()
        db.close()
        self.uis.close()
    
    def delete_room(self,id):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("DELETE FROM rooms WHERE id = %s", (id,))
        db.commit()
        self.update_room_items()
        db.close()
        self.uir.close()
    
    def delete_inter(self,id):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("DELETE FROM interfaces WHERE id = %s", (id,))
        db.commit()
        self.update_inter_items()
        db.close()
        self.uii.close()
    
    ## Combo boxes:
    def read_combo_id(self,combo_soc):
        try:
            text=combo_soc.currentText()
            return int(text.split()[0][1:-1])
        except:
            return None

    def fill_combo_soc(self,combo_soc):
        all_objects,all_tasks=self.get_all_objects()
        for row in all_objects:
           combo_soc.addItem('|'+str(row[0])+'| '+str(row[1])+', '+str(row[2])+', '+str(row[3]))

    def fill_combo_inter_room(self,combo_inter_room):
        all_rooms=self.get_all_rooms()
        for row in all_rooms:
           combo_inter_room.addItem('|'+str(row[0])+'| '+str(row[1]))

    def fill_combo_inter_type(self,combo_inter_type):
        all_inter_types=self.get_all_inter_types()
        for row in all_inter_types:
            if (str(row[1])!='MASTER'):
                combo_inter_type.addItem('|'+str(row[0])+'| '+str(row[1]))

    def open_website(self,url):
        h=self.pool.request('GET', url)
        message=h.data
        html_id=message.find('</html>')   
        message=message[html_id+9:].replace('\n', ' ').replace('\r', '') 
        #print(message)   
        self.statusBar().showMessage(message)

    def send_speech(self,speech):
        try:
            fullurl = 'http://'+self.IP+'/cgi-bin/send.py?speech='+str(speech)
            fullurl = urllib.quote(fullurl, safe="%/:=&?~#+!$,;'@()*[]")
            self.open_website(fullurl)
            #self.statusBar().showMessage('Message sent')
            self.ui.line_speech.setText('') 
        except:
            self.statusBar().showMessage('cannot reach server')
        
    def get_state_socket(self,id):
        #switch state of socket
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("SELECT IP FROM sockets WHERE id = %s",(str(id),))
        IP=list(sum(cursor.fetchall(), ()))  # generate from object list
        db.close()    
        try:
            state=urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?state',timeout=0.8)
            return int(state.read())
        except:
            return -1

    def switch_socket(self,button):
        #switch state of socket
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        id=self.ui.button_group.id(button)
        on_off=self.get_state_socket(id)
        cursor.execute("SELECT IP FROM sockets WHERE id = %s",(str(id),))
        IP=list(sum(cursor.fetchall(), ()))  # generate from object list
        if on_off==1:
            urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?off')
            self.statusBar().showMessage('Socket'+str(id)+' turned off')
            button.setStyleSheet('QPushButton {background-color: red; color: white;}')
        elif on_off==0:
            urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?on')
            self.statusBar().showMessage('Socket'+str(id)+' turned on')
            button.setStyleSheet('QPushButton {background-color: green; color: white;}')
        else:
            self.statusBar().showMessage('Socket cannot be reached!')
        db.close() 
    def show_about(self):
        QtGui.QMessageBox.about(self, "About",
        """<b>FlatBUDDY - Control</b> v %s
        <p>Copyright 2015 Simon Bilgeri.
        All rights reserved in accordance with
        GPL v2 or later - NO WARRANTIES!
        <p>This application can be used to monitor and change the status of the FlatBUDDY system 
        aswell as interact with it.
        <p>Python %s -  PySide version %s - Qt version %s on %s""" % (__version__,
        platform.python_version(), PySide.__version__,  PySide.QtCore.__version__,
        platform.system()))

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    mySW = ControlMainWindow()
    sys.exit(app.exec_())