#!/usr/bin/python


import sys
import urllib2
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
from master_ip import *
import yaml

path=''
__version__ = '1.0.2'

class ControlMainWindow(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        # try to connect to master
        [self.IP,self.usr,self.pwd]=self.load_yaml()
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
            
        self.ui.but_new_object.clicked.connect(self.new_object) 
        self.ui.but_new_socket.clicked.connect(self.new_socket) 
        
        # Menu actions:
        self.ui.actionAbout.setShortcut('Ctrl+A')
        self.ui.actionAbout.setStatusTip('Show About Popup')
        self.ui.actionAbout.triggered.connect(self.show_about)
        self.ui.actionExit.setShortcut('Ctrl+Q')
        self.ui.actionExit.setStatusTip('Exit Program')
        self.ui.actionExit.triggered.connect(self.destroy)
        # generate socket buttons, which indicate the state and if reachable  
        self.ui.layout_buttons.addStretch(1) 
        all_socs=self.get_all_sockets()
        self.ui.button_group=QtGui.QButtonGroup(self.ui.layout_buttons)
        buttons=[]
        states=[]
        ids=[]
        i=0
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
        self.show()        
    def connect_to_master(self):
        #try to open database connection and look for users
        try: 
            db = MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
            cursor = db.cursor()
            cursor.execute("SELECT type FROM objects")
            db.commit();
            db.close();
            return 1
        except MySQLdb.OperationalError and MySQLdb.ProgrammingError as err:
            if err[0]==1045:
                QtGui.QMessageBox.warning(self, 'Server problem', 'Username or password invalid!')
            elif err[0]==1049:
                QtGui.QMessageBox.warning(self, 'Server problem', 'No database "FB" found!')
            elif err[0]==1054 or err[0]==1146:
                # create all necessary tables
                db = MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
                cursor = db.cursor()
                cursor.execute('''CREATE TABLE IF NOT EXISTS objects(id INT PRIMARY KEY AUTO_INCREMENT, type TEXT,room TEXT, location TEXT, how TEXT)''')
                cursor.execute('''CREATE TABLE IF NOT EXISTS sockets(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, IP TEXT, state INT)''')
                cursor.execute('''CREATE TABLE IF NOT EXISTS mods(id INT PRIMARY KEY AUTO_INCREMENT, objectID INT, modification TEXT)''')
                db.commit();
                db.close();
                return 1
            else:
                QtGui.QMessageBox.warning(self, 'Server problem', 'Server could not be reached!')
            return 0
    def ask_for_master(self):
        self.hide()
        uim=Ui_master_ip()
        self.uim = QtGui.QWidget()
        uim.setupUi(self.uim)
        uim.line_master_pwd.setEchoMode(QtGui.QLineEdit.Password)
        uim.line_master_pwd.setText(self.pwd)
        uim.line_master_usr.setText(self.usr)
        uim.line_master_IP.setText(self.IP)
        # button actions
        uim.but_quit_master.clicked.connect(self.uim.destroy)
        uim.but_save_master.clicked.connect(lambda: self.save_yaml([uim.line_master_IP.text(),uim.line_master_usr.text(),uim.line_master_pwd.text()]))
        self.uim.show()
    def save_yaml(self,data):
        self.IP=data[0]
        self.usr=data[1]
        self.pwd=data[2]
        write_data={'IP':self.IP,'usr':self.usr,'pwd':self.pwd}
        reached=self.connect_to_master()
        if reached:
            # save to yaml file
            with open('master.yml', 'w') as outfile:
                outfile.write( yaml.dump(write_data, default_flow_style=True) )
            self.uim.close()
            self.init_main()
    
    def load_yaml(self):
        with open('master.yml', 'r') as readfile:
            data=yaml.load(readfile)
            try:
                return [data['IP'],data['usr'],data['pwd']]
            except:
                return ['','','']
            
    def get_obj_type(self, id):
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        # get values for item
        cursor.execute("SELECT type FROM objects WHERE id = %s", (str(id),))
        type = cursor.fetchall()[0][0]
        db.close()
        return type
    def update_obj_items(self):
        self.ui.list_objects.clear()
        all_objects,all_tasks=self.get_all_objects()
        for row in all_objects:
            tasks=''
            for task in all_tasks:
                if task[1]==row[0]:
                    tasks=tasks+task[2]+', '
            QtGui.QListWidgetItem('|'+str(row[0])+'| '+str(row[1])+', '+str(row[2])+', '+str(row[3])+', '+str(row[4])+' | tasks: '+tasks[:-2], self.ui.list_objects)
   
    def update_soc_items(self):
        self.ui.list_sockets.clear()
        all_sockets=self.get_all_sockets()
        for row in all_sockets:
            QtGui.QListWidgetItem('|'+str(row[0])+'| object: '+str(row[1])+', IP: '+str(row[2]), self.ui.list_sockets)
          
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
    
    def new_object(self):
        # init new window:
        uin=Ui_new_object()
        self.uin = QtGui.QWidget()
        uin.setupUi(self.uin)
        
        # button actions
        uin.but_cancel_obj.clicked.connect(self.uin.close)
        uin.but_create_obj.clicked.connect(lambda: self.create_new_object([uin.line_obj_type.text(),uin.line_obj_room.text(),uin.line_obj_loc.text(),uin.line_obj_how.text(),uin.line_obj_tasks.text()]))
        self.uin.show()
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
    def change_object(self,curr):
        # init new window:
        id=int(curr.text().split()[0][1:-1]) # extract id from string
        uic=Ui_change_object()
        self.uicw = QtGui.QWidget()
        uic.setupUi(self.uicw)
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
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
        uic.line_obj_room.setText(obj_data[1])
        uic.line_obj_loc.setText(obj_data[2])
        uic.line_obj_how.setText(obj_data[3])
        uic.line_obj_tasks.setText(tasks[:-2])
        # button actions
        uic.but_cancel_obj.clicked.connect(self.uicw.close)
        uic.but_upd_obj.clicked.connect(lambda: self.update_object([id,uic.line_obj_type.text(),uic.line_obj_room.text(),uic.line_obj_loc.text(),uic.line_obj_how.text(),uic.line_obj_tasks.text()]))
        uic.but_del_obj.clicked.connect(lambda: self.delete_object(id))
        # close DB
        db.close()   
        self.uicw.show()
        
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
    def read_combo_id(self,combo_soc):
        text=combo_soc.currentText()
        return int(text.split()[0][1:-1])
    def fill_combo_soc(self,combo_soc):
        all_objects,all_tasks=self.get_all_objects()
        for row in all_objects:
           combo_soc.addItem('|'+str(row[0])+'| '+str(row[1])+', '+str(row[2])+', '+str(row[3]))
     
    def update_socket(self,data):
        # connect db
        print(data)
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("UPDATE sockets SET objectID = %s, IP = %s WHERE id = %s", (data[1],data[2],data[0]))
        db.commit()
        # close DB
        db.close()
        self.update_soc_items()
        self.uis.close()      
        
    def delete_socket(self,id):
        # connect db
        db= MySQLdb.connect(self.IP,self.usr,self.pwd,'FB')
        cursor = db.cursor()
        cursor.execute("DELETE FROM sockets WHERE id = %s", (id,))
        db.commit()
        self.update_soc_items()
        db.close()
        self.uis.close()
        
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
    def open_website(self,url):
        return urllib2.urlopen(url)

    def send_speech(self,speech):
        try:
            fullurl = 'http://'+self.IP+'/cgi-bin/send.py?speech='+str(speech)
            fullurl = urllib.quote(fullurl, safe="%/:=&?~#+!$,;'@()*[]")
            Thread(target=self.open_website, args=(fullurl,)).start()
            self.statusBar().showMessage('Message sent')
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
            state=urllib2.urlopen('http://'+IP[0]+'/cgi-bin/turn.cgi?state')
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
        <p>This application can be used to monitor and change the status of the FlatBUDDY syste 
        aswell as interact with it.
        <p>Python %s -  PySide version %s - Qt version %s on %s""" % (__version__,
        platform.python_version(), PySide.__version__,  PySide.QtCore.__version__,
        platform.system()))

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    mySW = ControlMainWindow()
    sys.exit(app.exec_())