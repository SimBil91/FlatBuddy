# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'FlatBUDDY.ui'
#
# Created: Wed Sep 23 00:36:55 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(626, 430)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtGui.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(9, 10, 611, 371))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.tab_control = QtGui.QTabWidget(self.verticalLayoutWidget)
        self.tab_control.setObjectName("tab_control")
        self.tab_3 = QtGui.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.verticalLayoutWidget_2 = QtGui.QWidget(self.tab_3)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 10, 591, 321))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.layout_buttons = QtGui.QHBoxLayout()
        self.layout_buttons.setObjectName("layout_buttons")
        self.verticalLayout_3.addLayout(self.layout_buttons)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalLayout_4.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.horizontalLayout_4.addLayout(self.horizontalLayout_7)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.label_center = QtGui.QLabel(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setWeight(75)
        font.setItalic(True)
        font.setBold(True)
        self.label_center.setFont(font)
        self.label_center.setAlignment(QtCore.Qt.AlignCenter)
        self.label_center.setObjectName("label_center")
        self.verticalLayout_3.addWidget(self.label_center)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem1)
        self.but_open_stream = QtGui.QPushButton(self.verticalLayoutWidget_2)
        self.but_open_stream.setEnabled(False)
        self.but_open_stream.setObjectName("but_open_stream")
        self.horizontalLayout_8.addWidget(self.but_open_stream)
        spacerItem2 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem2)
        self.verticalLayout_3.addLayout(self.horizontalLayout_8)
        spacerItem3 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtGui.QLabel(self.verticalLayoutWidget_2)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.line_speech = QtGui.QLineEdit(self.verticalLayoutWidget_2)
        self.line_speech.setText("")
        self.line_speech.setObjectName("line_speech")
        self.horizontalLayout.addWidget(self.line_speech)
        self.but_send_com = QtGui.QPushButton(self.verticalLayoutWidget_2)
        self.but_send_com.setObjectName("but_send_com")
        self.horizontalLayout.addWidget(self.but_send_com)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.tab_control.addTab(self.tab_3, "")
        self.tab_4 = QtGui.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.tabWidget = QtGui.QTabWidget(self.tab_4)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 611, 341))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtGui.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayoutWidget_6 = QtGui.QWidget(self.tab)
        self.verticalLayoutWidget_6.setGeometry(QtCore.QRect(0, 10, 601, 291))
        self.verticalLayoutWidget_6.setObjectName("verticalLayoutWidget_6")
        self.verticalLayout_7 = QtGui.QVBoxLayout(self.verticalLayoutWidget_6)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.list_rooms = QtGui.QListWidget(self.verticalLayoutWidget_6)
        self.list_rooms.setObjectName("list_rooms")
        self.verticalLayout_7.addWidget(self.list_rooms)
        self.horizontalLayout_13 = QtGui.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_13.addItem(spacerItem4)
        self.but_new_room = QtGui.QPushButton(self.verticalLayoutWidget_6)
        self.but_new_room.setObjectName("but_new_room")
        self.horizontalLayout_13.addWidget(self.but_new_room)
        self.verticalLayout_7.addLayout(self.horizontalLayout_13)
        self.tabWidget.addTab(self.tab, "")
        self.tab_5 = QtGui.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.verticalLayoutWidget_3 = QtGui.QWidget(self.tab_5)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(0, 10, 601, 291))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.list_objects = QtGui.QListWidget(self.verticalLayoutWidget_3)
        self.list_objects.setObjectName("list_objects")
        self.verticalLayout_4.addWidget(self.list_objects)
        self.horizontalLayout_10 = QtGui.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_10.addItem(spacerItem5)
        self.but_new_object = QtGui.QPushButton(self.verticalLayoutWidget_3)
        self.but_new_object.setObjectName("but_new_object")
        self.horizontalLayout_10.addWidget(self.but_new_object)
        self.verticalLayout_4.addLayout(self.horizontalLayout_10)
        self.tabWidget.addTab(self.tab_5, "")
        self.tab_6 = QtGui.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.verticalLayoutWidget_4 = QtGui.QWidget(self.tab_6)
        self.verticalLayoutWidget_4.setGeometry(QtCore.QRect(0, 10, 601, 291))
        self.verticalLayoutWidget_4.setObjectName("verticalLayoutWidget_4")
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.verticalLayoutWidget_4)
        self.verticalLayout_5.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.list_sockets = QtGui.QListWidget(self.verticalLayoutWidget_4)
        self.list_sockets.setObjectName("list_sockets")
        self.verticalLayout_5.addWidget(self.list_sockets)
        self.horizontalLayout_11 = QtGui.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        spacerItem6 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_11.addItem(spacerItem6)
        self.but_new_socket = QtGui.QPushButton(self.verticalLayoutWidget_4)
        self.but_new_socket.setObjectName("but_new_socket")
        self.horizontalLayout_11.addWidget(self.but_new_socket)
        self.verticalLayout_5.addLayout(self.horizontalLayout_11)
        self.tabWidget.addTab(self.tab_6, "")
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayoutWidget_5 = QtGui.QWidget(self.tab_2)
        self.verticalLayoutWidget_5.setGeometry(QtCore.QRect(0, 10, 601, 291))
        self.verticalLayoutWidget_5.setObjectName("verticalLayoutWidget_5")
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.verticalLayoutWidget_5)
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.list_interfaces = QtGui.QListWidget(self.verticalLayoutWidget_5)
        self.list_interfaces.setObjectName("list_interfaces")
        self.verticalLayout_6.addWidget(self.list_interfaces)
        self.horizontalLayout_12 = QtGui.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        spacerItem7 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_12.addItem(spacerItem7)
        self.but_new_interface = QtGui.QPushButton(self.verticalLayoutWidget_5)
        self.but_new_interface.setObjectName("but_new_interface")
        self.horizontalLayout_12.addWidget(self.but_new_interface)
        self.verticalLayout_6.addLayout(self.horizontalLayout_12)
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_control.addTab(self.tab_4, "")
        self.verticalLayout.addWidget(self.tab_control)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 626, 25))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuHelp = QtGui.QMenu(self.menubar)
        self.menuHelp.setObjectName("menuHelp")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionExit = QtGui.QAction(MainWindow)
        self.actionExit.setObjectName("actionExit")
        self.actionAbout = QtGui.QAction(MainWindow)
        self.actionAbout.setObjectName("actionAbout")
        self.menuFile.addAction(self.actionExit)
        self.menuHelp.addAction(self.actionAbout)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuHelp.menuAction())

        self.retranslateUi(MainWindow)
        self.tab_control.setCurrentIndex(0)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "FlatBUDDY - CONTROL V1.0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_center.setText(QtGui.QApplication.translate("MainWindow", "FlatBUDDY CONTROL V1.0", None, QtGui.QApplication.UnicodeUTF8))
        self.but_open_stream.setToolTip(QtGui.QApplication.translate("MainWindow", "<html><head/><body><p>send message to server</p><p><br/></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.but_open_stream.setText(QtGui.QApplication.translate("MainWindow", "Open Stream", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Command:", None, QtGui.QApplication.UnicodeUTF8))
        self.line_speech.setToolTip(QtGui.QApplication.translate("MainWindow", "<html><head/><body><p>Send message to FlatBUDDY</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.but_send_com.setToolTip(QtGui.QApplication.translate("MainWindow", "<html><head/><body><p>send message to server</p><p><br/></p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.but_send_com.setText(QtGui.QApplication.translate("MainWindow", "send", None, QtGui.QApplication.UnicodeUTF8))
        self.tab_control.setTabText(self.tab_control.indexOf(self.tab_3), QtGui.QApplication.translate("MainWindow", "Control", None, QtGui.QApplication.UnicodeUTF8))
        self.but_new_room.setText(QtGui.QApplication.translate("MainWindow", "New Room", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QtGui.QApplication.translate("MainWindow", "Rooms", None, QtGui.QApplication.UnicodeUTF8))
        self.but_new_object.setText(QtGui.QApplication.translate("MainWindow", "New Object", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), QtGui.QApplication.translate("MainWindow", "Objects", None, QtGui.QApplication.UnicodeUTF8))
        self.but_new_socket.setText(QtGui.QApplication.translate("MainWindow", "New Socket", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), QtGui.QApplication.translate("MainWindow", "Sockets", None, QtGui.QApplication.UnicodeUTF8))
        self.but_new_interface.setText(QtGui.QApplication.translate("MainWindow", "New Interface", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QtGui.QApplication.translate("MainWindow", "Interfaces", None, QtGui.QApplication.UnicodeUTF8))
        self.tab_control.setTabText(self.tab_control.indexOf(self.tab_4), QtGui.QApplication.translate("MainWindow", "Config", None, QtGui.QApplication.UnicodeUTF8))
        self.menuFile.setTitle(QtGui.QApplication.translate("MainWindow", "File", None, QtGui.QApplication.UnicodeUTF8))
        self.menuHelp.setTitle(QtGui.QApplication.translate("MainWindow", "Help", None, QtGui.QApplication.UnicodeUTF8))
        self.actionExit.setText(QtGui.QApplication.translate("MainWindow", "Exit", None, QtGui.QApplication.UnicodeUTF8))
        self.actionAbout.setText(QtGui.QApplication.translate("MainWindow", "About", None, QtGui.QApplication.UnicodeUTF8))

