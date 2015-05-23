# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Master_IP.ui'
#
# Created: Fri May 22 17:10:39 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_master_ip(object):
    def setupUi(self, master_ip):
        master_ip.setObjectName("master_ip")
        master_ip.resize(402, 171)
        self.verticalLayoutWidget = QtGui.QWidget(master_ip)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 161))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtGui.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName("formLayout")
        self.label2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label2.setObjectName("label2")
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label2)
        self.line_master_IP = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_master_IP.setObjectName("line_master_IP")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.line_master_IP)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_3)
        self.line_master_usr = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_master_usr.setObjectName("line_master_usr")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_master_usr)
        self.label_4 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_4)
        self.line_master_pwd = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_master_pwd.setObjectName("line_master_pwd")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.line_master_pwd)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_quit_master = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_quit_master.setObjectName("but_quit_master")
        self.horizontalLayout.addWidget(self.but_quit_master)
        self.but_save_master = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_save_master.setObjectName("but_save_master")
        self.horizontalLayout.addWidget(self.but_save_master)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(master_ip)
        QtCore.QMetaObject.connectSlotsByName(master_ip)

    def retranslateUi(self, master_ip):
        master_ip.setWindowTitle(QtGui.QApplication.translate("master_ip", "Master IP", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("master_ip", "Master could not be reached!", None, QtGui.QApplication.UnicodeUTF8))
        self.label2.setText(QtGui.QApplication.translate("master_ip", "IP:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("master_ip", "username:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("master_ip", "password:", None, QtGui.QApplication.UnicodeUTF8))
        self.but_quit_master.setText(QtGui.QApplication.translate("master_ip", "Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.but_save_master.setText(QtGui.QApplication.translate("master_ip", "Ok", None, QtGui.QApplication.UnicodeUTF8))

