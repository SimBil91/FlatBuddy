# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'new_interface.ui'
#
# Created: Mon May 25 02:03:53 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_new_interface(object):
    def setupUi(self, new_interface):
        new_interface.setObjectName("new_interface")
        new_interface.resize(395, 159)
        self.verticalLayoutWidget = QtGui.QWidget(new_interface)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 141))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName("formLayout")
        self.label_2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label_2)
        self.line_inter_IP = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_inter_IP.setObjectName("line_inter_IP")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.line_inter_IP)
        self.label2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label2.setObjectName("label2")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label2)
        self.com_inter_type = QtGui.QComboBox(self.verticalLayoutWidget)
        self.com_inter_type.setObjectName("com_inter_type")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.com_inter_type)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_3)
        self.con_inter_room = QtGui.QComboBox(self.verticalLayoutWidget)
        self.con_inter_room.setObjectName("con_inter_room")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.con_inter_room)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_cancel_inter = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_inter.setObjectName("but_cancel_inter")
        self.horizontalLayout.addWidget(self.but_cancel_inter)
        self.but_create_inter = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_create_inter.setObjectName("but_create_inter")
        self.horizontalLayout.addWidget(self.but_create_inter)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(new_interface)
        QtCore.QMetaObject.connectSlotsByName(new_interface)

    def retranslateUi(self, new_interface):
        new_interface.setWindowTitle(QtGui.QApplication.translate("new_interface", "Create new interface", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("new_interface", "IP:", None, QtGui.QApplication.UnicodeUTF8))
        self.label2.setText(QtGui.QApplication.translate("new_interface", "type:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("new_interface", "room:", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_inter.setText(QtGui.QApplication.translate("new_interface", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_create_inter.setText(QtGui.QApplication.translate("new_interface", "Create", None, QtGui.QApplication.UnicodeUTF8))

