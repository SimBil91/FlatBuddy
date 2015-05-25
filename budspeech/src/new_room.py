# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'new_room.ui'
#
# Created: Mon May 25 02:04:07 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_new_object(object):
    def setupUi(self, new_object):
        new_object.setObjectName("new_object")
        new_object.resize(397, 104)
        self.verticalLayoutWidget = QtGui.QWidget(new_object)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 81))
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
        self.line_room_name = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_room_name.setObjectName("line_room_name")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.line_room_name)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_cancel_room = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_room.setObjectName("but_cancel_room")
        self.horizontalLayout.addWidget(self.but_cancel_room)
        self.but_create_room = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_create_room.setObjectName("but_create_room")
        self.horizontalLayout.addWidget(self.but_create_room)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(new_object)
        QtCore.QMetaObject.connectSlotsByName(new_object)

    def retranslateUi(self, new_object):
        new_object.setWindowTitle(QtGui.QApplication.translate("new_object", "Create new room", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("new_object", "name:", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_room.setText(QtGui.QApplication.translate("new_object", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_create_room.setText(QtGui.QApplication.translate("new_object", "Create", None, QtGui.QApplication.UnicodeUTF8))

