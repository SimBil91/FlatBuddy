# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'change_room.ui'
#
# Created: Mon May 25 11:24:38 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_change_room(object):
    def setupUi(self, change_room):
        change_room.setObjectName("change_room")
        change_room.resize(399, 120)
        self.verticalLayoutWidget = QtGui.QWidget(change_room)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 101))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName("formLayout")
        self.label = QtGui.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label)
        self.label_room_id = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_room_id.setObjectName("label_room_id")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.label_room_id)
        self.label_2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_2)
        self.line_room_name = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_room_name.setObjectName("line_room_name")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_room_name)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_del_room = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_del_room.setObjectName("but_del_room")
        self.horizontalLayout.addWidget(self.but_del_room)
        self.but_cancel_room = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_room.setObjectName("but_cancel_room")
        self.horizontalLayout.addWidget(self.but_cancel_room)
        self.but_upd_room = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_upd_room.setObjectName("but_upd_room")
        self.horizontalLayout.addWidget(self.but_upd_room)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(change_room)
        QtCore.QMetaObject.connectSlotsByName(change_room)

    def retranslateUi(self, change_room):
        change_room.setWindowTitle(QtGui.QApplication.translate("change_room", "Change room", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("change_room", "id:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_room_id.setText(QtGui.QApplication.translate("change_room", "id_label", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("change_room", "name", None, QtGui.QApplication.UnicodeUTF8))
        self.but_del_room.setText(QtGui.QApplication.translate("change_room", "Delete", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_room.setText(QtGui.QApplication.translate("change_room", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_upd_room.setText(QtGui.QApplication.translate("change_room", "Update", None, QtGui.QApplication.UnicodeUTF8))

