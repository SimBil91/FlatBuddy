# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'change_interface.ui'
#
# Created: Mon May 25 02:03:39 2015
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_change_interface(object):
    def setupUi(self, change_interface):
        change_interface.setObjectName("change_interface")
        change_interface.resize(400, 178)
        self.verticalLayoutWidget = QtGui.QWidget(change_interface)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 161))
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
        self.label_obj_id = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_obj_id.setObjectName("label_obj_id")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.label_obj_id)
        self.label_2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_2)
        self.line_inter_IP = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_inter_IP.setObjectName("line_inter_IP")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_inter_IP)
        self.label_6 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_6)
        self.com_inter_type = QtGui.QComboBox(self.verticalLayoutWidget)
        self.com_inter_type.setObjectName("com_inter_type")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.com_inter_type)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_3)
        self.com_inter_room = QtGui.QComboBox(self.verticalLayoutWidget)
        self.com_inter_room.setObjectName("com_inter_room")
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.com_inter_room)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_del_obj = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_del_obj.setObjectName("but_del_obj")
        self.horizontalLayout.addWidget(self.but_del_obj)
        self.but_cancel_obj = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_obj.setObjectName("but_cancel_obj")
        self.horizontalLayout.addWidget(self.but_cancel_obj)
        self.but_upd_obj = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_upd_obj.setObjectName("but_upd_obj")
        self.horizontalLayout.addWidget(self.but_upd_obj)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(change_interface)
        QtCore.QMetaObject.connectSlotsByName(change_interface)

    def retranslateUi(self, change_interface):
        change_interface.setWindowTitle(QtGui.QApplication.translate("change_interface", "Change Interface", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("change_interface", "id:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_obj_id.setText(QtGui.QApplication.translate("change_interface", "id_label", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("change_interface", "IP", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("change_interface", "type:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("change_interface", "room:", None, QtGui.QApplication.UnicodeUTF8))
        self.but_del_obj.setText(QtGui.QApplication.translate("change_interface", "Delete", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_obj.setText(QtGui.QApplication.translate("change_interface", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_upd_obj.setText(QtGui.QApplication.translate("change_interface", "Update", None, QtGui.QApplication.UnicodeUTF8))

