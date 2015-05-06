# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'change_socket.ui'
#
# Created: Tue Mar 31 16:52:49 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_change_socket(object):
    def setupUi(self, change_socket):
        change_socket.setObjectName("change_socket")
        change_socket.resize(405, 144)
        self.verticalLayoutWidget = QtGui.QWidget(change_socket)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 128))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.formLayout = QtGui.QFormLayout()
        self.formLayout.setFieldGrowthPolicy(QtGui.QFormLayout.AllNonFixedFieldsGrow)
        self.formLayout.setObjectName("formLayout")
        self.label_2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_2)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_3)
        self.line_soc_IP = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_soc_IP.setObjectName("line_soc_IP")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.line_soc_IP)
        self.combo_soc = QtGui.QComboBox(self.verticalLayoutWidget)
        self.combo_soc.setObjectName("combo_soc")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.combo_soc)
        self.label = QtGui.QLabel(self.verticalLayoutWidget)
        self.label.setObjectName("label")
        self.formLayout.setWidget(0, QtGui.QFormLayout.LabelRole, self.label)
        self.label_soc_id = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_soc_id.setObjectName("label_soc_id")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.label_soc_id)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_del_soc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_del_soc.setObjectName("but_del_soc")
        self.horizontalLayout.addWidget(self.but_del_soc)
        self.but_cancel_soc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_soc.setObjectName("but_cancel_soc")
        self.horizontalLayout.addWidget(self.but_cancel_soc)
        self.but_update_soc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_update_soc.setObjectName("but_update_soc")
        self.horizontalLayout.addWidget(self.but_update_soc)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(change_socket)
        QtCore.QMetaObject.connectSlotsByName(change_socket)

    def retranslateUi(self, change_socket):
        change_socket.setWindowTitle(QtGui.QApplication.translate("change_socket", "Change socket", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("change_socket", "object:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("change_socket", "IP:", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("change_socket", "id:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_soc_id.setText(QtGui.QApplication.translate("change_socket", "id_socket", None, QtGui.QApplication.UnicodeUTF8))
        self.but_del_soc.setText(QtGui.QApplication.translate("change_socket", "Delete", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_soc.setText(QtGui.QApplication.translate("change_socket", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_update_soc.setText(QtGui.QApplication.translate("change_socket", "Update", None, QtGui.QApplication.UnicodeUTF8))

