# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'new_socket.ui'
#
# Created: Tue Mar 31 17:23:54 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_new_socket(object):
    def setupUi(self, new_socket):
        new_socket.setObjectName("new_socket")
        new_socket.resize(400, 134)
        self.verticalLayoutWidget = QtGui.QWidget(new_socket)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 111))
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
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label_3)
        self.line_soc_IP = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_soc_IP.setObjectName("line_soc_IP")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_soc_IP)
        self.combo_soc_new = QtGui.QComboBox(self.verticalLayoutWidget)
        self.combo_soc_new.setObjectName("combo_soc_new")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.combo_soc_new)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_cancel_soc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_soc.setObjectName("but_cancel_soc")
        self.horizontalLayout.addWidget(self.but_cancel_soc)
        self.but_create_soc = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_create_soc.setObjectName("but_create_soc")
        self.horizontalLayout.addWidget(self.but_create_soc)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(new_socket)
        QtCore.QMetaObject.connectSlotsByName(new_socket)

    def retranslateUi(self, new_socket):
        new_socket.setWindowTitle(QtGui.QApplication.translate("new_socket", "Create new socket", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("new_socket", "object:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("new_socket", "IP:", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_soc.setText(QtGui.QApplication.translate("new_socket", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_create_soc.setText(QtGui.QApplication.translate("new_socket", "Create", None, QtGui.QApplication.UnicodeUTF8))

