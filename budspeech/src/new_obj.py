# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'new_object.ui'
#
# Created: Fri Apr 10 17:00:34 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_new_object(object):
    def setupUi(self, new_object):
        new_object.setObjectName("new_object")
        new_object.resize(400, 232)
        self.verticalLayoutWidget = QtGui.QWidget(new_object)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 211))
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
        self.line_obj_type = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_type.setObjectName("line_obj_type")
        self.formLayout.setWidget(0, QtGui.QFormLayout.FieldRole, self.line_obj_type)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_3)
        self.line_obj_loc = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_loc.setObjectName("line_obj_loc")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.line_obj_loc)
        self.label_4 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_4)
        self.line_obj_how = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_how.setObjectName("line_obj_how")
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.line_obj_how)
        self.label_5 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.formLayout.setWidget(4, QtGui.QFormLayout.LabelRole, self.label_5)
        self.line_obj_tasks = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_tasks.setObjectName("line_obj_tasks")
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.line_obj_tasks)
        self.label2 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label2.setObjectName("label2")
        self.formLayout.setWidget(1, QtGui.QFormLayout.LabelRole, self.label2)
        self.line_obj_room = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_room.setObjectName("line_obj_room")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_obj_room)
        self.verticalLayout.addLayout(self.formLayout)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.but_cancel_obj = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_cancel_obj.setObjectName("but_cancel_obj")
        self.horizontalLayout.addWidget(self.but_cancel_obj)
        self.but_create_obj = QtGui.QPushButton(self.verticalLayoutWidget)
        self.but_create_obj.setObjectName("but_create_obj")
        self.horizontalLayout.addWidget(self.but_create_obj)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(new_object)
        QtCore.QMetaObject.connectSlotsByName(new_object)

    def retranslateUi(self, new_object):
        new_object.setWindowTitle(QtGui.QApplication.translate("new_object", "Create new object", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("new_object", "type:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("new_object", "location:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("new_object", "feature:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("new_object", "tasks:", None, QtGui.QApplication.UnicodeUTF8))
        self.label2.setText(QtGui.QApplication.translate("new_object", "room", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_obj.setText(QtGui.QApplication.translate("new_object", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_create_obj.setText(QtGui.QApplication.translate("new_object", "Create", None, QtGui.QApplication.UnicodeUTF8))

