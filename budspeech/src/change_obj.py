# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'change_object.ui'
#
# Created: Fri Apr 10 16:44:12 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_change_object(object):
    def setupUi(self, change_object):
        change_object.setObjectName("change_object")
        change_object.resize(400, 246)
        self.verticalLayoutWidget = QtGui.QWidget(change_object)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 10, 381, 227))
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
        self.line_obj_type = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_type.setObjectName("line_obj_type")
        self.formLayout.setWidget(1, QtGui.QFormLayout.FieldRole, self.line_obj_type)
        self.label_3 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.formLayout.setWidget(3, QtGui.QFormLayout.LabelRole, self.label_3)
        self.line_obj_loc = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_loc.setObjectName("line_obj_loc")
        self.formLayout.setWidget(3, QtGui.QFormLayout.FieldRole, self.line_obj_loc)
        self.label_4 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.formLayout.setWidget(4, QtGui.QFormLayout.LabelRole, self.label_4)
        self.line_obj_how = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_how.setObjectName("line_obj_how")
        self.formLayout.setWidget(4, QtGui.QFormLayout.FieldRole, self.line_obj_how)
        self.label_5 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.formLayout.setWidget(5, QtGui.QFormLayout.LabelRole, self.label_5)
        self.line_obj_tasks = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_tasks.setObjectName("line_obj_tasks")
        self.formLayout.setWidget(5, QtGui.QFormLayout.FieldRole, self.line_obj_tasks)
        self.label_6 = QtGui.QLabel(self.verticalLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.formLayout.setWidget(2, QtGui.QFormLayout.LabelRole, self.label_6)
        self.line_obj_room = QtGui.QLineEdit(self.verticalLayoutWidget)
        self.line_obj_room.setObjectName("line_obj_room")
        self.formLayout.setWidget(2, QtGui.QFormLayout.FieldRole, self.line_obj_room)
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

        self.retranslateUi(change_object)
        QtCore.QMetaObject.connectSlotsByName(change_object)

    def retranslateUi(self, change_object):
        change_object.setWindowTitle(QtGui.QApplication.translate("change_object", "Change object", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("change_object", "id:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_obj_id.setText(QtGui.QApplication.translate("change_object", "id_label", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("change_object", "type:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("change_object", "location:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("change_object", "feature:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("change_object", "tasks:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("change_object", "room", None, QtGui.QApplication.UnicodeUTF8))
        self.but_del_obj.setText(QtGui.QApplication.translate("change_object", "Delete", None, QtGui.QApplication.UnicodeUTF8))
        self.but_cancel_obj.setText(QtGui.QApplication.translate("change_object", "Cancel", None, QtGui.QApplication.UnicodeUTF8))
        self.but_upd_obj.setText(QtGui.QApplication.translate("change_object", "Update", None, QtGui.QApplication.UnicodeUTF8))

