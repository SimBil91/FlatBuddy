# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'webstream.ui'
#
# Created: Wed Sep 23 01:02:12 2015
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_webstream(object):
    def setupUi(self, webstream):
        webstream.setObjectName("webstream")
        webstream.resize(640, 480)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(webstream.sizePolicy().hasHeightForWidth())
        webstream.setSizePolicy(sizePolicy)
        self.img_stream = QtGui.QLabel(webstream)
        self.img_stream.setGeometry(QtCore.QRect(0, 0, 640, 480))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.img_stream.sizePolicy().hasHeightForWidth())
        self.img_stream.setSizePolicy(sizePolicy)
        self.img_stream.setText("")
        self.img_stream.setScaledContents(True)
        self.img_stream.setObjectName("img_stream")

        self.retranslateUi(webstream)
        QtCore.QMetaObject.connectSlotsByName(webstream)

    def retranslateUi(self, webstream):
        webstream.setWindowTitle(QtGui.QApplication.translate("webstream", "FlatBuddy Stream", None, QtGui.QApplication.UnicodeUTF8))

