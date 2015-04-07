# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'animator.ui'
#
# Created: Sat Dec 20 15:47:50 2014
#      by: PyQt4 UI code generator 4.10.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(504, 280)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.gridLayout = QtGui.QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))

        self.yValueSlider = QtGui.QSlider(self.centralwidget)
        self.yValueSlider.setOrientation(QtCore.Qt.Vertical)
        self.yValueSlider.setObjectName(_fromUtf8("yValueSlider"))
        self.gridLayout.addWidget(self.yValueSlider, 1, 1, 1, 1)
        self.yValueSlider.setRange(-45, 45);

        self.yTargetSlider = QtGui.QSlider(self.centralwidget)
        self.yTargetSlider.setOrientation(QtCore.Qt.Vertical)
        self.yTargetSlider.setObjectName(_fromUtf8("yTargetSlider"))
        self.gridLayout.addWidget(self.yTargetSlider, 1, 4, 1, 1)
        self.yTargetSlider.setRange(-45, 45);

        self.xValueSlider = QtGui.QSlider(self.centralwidget)
        self.xValueSlider.setOrientation(QtCore.Qt.Horizontal)
        self.xValueSlider.setObjectName(_fromUtf8("xValueSlider"))
        self.gridLayout.addWidget(self.xValueSlider, 8, 1, 1, 1)
        self.xValueSlider.setRange(0, 48);

        self.xTargetSlider = QtGui.QSlider(self.centralwidget)
        self.xTargetSlider.setOrientation(QtCore.Qt.Horizontal)
        self.xTargetSlider.setObjectName(_fromUtf8("xTargetSlider"))
        self.gridLayout.addWidget(self.xTargetSlider, 8, 4, 1, 1)
        self.xTargetSlider.setRange(-90, 90);

        MainWindow.setCentralWidget(self.centralwidget)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "Face Animation", None))
        # self.label_13.setText(_translate("MainWindow", "Current Target:", None))
        # self.label_10.setText(_translate("MainWindow", "Confusion Target", None))
        # self.label_2.setText(_translate("MainWindow", "More Confused", None))
        # self.label.setText(_translate("MainWindow", "Less Confused", None))
        # self.label_11.setText(_translate("MainWindow", "Current Value:", None))
        # self.label_4.setText(_translate("MainWindow", "Current Velocity:", None))
        # self.label_5.setText(_translate("MainWindow", "Less Confused", None))
        # self.label_8.setText(_translate("MainWindow", "Faster", None))
        # self.label_6.setText(_translate("MainWindow", "More Confused", None))
        # self.label_9.setText(_translate("MainWindow", "Velocity (FPS)", None))
        # self.label_7.setText(_translate("MainWindow", "Slower", None))
        # self.label_3.setText(_translate("MainWindow", "Confusion Value", None))

