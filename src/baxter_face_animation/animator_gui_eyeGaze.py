#!/usr/bin/env python

import basewindow
import animator_ui_eyeGaze
from PyQt4.QtCore import SIGNAL, QTimer
from PyQt4.QtGui import QMainWindow
import rospy
from std_msgs.msg import Int32


class MainWindow(QMainWindow, animator_ui_eyeGaze.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        # self.connect(self.xTargetSlider,
        #              SIGNAL("valueChanged(int)"),
        #              self.xTargetChanged) 
        self.connect(self.xValueSlider,
                     SIGNAL("valueChanged(int)"),
                     self.xValueChanged)

        self.connect(self.yTargetSlider,
                     SIGNAL("valueChanged(int)"),
                     self.yTargetChanged) 
        self.connect(self.yValueSlider,
                     SIGNAL("valueChanged(int)"),
                     self.yValueChanged)

        self.timer = QTimer()
        self.timer.start(100)
        self.connect(self.timer,
                     SIGNAL("timeout()"), self.roscb)

        # self.wobbler = move_head.Wobbler()
        self.valuex_publisher = rospy.Publisher("/eyeGaze/valuex/command", Int32, queue_size=10)
        # self.targetx_publisher = rospy.Publisher("/eyeGaze/targetx/command", Int32, queue_size=10)
        self.valuey_publisher = rospy.Publisher("/eyeGaze/valuey/command", Int32, queue_size=10)
        self.targety_publisher = rospy.Publisher("/eyeGaze/targety/command", Int32, queue_size=10)



    def roscb(self):
        rospy.sleep(0.1)

    def yValueChanged(self, value):
        self.valuey_publisher.publish(value)

    def yTargetChanged(self, value):
        self.targety_publisher.publish(value)

    def xValueChanged(self, value):
        self.valuex_publisher.publish(value)

    # def xTargetChanged(self, value):
    #     self.targetx_publisher.publish(value)

def main():
    app = basewindow.makeApp()
    rospy.init_node('animator_gui', anonymous=True)
    wnd = MainWindow()
    wnd.show()

    print rospy.is_shutdown()

    app.exec_()
if __name__ == "__main__":
    main()
