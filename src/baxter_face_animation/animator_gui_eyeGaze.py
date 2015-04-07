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
        self.connect(self.xTargetSlider,
                     SIGNAL("valueChanged(int)"),
                     self.xTargetChanged) 
        self.connect(self.xValueSlider,
                     SIGNAL("valueChanged(int)"),
                     self.xValueChanged)

        self.timer = QTimer()
        self.timer.start(100)
        self.connect(self.timer,
                     SIGNAL("timeout()"), self.roscb)

        # self.wobbler = move_head.Wobbler()
        self.value_publisher = rospy.Publisher("/eyeGaze/value/command", Int32, queue_size=10)
        self.target_publisher = rospy.Publisher("/eyeGaze/target/command", Int32, queue_size=10)



    def roscb(self):
        rospy.sleep(0.1)

    # def displayConfusionValue(self, value):
    #     value = value.data
    #     self.confusionValueLabel.setText(str(value))

    # def displayConfusionTarget(self, target):
    #     target = target.data
    #     self.confusionValueLabel.setText(str(target))


    # def displayConfusionVelocity(self, velocity):
    #     velocity = velocity.data
    #     self.confusionVelocityLabel.setText(str(velocity))

    def xValueChanged(self, value):
        self.value_publisher.publish(value)

    def xTargetChanged(self, value):
        self.target_publisher.publish(value)

def main():
    app = basewindow.makeApp()
    rospy.init_node('animator_gui', anonymous=True)
    wnd = MainWindow()
    wnd.show()

    print rospy.is_shutdown()

    app.exec_()
if __name__ == "__main__":
    main()
