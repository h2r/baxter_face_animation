#!/usr/bin/env python

import basewindow
import animator_ui
from PyQt4.QtCore import SIGNAL, QTimer
from PyQt4.QtGui import QMainWindow
import rospy
from std_msgs.msg import Int32



class MainWindow(QMainWindow, animator_ui.Ui_MainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.connect(self.confusionValueSlider,
                     SIGNAL("valueChanged(int)"),
                     self.confusionValueChanged) 
        self.connect(self.confusionTargetSlider,
                     SIGNAL("valueChanged(int)"),
                     self.confusionTargetChanged)

        self.timer = QTimer()
        self.timer.start(100)
        self.connect(self.timer,
                     SIGNAL("timeout()"), self.roscb)

        self.value_publisher = rospy.Publisher("/confusion/value/command", Int32,
                                               queue_size=10)

        self.target_publisher = rospy.Publisher("/confusion/target/command", Int32,
                                                queue_size=10)


        self.value_subscriber = rospy.Subscriber("/confusion/value/state", Int32,
                                                 self.displayConfusionValue)
        self.target_subscriber = rospy.Subscriber("/confusion/target/state", Int32,
                                                  self.displayConfusionTarget)

    def roscb(self):
        rospy.sleep(0.1)

    def displayConfusionValue(self, value):
        value = value.data
        self.confusionValueLabel.setText(str(value))

    def displayConfusionTarget(self, target):
        target = target.data
        self.confusionValueLabel.setText(str(target))


    def displayConfusionVelocity(self, velocity):
        velocity = velocity.data
        self.confusionVelocityLabel.setText(str(velocity))

    def confusionTargetChanged(self, value):
        self.target_publisher.publish(value)

    def confusionValueChanged(self, value):
        self.value_publisher.publish(value)

def main():
    app = basewindow.makeApp()
    rospy.init_node('animator_gui', anonymous=True)
    wnd = MainWindow()
    wnd.show()

    print rospy.is_shutdown()

    app.exec_()
if __name__ == "__main__":
    main()
