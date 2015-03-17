#!/usr/bin/env python

# Copyright (c) 2014, Andoni Aguirrezabal
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# ROS
import rospy
from sensor_msgs.msg import Range

# GUI
from matplotlib import pyplot as plt


class MainApplication:
    def __init__(self):
        self.minVal = 0
        self.maxVal = 1

        self.leftSensorVal = 0
        self.rightSensorVal = 0

        self.BaxterJointNum = [1, 2]
        self.BaxterJointName = ["Left", "Right"]
        pass

    def __del__(self):
        pass

    def createWidgets(self):
        plt.ion()
        plt.show()

    def updateGraph(self):
        plt.clf()
        plt.ylim(self.minVal, self.maxVal)
        plt.bar(1, self.maxVal - self.leftSensorVal, color='g', align='center')
        plt.bar(2, self.maxVal - self.rightSensorVal, color='r', align='center')
        plt.xticks(self.BaxterJointNum, self.BaxterJointName)
        plt.draw()

    def leftSensorCallback(self, data):
        self.minVal = data.min_range
        self.maxVal = data.max_range
        if data.range > data.max_range:
            self.leftSensorVal = self.maxVal - 0.00000001
        else:
            self.leftSensorVal = data.range

    def rightSensorCallback(self, data):
        if data.range > data.max_range:
            self.rightSensorVal = self.maxVal - 0.00000001
        else:
            self.rightSensorVal = data.range

    def main(self):
        rospy.init_node('baxterdistanceviz', anonymous=True)

        rospy.Subscriber("/robot/range/left_hand_range/state", Range, self.leftSensorCallback)
        rospy.Subscriber("/robot/range/right_hand_range/state", Range, self.rightSensorCallback)

        self.createWidgets()

        while not rospy.is_shutdown():
            self.updateGraph()
            # print "Left Distance: ", self.leftSensorVal
            # print "Right Distance: ", self.rightSensorVal

if __name__ == "__main__":
    try:
        app = MainApplication()
        app.main()
    except rospy.ROSInterruptException:
        pass
