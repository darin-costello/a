#!/usr/bin/python

import sys, rospy
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from sphero_swarm_node.msg import SpheroTwist
from multi_apriltags_tracker.msg import april_tag_pos
from kris_alder_algo.msg import changeLeader
from geometry_msgs.msg import Pose2D


RADIUS = 45

class AgentControl(QtGui.QWidget):
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        width = 960 #TODO change to query param
        height = 800 #TODO change to query param
        self.resize(width, height)
        self.initUI()
        self.leader = -1
        rospy.init_node("agent_control", anonymous = True)
        self.tagLocSub = rospy.Subscriber('/april_tag_pos', april_tag_pos, self.tagPosCallback)
        self.changeLeadPub = rospy.Publisher('/change_leader', changeLeader, queue_size = 1)
        self.blackPen = QtGui.QPen(QtCore.Qt.black, 2, QtCore.Qt.SolidLine)
        self.bluePen = QtGui.QPen(QtCore.Qt.blue, 2, QtCore.Qt.SolidLine)
        self.agentMap = {}


    def initUI(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.white)
        self.setPalette(p)
        self.show()


    def tagPosCallback(self, msg):
        for i in range(0, len(msg.id)):
            self.agentMap[msg.id[i]] = msg.pose[i]
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter();
        painter.begin(self)
        for key, value in self.agentMap.iteritems():
            rect = QRect(value.x - RADIUS/2, value.y - RADIUS/2, RADIUS, RADIUS)
            if key < 60:
                painter.setPen(self.bluePen)
                painter.drawEllipse(rect)
                painter.drawText(rect, QtCore.Qt.AlignCenter, str(key))
            else:
                painter.setPen(self.blackPen)
                painter.drawRect(rect)
                painter.drawText(rect, QtCore.Qt.AlignCenter, str(key))

        if self.leader != -1:
            value = self.agentMap[self.leader]
            rectF = QRectF(value.x - RADIUS/2, value.y - RADIUS/2, RADIUS, RADIUS)
            painter.setBrush(Qt.blue)
            painter.drawEllipse(rectF)
            painter.setPen(self.blackPen)
            painter.drawText(rectF, QtCore.Qt.AlignCenter, str(self.leader))
        painter.end()


    def mousePressEvent(self, QMouseEvent):
        mouse_state = QMouseEvent.button()

        self.mouseX = QMouseEvent.x()
        self.mouseY = QMouseEvent.y()

        if  mouse_state == QtCore.Qt.LeftButton:
            newleader = -1;
            for key, value in self.agentMap.iteritems():
                dist = self.sqDist((value.x, value.y), (self.mouseX, self.mouseY))
                if(dist < (RADIUS * RADIUS)):
                    if key < 60:
                        newleader = key
                    break
            self.setLeader(newleader, self.mouseX, self.mouseY)

    def setLeader(self, newleader, x, y):
        lid = -1
        lx = -1
        ly = -1
        if newleader == -1:

            if self.leader == -1:
                return
            lid = self.leader
            lx = x
            ly = y
        else:
            if newleader != self.leader:
                lid = newleader
                lx = self.agentMap[lid].x
                ly = self.agentMap[lid].y
                self.leader = newleader
            else:
                self.leader = -1

        l = changeLeader()
        l.id = lid
        pos = Pose2D()
        pos.theta = 0
        pos.x = lx
        pos.y = ly
        l.pose = pos
        self.changeLeadPub.publish(l)

    def sqDist(self, t1, t2):
        diffx = t1[0] - t2[0]
        diffy = t1[1] -t2[1]
        return (diffx * diffx) + (diffy * diffy)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    w = AgentControl()
    w.show()
    sys.exit(app.exec_())
