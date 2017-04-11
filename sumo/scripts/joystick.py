#!/usr/bin/env python
import pygame
import sys
import time
from pygame.locals import *
import rospy
from rospy_message_converter import message_converter
from std_msgs.msg import String
from sumo.msg import Joystick


if __name__ == '__main__':
    keyPublisher= rospy.Publisher('Joykey', Joystick, queue_size = 10)
    pygame.joystick.init()
    print pygame.joystick.get_count()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    buttonStatus = ["False" for x in range(8)]
    buttonStatus.append('0')

    buttons = [x for x in range(0,9)]

    buttonName = ['key1','key2','key3','key4','l1','r1','l2','r2','lstick']

    buttonMap = dict(zip(buttons,buttonName))

    buttonMsg = dict(zip(buttonName,buttonStatus))
    buttonMsg['lstick'] = '0'

    pygame.init()
    done = False
    diff = 0.5
    rospy.init_node("joystick")
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
            if event.type == pygame.JOYBUTTONDOWN:
                print "daba"
                buttonMsg[buttonMap[event.button]] = "True"
            if event.type == pygame.JOYBUTTONUP:
                print event
                buttonMsg[buttonMap[event.button]] = "False"
            if event.type == pygame.JOYAXISMOTION:
                buttonMsg['lstick'] = '0'
                if event.axis == 1:
                    if event.value <-1*diff:
                        buttonMsg['lstick'] = "w"
                    if event.value >diff:
                        buttonMsg['lstick'] = "s"
                elif event.axis == 0:
                    if event.value <-1*diff:
                        buttonMsg['lstick'] = "a"
                    if event.value >diff:
                        buttonMsg['lstick'] = "d"
        buttonMessage = message_converter.convert_dictionary_to_ros_message('penalty_kick/Joystick',buttonMsg)
        rospy.loginfo(buttonMessage)
        keyPublisher.publish(buttonMessage)
        print "-------------------------"
