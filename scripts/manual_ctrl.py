#!/usr/bin/env python3
# -*- encoding: iso-8859-1 -*-
import numpy as np
import roslib
import sys
import cv2
import rospy
from omni_ros.msg import m_motion, m_motion_n, m_flags
from nav_msgs.msg import Odometry
import PySimpleGUI as sg
import matplotlib.pyplot as plt
import rospkg

class c_manual_ctrl:

	def __init__(self):

		self.x = 0
		self.y = 0
		self.t = 0

		#Carrega endereco do package omni_ros_gui
		rospac = rospkg.RosPack()
		self.omni_ros_gui_path = rospac.get_path("omni_ros_gui")
		self.omni_ros_gui_path += "/scripts"

		layout = [[sg.Text('Robotics Remote Control')],
				[sg.RealtimeButton('LF'), sg.RealtimeButton('F'), sg.RealtimeButton('RF')],
				[sg.RealtimeButton('L'), sg.T(' '  * 23), sg.RealtimeButton('R')],
				[sg.RealtimeButton('LB'), sg.RealtimeButton('B'), sg.RealtimeButton('RB')],
				[sg.T('')],
				[sg.RealtimeButton('START'), sg.RealtimeButton('PAUSE'), sg.RealtimeButton('STOP')],
				[sg.T('')],
				[sg.RealtimeButton('ODOM')],
				[sg.T('')],
				[sg.Slider(range=(5,20), orientation='h', size=(10,20), key='slider'), sg.RealtimeButton('CW'), sg.RealtimeButton('CCW')],
				[sg.Quit(button_color=('black', 'orange'))]
				]

		self.window = sg.Window('Robotics Remote Control', layout, no_titlebar=False, auto_size_buttons=False, keep_on_top=True, grab_anywhere=True)

		#Cria Publisher
		self.motion_pub = rospy.Publisher("t_motion", m_motion, queue_size = 10)
		self.flags_pub = rospy.Publisher("t_flags", m_flags, queue_size = 10)

		#Cria Subscriber
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self._get_odom)

	def __del__(self):
		#Fecha arquivo
		#self.fout.close
		self.window.close()
		print("Close control")

	def get_x(self):
		return self.x

	def get_y(self):
		return self.y

	def get_t(self):
		return self.t

	def get_window(self):
		return self.window

	def _get_odom(self, data):
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y
		self.t = data.pose.pose.orientation.w #TODO

def main(args):

	## Variáveis de controle
	start_aquisition = False
	pause_aquisition = False
	stop_aquisition = False
	reset_odom = False
	send_data = False

	## Criação da classe
	cmc = c_manual_ctrl()

	## Criação do Nó
	rospy.init_node('n_manual_ctrl', anonymous=True)
	rate = rospy.Rate(15)

	## Mensagens de movimento
	_m_motion = m_motion()
	_m_motion1 = m_motion_n()
	_m_motion2 = m_motion_n()
	_m_motion3 = m_motion_n()
	_m_flags = m_flags()

	while not rospy.is_shutdown():
		event, values = cmc.get_window().read(timeout=0)
		sz_slider = int(values['slider'])

		#print ("x: " + str(cmc.get_x()) + " y: " + str(cmc.get_y()) + " t: " + str(cmc.get_t()) )

		###############################################
		##  Tratamento dos botões
		###############################################
		if event in ('Quit', sg.WIN_CLOSED):
			break

		if event == "F":
			#print("Forward")
			_m_motion1.rpm = sz_slider
			_m_motion1.angle = -1.0
			_m_motion2.rpm = 0.0
			_m_motion2.angle = 0.0
			_m_motion3.rpm = sz_slider
			_m_motion3.angle = 1.0
			send_data = True;

		if event == "L":
			#print("Left")
			_m_motion1.rpm = 0.0
			_m_motion1.angle = 0.0
			_m_motion2.rpm = 0.0
			_m_motion2.angle = 0.0
			_m_motion3.rpm = 0.0
			_m_motion3.angle = 0.0

		if event == "R":
			#print("Right")
			_m_motion1.rpm = 0.0
			_m_motion1.angle = 0.0
			_m_motion2.rpm = 0.0
			_m_motion2.angle = 0.0
			_m_motion3.rpm = 0.0
			_m_motion3.angle = 0.0

		if event == "B":
			#print("Reverse")
			_m_motion1.rpm = sz_slider
			_m_motion1.angle = 1.0
			_m_motion2.rpm = 0.0
			_m_motion2.angle = 0.0
			_m_motion3.rpm = sz_slider
			_m_motion3.angle = -1.0
			send_data = True;

		if event == "CW":
			#print("Reverse")
			_m_motion1.rpm = sz_slider
			_m_motion1.angle = -1.0
			_m_motion2.rpm = sz_slider
			_m_motion2.angle = -1.0
			_m_motion3.rpm = sz_slider
			_m_motion3.angle = -1.0
			send_data = True;

		if event == "CCW":
			#print("Reverse")
			_m_motion1.rpm = sz_slider
			_m_motion1.angle = 1.0
			_m_motion2.rpm = sz_slider
			_m_motion2.angle = 1.0
			_m_motion3.rpm = sz_slider
			_m_motion3.angle = 1.0
			send_data = True;

		if event == "__TIMEOUT__":
			#print("TimeOut")
			_m_motion1.rpm = 0.0
			_m_motion1.angle = 0.0
			_m_motion2.rpm = 0.0
			_m_motion2.angle = 0.0
			_m_motion3.rpm = 0.0
			_m_motion3.angle = 0.0
			#send_data = True;

		if event == "ODOM":
			reset_odom = True
			send_data = True;

		if event == "START":
			start_aquisition = True
			send_data = True;

		if event == "PAUSE":
			pause_aquisition = True
			send_data = True;

		if event == "STOP":
			stop_aquisition = True
			send_data = True;


		###############################################
		##  Chamada de funções
		###############################################
		if reset_odom == True:
			print("Reset Odometry")
			reset_odom = False
			_m_flags.reset_odom = True
			_m_flags.stop = True
			cmc.flags_pub.publish(_m_flags)

		if start_aquisition == True:
			print("Start data aquisition")
			start_aquisition = False
			_m_flags.reset_odom = False
			_m_flags.stop = False
			_m_flags.aquisition = 1
			cmc.flags_pub.publish(_m_flags)

		if pause_aquisition == True:
			print("Pause data aquisition")
			pause_aquisition = False
			_m_flags.reset_odom = False
			_m_flags.aquisition = 2
			cmc.flags_pub.publish(_m_flags)

		if stop_aquisition == True:
			print("Stop data aquisition")
			stop_aquisition = False
			_m_flags.reset_odom = False
			_m_flags.stop = True
			_m_flags.aquisition = 0
			cmc.flags_pub.publish(_m_flags)


		###############################################
		##  Envio de mensagens
		###############################################
		if send_data == True:
			_m_motion.motion1 = _m_motion1
			_m_motion.motion2 = _m_motion2
			_m_motion.motion3 = _m_motion3
			cmc.motion_pub.publish(_m_motion)
			send_data = False

		rate.sleep()

if __name__ == '__main__':
	main(sys.argv)




