#!/usr/bin/env python

""" 
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
"""
import rospy
from a50_dvl_algorithms.A50DvlAlgorithm import my_generic_sum_function
from std_msgs.msg import String
import socket
import json
from time import sleep
from std_msgs.msg import String, Int8
from dsor_msgs.msg import Measurement
from a50_dvl.msg import DVL
from a50_dvl.msg import DVLBeam

import select

class A50DvlNode():
	def __init__(self):
		"""
		Constructor for ros node
		"""

		"""
		#############################################################################
		@.@ Init node
		#############################################################################
		"""
		rospy.init_node('a50_dvl_node')

		"""
		#############################################################################
		@.@ Handy Variables
		#############################################################################
		# Declare here some variables you might think usefull -> example: self.fiic = true

		"""
		self.h_timerActivate = False
		self.tcp_ip = None
		self.tcp_port = None
		self.s_socket = None
		self.old_json = ""
		self.dvl = DVL()
		self.beam0 = DVLBeam()
		self.beam1 = DVLBeam()
		self.beam2 = DVLBeam()
		self.beam3 = DVLBeam()

		"""
	    ###########################################################################
		@.@ Dirty work of declaring subscribers, publishers and load parameters 
		############################################################################
		"""
		self.initializeSubscribers()
		self.initializePublishers()
		self.loadParams()
		self.connect()
		self.initializeTimer()

		# +.+ Initialize DVL acoustic disable
		#message = {"command": "set_config", "parameters": {"acoustic_enabled": False}}
		#message = json.dumps(message)
		#self.s_socket.send(message.encode('utf-8'))

	def connect(self):
		#print(self.tcp_ip)
		#print(self.tcp_port)
		try:
			self.s_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			self.s_socket.connect((self.tcp_ip, self.tcp_port))
			self.s_socket.settimeout(1)
		except socket.error as err:
			rospy.logerr("No route to host, DVL might be booting? {}".format(err))
			sleep(1)
			self.connect()

	def getData(self):
		raw_data = ""

		while not '\n' in raw_data:
			try:
				rec = self.s_socket.recv(1) # Add timeout for that
				if len(rec) == 0:
					rospy.logerr("Socket closed by the DVL, reopening")
					self.connect()
					continue
			except socket.timeout as err:
				rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
				self.connect()
				continue
			raw_data = raw_data + rec.decode('UTF-8')
		raw_data = self.old_json + raw_data
		self.old_json = ""
		raw_data = raw_data.split('\n')
		self.old_json = raw_data[1]
		raw_data = raw_data[0]
		return raw_data


	"""
	#####################################################################################
	@.@ Member Helper function to set up subscribers; 
	#####################################################################################
	"""
	def initializeSubscribers(self):
		rospy.loginfo('Initializing Subscribers for A50DvlNode')
		rospy.Subscriber(rospy.get_param('~topics/subscribers/dvl_enable'), Int8, self.dvlEnable)

	"""
	#####################################################################################
	@.@ Member Helper function to set up publishers; 
	#####################################################################################
	"""
	def initializePublishers(self):
		rospy.loginfo('Initializing Publishers for A50DvlNode')
		self.dvl_json_pub = rospy.Publisher(rospy.get_param('~topics/publishers/dvl_json_data'), String, queue_size=10)
		self.dvl_pub = rospy.Publisher(rospy.get_param('~topics/publishers/dvl_data'), DVL, queue_size=10)
		self.dvl_filter_pub = rospy.Publisher(rospy.get_param('~topics/publishers/velocity'), Measurement, queue_size=10)

	"""
	#####################################################################################
	@.@ Member Helper function to set up parameters; 
	#####################################################################################
	"""
	def loadParams(self):
		self.node_frequency = rospy.get_param('~node_frequency', 10)
		self.tcp_ip = rospy.get_param('~ip', "192.168.2.95")
		self.tcp_port = rospy.get_param('~port', 16171)
		self.do_log_raw_data = rospy.get_param("~do_log_raw_data", False)

	"""
	#####################################################################################
	@.@ Member helper function to set up the timer; This is a more flexible and useful form of the ros::Rate 
	#####################################################################################
	"""
	def initializeTimer(self):
		self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)
		self.h_timerActivate = True

	"""
	#####################################################################################
	@.@ Member helper function to shutdown timer;
	#####################################################################################
	"""
	def shutdownTimer(self):
		self.timer.shutdown()
		self.h_timerActivate = False

	"""
	#####################################################################################
	@.@ Callback functions/methods 
	#####################################################################################
	"""

	def dvlEnable(self, msg):
		if msg.data:
			message = {"command":"set_config","parameters":{"acoustic_enabled": True}}
			message = json.dumps(message)
			self.s_socket.send(message.encode('utf-8'))
			self.initializeTimer()
		else:
			message = {"command": "set_config", "parameters": {"acoustic_enabled": False}}
			message = json.dumps(message)
			self.s_socket.send(message.encode('utf-8'))
			self.shutdownTimer()

	# +.+ timer iter callback
	def timerIterCallback(self, event=None):

		raw_data = self.getData()
		if self.do_log_raw_data:
			rospy.loginfo(raw_data)
		# TODO: There is a problem reading json inspect later
		try:
			data = json.loads(raw_data)
		except:
			return
		if data["type"] != "velocity":
			return

		#print('Here')
		self.dvl_json_pub.publish(raw_data)

		meas = Measurement()

		self.dvl.header.stamp = rospy.Time.now()
		self.dvl.header.frame_id = "dvl_link"
		self.dvl.time = data["time"]
		self.dvl.velocity.x = data["vx"]
		self.dvl.velocity.y = data["vy"]
		self.dvl.velocity.z = data["vz"]
		self.dvl.fom = data["fom"]
		self.dvl.altitude = data["altitude"]
		self.dvl.velocity_valid = data["velocity_valid"]
		self.dvl.status = data["status"]
		self.dvl.form = data["format"]

		meas.header.stamp = rospy.Time.now()
		meas.header.frame_id = "dvl_bt"
		meas.value = [self.dvl.velocity.x, self.dvl.velocity.y, self.dvl.velocity.z]
		meas.noise = [0.0225, 0.0225, 0.0225]

		self.beam0.id = data["transducers"][0]["id"]
		self.beam0.velocity = data["transducers"][0]["velocity"]
		self.beam0.distance = data["transducers"][0]["distance"]
		self.beam0.rssi = data["transducers"][0]["rssi"]
		self.beam0.nsd = data["transducers"][0]["nsd"]
		self.beam0.valid = data["transducers"][0]["beam_valid"]

		self.beam1.id = data["transducers"][1]["id"]
		self.beam1.velocity = data["transducers"][1]["velocity"]
		self.beam1.distance = data["transducers"][1]["distance"]
		self.beam1.rssi = data["transducers"][1]["rssi"]
		self.beam1.nsd = data["transducers"][1]["nsd"]
		self.beam1.valid = data["transducers"][1]["beam_valid"]

		self.beam2.id = data["transducers"][2]["id"]
		self.beam2.velocity = data["transducers"][2]["velocity"]
		self.beam2.distance = data["transducers"][2]["distance"]
		self.beam2.rssi = data["transducers"][2]["rssi"]
		self.beam2.nsd = data["transducers"][2]["nsd"]
		self.beam2.valid = data["transducers"][2]["beam_valid"]

		self.beam3.id = data["transducers"][3]["id"]
		self.beam3.velocity = data["transducers"][3]["velocity"]
		self.beam3.distance = data["transducers"][3]["distance"]
		self.beam3.rssi = data["transducers"][3]["rssi"]
		self.beam3.nsd = data["transducers"][3]["nsd"]
		self.beam3.valid = data["transducers"][3]["beam_valid"]

		self.dvl.beams = [self.beam0, self.beam1, self.beam2, self.beam3]

		self.dvl_pub.publish(self.dvl)
		self.dvl_filter_pub.publish(meas)



def main():

	a50Dvl = A50DvlNode()

	# +.+ Added to work with timer -> going into spin; let the callbacks do all the work

	rospy.spin()

if __name__ == '__main__':
	main()
