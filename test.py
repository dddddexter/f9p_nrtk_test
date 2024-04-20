#!/usr/bin/python3
# -*- coding: UTF-8 -*-
# f9p_rtk.py
# by Lee in 2023/09/21
# ubuntu18.04 + python3.6
#test 20240421
import spidev
import io
import rospy
import time

import http.client
from http.client import HTTPException
from http.client import HTTPConnection
from http.client import IncompleteRead
import socket

from base64 import b64encode
from threading import Thread
import serial
from sensor_msgs.msg import NavSatFix

import logging
import importlib

## UART configuration
f9p_uart2_port = '/dev/ttyTHS0'
f9p_uart2_baud = 38400

## NTRIP configuration
_ntrip_server = '203.107.45.154'
_ntrip_port = 8002
_ntrip_mountpoint = 'AUTO'
#_ntrip_mountpoint = 'RTCM32_GGB'
_ntrip_username = 'qxwjnj00718'
_ntrip_password = '5a6ddf8'

#############################################################################################
def NiceToICY(self):
    class InterceptedHTTPResponse():
        pass
    line = self.fp.readline().replace(b"ICY 200 OK\r\n", b"HTTP/1.0 200 OK\r\n")
    InterceptedSelf = InterceptedHTTPResponse()
    InterceptedSelf.fp = io.BufferedReader(io.BytesIO(line))
    InterceptedSelf.debuglevel = self.debuglevel
    InterceptedSelf._close_conn = self._close_conn
    return ORIGINAL_HTTP_CLIENT_READ_STATUS(InterceptedSelf)

ORIGINAL_HTTP_CLIENT_READ_STATUS = http.client.HTTPResponse._read_status
http.client.HTTPResponse._read_status = NiceToICY


def patch_http_response_read(func):
    def inner(*args):
        try:
            return func(*args)
        except http.client.IncompleteRead as e:
            return e.partial
    return inner
http.client.HTTPResponse.read = patch_http_response_read(http.client.HTTPResponse.read)

# Send RTCM Message to UART2 of Ublox-F9P module
uart_port = f9p_uart2_port
uart_baud = f9p_uart2_baud
uart_isopened = False
try:
    uart2_f9p = serial.Serial(uart_port, uart_baud, timeout = 0)
    uart_isopened = True
except:
    rospy.loginfo("Open %s failed! Please make sure you entered right port and baudrate!" % (uart_port))

def _parse_degrees(nmea_data):
	# Parse a NMEA lat/long data pair 'dddmm.mmmm' into a pure degrees value.
	# Where ddd is the degrees, mm.mmmm is the minutes.
	if nmea_data is None or len(nmea_data) < 3:
		return None
	raw = float(nmea_data)
	deg = raw // 100
	minutes = raw % 100
	return deg + minutes / 60

def _parse_int(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return int(nmea_data)

def _parse_float(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return float(nmea_data)

def _parse_str(nmea_data):
	if nmea_data is None or nmea_data == "":
		return None
	return str(nmea_data)

class ntripconnect(Thread):
	def __init__(self, ntc):
		super(ntripconnect, self).__init__()
		self.ntc = ntc
		self.connected = False
		self.connection = HTTPConnection(self.ntc.ntrip_server, self.ntc.ntrip_port, timeout=5)
		self.stop = False

	def do_connect(self):
		if self.ntc.gpgga is not None:
			self.connection = HTTPConnection(self.ntc.ntrip_server, self.ntc.ntrip_port, timeout=5)
			usr_pwd = self.ntc.ntrip_username + ':' + self.ntc.ntrip_password
			headers = {
				'Ntrip-Version': 'Ntrip/1.0',
				'User-Agent': 'NTRIP ntrip_ros',
				'Connection': 'close',
				#python3.8 syntax requirment
				'Authorization': 'Basic ' + str(b64encode(usr_pwd.encode('utf-8')),'utf-8')
			}
			try:				
				self.connection.request('GET', '/' + self.ntc.ntrip_mountpoint, self.ntc.gpgga, headers)
				self.connected = True
				print (" ------ In connecting process: Connected ------ ")
				return True
			except KeyboardInterrupt:
				print (" ------ In connecting process: Keyboard interrupt ------ ")
				self.connected = False
				self.stop = True
				return False
			except (HTTPException, socket.error) as e:
				print (" ------ In connecting process: HTTP error ------ ")
				print (e)
				return False
		else:
			return False
	
	def do_get(self):
		if self.ntc.gpgga is not None:
			# self.connection = HTTPConnection(self.ntc.ntrip_server, self.ntc.ntrip_port, timeout=5)
			usr_pwd = self.ntc.ntrip_username + ':' + self.ntc.ntrip_password
			headers = {
				'Ntrip-Version': 'Ntrip/1.0',
				'User-Agent': 'NTRIP ntrip_ros',
				'Connection': 'close',
				#python3.8 syntax requirement
				'Authorization': 'Basic ' + str(b64encode(usr_pwd.encode('utf-8')),'utf-8')
			}
			try:				
				self.connection.request('GET', '/' + self.ntc.ntrip_mountpoint, self.ntc.gpgga, headers)
				return True
			except KeyboardInterrupt:
				print (" ------ In connecting process: Keyboard interrupt ------ ")
				self.connected = False
				self.stop = True
				return False
			except (HTTPException, socket.error) as e:
				print (" ------ In connecting process: HTTP error ------ ")
				print (e)
				return False
		else:
			return False

	def run(self):
		while not self.connected:
			try:
				self.connected = self.do_connect()
			except KeyboardInterrupt:
				print (" Keyboard interrupt ")
				self.connected = False
				self.stop = True
			time.sleep(0.5)

		response = self.connection.getresponse()

		while (not self.stop) and self.connected:
			if response.status != 200:
				rospy.logerr("HTTP status is not 200! Reconnecting... ")
				self.do_connect()
				response = self.connection.getresponse()
				continue

			try:
				data = response.read(1)  # [1] Sync code : first byte should be '211' , that is 'D3' in hex
                                
			except (HTTPException, socket.error) as e:
				print (" Reconnecting...")
				if self.do_connect():
					response = self.connection.getresponse()
				continue
			except KeyboardInterrupt:
				print ("Keyboard interrupt!")
				self.stop = True
				break
			if data[0] != 211:
				rospy.logwarn("Header not in sync, continue!")
				continue
			# [2] Reversed code : 6bit , 000000
			# [3] Msg length : 10bit
			# Read 2nd byte , including 2 bits in Msg length
			# Read 3rd byte , including 8 bits remaining in Msg length
			# Got msg length : 2bit + 8bit = 10bit
			# [4] Data body : read from 4th byte
			# [5] CRC : read last 3 bytes
			# RTCM data = Sync code + Reversed code + Msg length + Data body + CRC
			buf = ""
				
			if data[0] == 211:
			#if False:
				t0 = time.time()
				buf = []
				buf.append(data[0])
				data = response.read(2)
				buf.append(data[0])
				buf.append(data[1])
				cnt = data[0] * 256 + data[1]
				data = response.read(2)
				buf.append(data[0])
				buf.append(data[1]) 
				cnt = cnt + 1
				for x in range(cnt):
					data = response.read(1)
					buf.append(data[0])
				#rmsg.message = buf   #rtcm_data = data + chr(l1) + chr(l2) + pkt + parity						
				rtcm_data = buf
				#t1 = time.time()
				#print('time is : ', t1-t0)
				if uart_isopened and (self.ntc.fix_quality != 3):
					# Send to UART2 of Ublox-F9P
					try:
						a = uart2_f9p.write(rtcm_data)
						buf = []
					except:
						print ("did not send!")
						pass
				elif not uart_isopened:
					rospy.logwarn("Uart not opened! RTCM data won't be sent!")
				elif self.ntc.fix_quality == 3:
					rospy.logwarn("Position not fixed! RTCM data won't be sent!")
				

		self.connection.close()
		self.connected = False
		print ("End of the process.")



class GPS:
	"""GPS parsing module.	Can parse simple NMEA data sentences from SPI
	GPS modules to read latitude, longitude, and more.
	"""
	def __init__(self,spiport):
		self.spiport = spiport
		# Initialize null starting values for GPS attributes.
		self.datatype = None
		self.timestamp_utc = None
		self.latitude = None
		self.longitude = None
		self.fix_quality = None
		self.fix_quality_3d = None
		self.satellites = None
		self.satellites_prev = None
		self.horizontal_dilution = None
		self.altitude_m = None
		self.height_geoid = None
		self.speed_knots = None
		self.track_angle_deg = None
		self.sats = None
		self.isactivedata = None
		self.true_track = None
		self.mag_track = None
		self.sat_prns = None
		self.sel_mode = None
		self.pdop = None
		self.hdop = None
		self.vdop = None
		self.total_mess_num = None
		self.mess_num = None
		self._raw_sentence = None
		self.debug = None
		self.gpgga = None
		self.gpgga_flag = False
		self.ntrip_server = _ntrip_server
		self.ntrip_port = _ntrip_port
		self.ntrip_username = _ntrip_username
		self.ntrip_password = _ntrip_password
		self.ntrip_mountpoint = _ntrip_mountpoint

		self.connection = None
		self.connection = ntripconnect(self)
		self.connection.start()

	def readSentence(self,spiport):
		s = ""
		buf = ""
		isstart = False
		while True:
			c = chr(spiport.readbytes(1)[0])
			if isstart == False :
				if c == '$' :
					isstart = True
					buf += c
			else :
				if c !='\n' :
					buf += c
				else :
					buf += c
					isstart = False
					break
		return buf

	def update(self):
		"""Check for updated data from the GPS module and process it
		accordingly.  Returns True if new data was processed, and False if
		nothing new was received.
		"""
		# Grab a sentence and check its data type to call the appropriate
		# parsing function.
		try:
			sentence = self.readSentence(self.spiport)
			#logging.warning(sentence)
			self.datatype = sentence.split(',')[0]
			#if self.datatype == "$GPRMC" or self.datatype == "$GNRMC":
				#logging.warning(sentence)
			if self.datatype == "$GPGGA" or self.datatype == "$GNGGA":
				#logging.warning(sentence)
				self.gpgga = sentence
				print("******************************************************")
				
		except UnicodeError:
			logging.warning('%%%%%%%%%%%')
			return None

		if sentence is None:
			logging.warning('&&&&&&&&&&&')
			return False

		self.datatype = sentence.split(',')[0]
		if self.datatype == "$GPGGA" or self.datatype == "$GNGGA":
			self.parse_gpgga(sentence)
			self.gpgga_flag = True
		else:
			self.gpgga_flag = False
		return True


	def parse_gpgga(self,sentence):
		data = sentence.split(",")
		if data is None or len(data) != 15 or (data[0] == ""):
			return	# Unexpected number of params.
		# Parse fix time.
		time_utc = int(_parse_float(data[1]))
		if time_utc is not None:
			hours = time_utc // 10000 + 8
			mins = (time_utc // 100) % 100
			secs = time_utc % 100
			# Set or update time to a friendly python time struct.
			if self.timestamp_utc is not None:
				self.timestamp_utc = time.struct_time(
					(
						self.timestamp_utc.tm_year,
						self.timestamp_utc.tm_mon,
						self.timestamp_utc.tm_mday,
						hours,
						mins,
						secs,
						0,
						0,
						-1,
					)
				)
			else:
				self.timestamp_utc = time.struct_time(
					(0, 0, 0, hours, mins, secs, 0, 0, -1)
				)
		# Parse latitude and longitude.
		self.latitude = _parse_degrees(data[2])
		if self.latitude is not None and data[3] is not None and data[3].lower() == "s":
			self.latitude *= -1.0
		self.longitude = _parse_degrees(data[4])
		if (
			self.longitude is not None
			and data[5] is not None
			and data[5].lower() == "w"
		):
			self.longitude *= -1.0
		# Parse out fix quality and other simple numeric values.
		self.fix_quality = _parse_int(data[6])
		self.satellites = _parse_int(data[7])
		self.horizontal_dilution = _parse_float(data[8])
		self.altitude_m = _parse_float(data[9])
		self.height_geoid = _parse_float(data[11])
		return 0

		
#start to run
rospy.init_node('gps_rtk', anonymous=True)
gps_pub = rospy.Publisher('gps_ori', NavSatFix, queue_size=10)
spiport =spidev.SpiDev()
spiport.open(2,0)#（2,0）for spi1，（0,0）for spi0
spiport.max_speed_hz = 1000000 # 1000000 7800000,15600000,62400000....

i = 0
gps = GPS(spiport)
print('--------test-----------')

importlib.reload(logging)
logging.basicConfig(level=logging.WARNING, filename='/home/antobot/test.log', datefmt='%Y-%m-%d %H:%M:%S', format='%(asctime)s.%(msecs)03d: '
'[%(filename)s: %(lineno)d]: %(message)s')

while not rospy.is_shutdown():
	gps.update()
	
	if gps.gpgga_flag == True:
		
		print(i)
		
		if gps.timestamp_utc is not None:
			print(
					"Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
						gps.timestamp_utc.tm_mon,	# Grab parts of the time from the
						gps.timestamp_utc.tm_mday,	# struct_time object that holds
						gps.timestamp_utc.tm_year,	# the fix time.	 Note you might
						gps.timestamp_utc.tm_hour,	# not get all data like year, day,
						gps.timestamp_utc.tm_min,	# month!
						gps.timestamp_utc.tm_sec,
					)
				)
		if gps.latitude is not None:
			print("Latitude: {0:.6f} degrees".format(gps.latitude))
		if gps.longitude is not None:
			print("Longitude: {0:.6f} degrees".format(gps.longitude))
		if gps.fix_quality is not None:
			print("Fix quality: {}".format(gps.fix_quality))
		# Some attributes beyond latitude, longitude and timestamp are optional
		# and might not be present.	 Check if they're None before trying to use!
		if gps.satellites is not None:
			print("# satellites: {}".format(gps.satellites))
		if gps.altitude_m is not None:
			print("Altitude: {}gpsfix meters".format(gps.altitude_m))
		if gps.speed_knots is not None:
			print("Speed: {} knots".format(gps.speed_knots))
		if gps.track_angle_deg is not None:
			print("Track angle: {} degrees".format(gps.track_angle_deg))
		if gps.horizontal_dilution is not None:
			print("Horizontal dilution: {}".format(gps.horizontal_dilution))
		if gps.height_geoid is not None:
			print("Height geo ID: {} meters".format(gps.height_geoid))
		i+=1

		if gps.latitude is not None and gps.latitude != 0:
			gpsfix = NavSatFix()
			gpsfix.header.stamp = rospy.Time.now()
			gpsfix.header.frame_id = 'gps_frame'
			gpsfix.latitude = gps.latitude
			gpsfix.longitude = gps.longitude
			gpsfix.altitude = gps.altitude_m
			gpsfix.position_covariance_type = gps.fix_quality
			gps_pub.publish(gpsfix)
