# -*- coding: utf-8 -*-
import serial
# This script open a connection with 2 robots and exchanged data with them using the "advanced sercom protocol".
# Tested with Python 3.4
# Requirements: PySerial module.

import time
import struct

INIT = "55"
DEBUG = "1111"
END = "5555"
# MOVE_FORWARD = "1001"
MESSAGE_INIT_LEN = 2
MESSAGE_TYPE_LEN = 4
MESSAGE_LENGTH_LEN = 4
byte_type = "c"
byte_amount_in_type = 1


def read_n_bytes(n, robot):
	""" should return char string """
	reply = ""
	while len(reply) != n:
		reply += robot.read()
	#     need to test reply = robot.read(n)
	# print("unpack int: ", struct.unpack("@" + "i"*len(reply), reply))
	return reply


def read_messages(robot):
	""" return message string """
	reply = byte_string_to_number(read_n_bytes(MESSAGE_LENGTH_LEN, robot))
	message = read_n_bytes(reply, robot)

	# todo add check_byte?

	return message


def debug_robot(robot, id):
	reply = read_messages(robot)
	print("DEBUG ROBOT ", id, " :", reply)


def debug_coord(robot):
	length = byte_string_to_number(read_n_bytes(MESSAGE_LENGTH_LEN, robot))

	x = ""
	while len(x) < length/2:
		x += robot.read()

	y = ""
	while len(y) < length/2:
		y += robot.read()

	print("DEBUG x, y:", x, y)


def byte_string_to_number(byte_reply):
	ans = 0
	for i in byte_reply:
		ans *= 10
		ans += int(i)
	return ans


try:
	robot1 = serial.Serial('/dev/cu.e-puck2_04214-UART', 115200, timeout=0)
	# Specify the robot communication port (after pairing).
	print("successfully connected to robot")
finally:
	print()


print('Введите X в формате: #### (см)')
x = raw_input()

print('Введите Y в формате: #### (см)')
y = raw_input()

message = struct.pack("<b" + "b" + "4s" + "4s" + "b", -ord('X'), 7, x, y, 0)
robot1.write(message)

reply = ""
loop = True
itt = 0
while loop:
	while reply != INIT:
		reply = read_n_bytes(MESSAGE_INIT_LEN, robot1)

	reply = read_n_bytes(MESSAGE_TYPE_LEN, robot1)
	if reply == DEBUG:
		debug_coord(robot1)
	elif reply == END:
		loop = False

robot1.close()
