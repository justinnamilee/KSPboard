#### Justin Lee <justin at taiz dot me>
# 
# kRPC Serial Front-End // v0.1 alpha
# Copyright: Justin Lee 2017-1018
# License: GNU GPLv3 or above
#
###
#
# Description:
#
#   Allows the user to control Kerbal over serial
#   using kRPC as the middle man.  This offers
#   greater flexibility in control when compared
#   with KSP Serial as you do not need to recompile
#   a mod to make changes to the serial interface.
#   See LISENCE and README for more information on
#   this project.
#
###


debug = False
print('debug: {}'.format(debug))

print('import...')

import krpc # control ksp
import time # for waiting and stuff
import serial # talk to other devices

print('starting up...')


# setup serial

print('serial setup...')
control = serial.Serial('COM6', 115200)
time.sleep(1)

def haveLine():
	return control.in_waiting > 0

def readInt(): # waits for a newline
	return int(control.readline().strip())

def readIntRaw(): # 0 to 255
	return int(control.read())

def readDirection(): # -1 to 1
	return (float(readInt()) / 2000.0) # -2000 to 2000

def readThottle(): # 0 to 1
	return (float(readInt()) / 1023.0) # analog read max

control.readline() # chomp and garbage off the front of the stream


# setup kRPC

print('kRPC setup...')
ksp = krpc.connect(name='KSPboard')
craft = ksp.space_center.active_vessel # should be in a try/catch
bridge = craft.control
kspControlLock = False

print('craft: {}'.format(craft.name))


# setup available KSP functions

print('dictionary setup...')

def KSP_act(n):
	def KSP_act_call():
		debug and print('action: {}'.format(n))
		bridge.toggle_action_group(n)
	return KSP_act_call

def KSP_delay(n):
	debug and print('delay: {}'.format(n))
	time.sleep(n)

def KSP_null():
	debug and print('null')

def KSP_stage():
	debug and print('stage')
	bridge.activate_next_stage()

def KSP_gear():
	debug and print('gear')
	oldState = bridge.gear
	bridge.gear = not oldState 

def KSP_light():
	debug and print('light')
	oldState = bridge.lights
	bridge.lights = not oldState

def KSP_rcs():
	debug and print('rcs')
	oldState = bridge.rcs
	bridge.rcs = not oldState

def KSP_brakes():
	debug and print('brakes')
	oldState = bridge.brakes
	bridge.brakes = not oldState

def KSP_abort():
	debug and print('abort')
	oldState = bridge.abort
	bridge.abort


# control dictionary (look up table / switch)
KSP_ops = {
	0  : KSP_null,
	1  : KSP_act(1),
	2  : KSP_act(2),
	3  : KSP_act(3),
	4  : KSP_act(4),
	5  : KSP_act(5),
	6  : KSP_act(6),
	7  : KSP_act(7),
	8  : KSP_act(8),
	9  : KSP_act(9),
	10 : KSP_act(10),
	11 : KSP_stage,
	12 : KSP_gear,
	13 : KSP_light,
	14 : KSP_rcs,
	15 : KSP_brakes,
	16 : KSP_abort,
}

# operations dictionary mapper
def kspOps(c):
	debug and print('command: {}'.format(c))
	KSP_ops[c]()

def kspOpsUpdate():

	if readInt() == 1:
		command = []
		state = readInt()

		while state != 0:
			command.append(state) # get command list
			state = readInt() # get next command

		for c in command:
			kspOps(c) # execute in order



# for directional input

KSP_helm = {
	'pitch'    : 0,
	'roll'     : 0,
	'yaw'      : 0,
	'throttle' : 0,
}

def kspHelm(e, v=None):

	if v is not None:
		debug and print('helm: update {}: {}'.format(e, v))
		KSP_helm[e] = v
	else:
		debug and print('helm: current {}: {}'.format(e, KSP_helm[e]))

	return KSP_helm[e]


def kspHelmUpdate():

	pitch = readDirection()
	yaw = readDirection()
	roll = readDirection()
	throttle = readThottle()

	if pitch != kspHelm('pitch'):
		kspHelm('pitch', pitch)
		bridge.pitch = pitch

	if yaw != kspHelm('yaw'):
		kspHelm('yaw', yaw)
		bridge.yaw = yaw

	if roll != kspHelm('roll'):
		kspHelm('roll', roll)
		bridge.roll = roll

	if throttle != kspHelm('throttle'):
		kspHelm('throttle', throttle)
		bridge.throttle = throttle


# loop

print('')
print('ready')
print('')

# wait for the startup command
while True:

	if readIntRaw() == 1:
		if readIntRaw() == 1:
			if readIntRaw() == 1:
				print('armed')
				break

# begin parsing
while True:

	# status check
	if readInt() == 0:
		print('')
		print('exit')
		break

	kspHelmUpdate() # updates control surfaces and throttle
	kspOpsUpdate() # action groups, staging, etc
