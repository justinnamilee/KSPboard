#### Justin Lee <justin at kitten dot pink>
# 
# kRPC Serial Front-End // v0.1 alpha
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


print('import...')

import krpc # control ksp
import time # for waiting and stuff
import serial # talk to other devices

print('starting up...')


# setup serial

print('serial setup...')
control = serial.Serial('COM6', 115200)
time.sleep(1)

def haveByte():
	return control.in_waiting > 0

def readByteB(): # blocking
	return int(control.readline().strip())

def readByte(): # 0 to 255
	return int(control.read())

def readFloat(): # -1 to 1
	return (2.0 * (float(readByte()) - 127.5) / 255.0)

def readUFloat(): # 0 to 1
	return (readFloat() / 2.0) + 0.5

control.readline()


# setup kRPC

print('kRPC setup...')
ksp = krpc.connect(name='KSPboard')
craft = ksp.space_center.active_vessel
bridge = craft.control
kspControlLock = False

print('craft: {}'.format(craft.name))


# setup available KSP functions

print('dictionary setup...')

def KSP_launch(): # this fires main engine, waits 1 sec, then stages again
	print('launch')
	KSP_stage()
	KSP_delay(1)
	KSP_stage()

def KSP_act(n):
	def KSP_act_call():
		print('action: {}'.format(n))
		bridge.toggle_action_group(n)
	return KSP_act_call

def KSP_delay(n):
	print('delay: {}'.format(n))
	time.sleep(n)

def KSP_stage():
	print('stage')
	bridge.activate_next_stage()

def KSP_null():
	print('null')

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
	11 : KSP_launch,
	12 : KSP_stage,
# 
}

# operations dictionary mapper
def kspOps(c):
	print('command: {}'.format(c))
	KSP_ops[c]()

def kspOpsUpdate():

	if readByte() == 1:
		command = []
		state = readByte()

		while state != 0:
			command.append(state) # get command list
			state = readByte() # get next command

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
		print('helm: update {}: {}'.format(e, v))
		KSP_helm[e] = v
	else:
		print('helm: current {}: {}'.format(e, KSP_helm[e]))

	return KSP_helm[e]


def kspHelmUpdate():

	pitch = readFloat()
	yaw = readFloat()
	roll = readFloat()
	throttle = readUFloat()

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

	if readByte() == 1:
		if readByte() == 1:
			if readByte() == 1:
				print('armed')
				break

# begin parsing
while True:

	if readByte() == 0:
		print('')
		print('exit')
		break

	kspHelmUpdate() # updates control surfaces and throttle
	kspOpsUpdate() # action groups, staging, etc
