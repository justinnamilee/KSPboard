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

def readControlByte():
	return int(control.readline().strip())

def readControlFloat():
	x = readControlByte()
	print(x)
	return (2.0 * (float(x) - 127.5) / 255.0)

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
def kspOps(n):
	KSP_ops[n]()

# for directional input
KSP_helm = {
	'pitch'  : 0,
	'roll'   : 0,
	'yaw'    : 0,
	'thrust' : 50,
}

def kspHelm():
	'null'


# loop

print('')
print('ready')

while True:

	test = readControlFloat()
	bridge.pitch = test
	print(test)	
	time.sleep(0.1)

#	if readControlByte() == 1:
#		print('command')
#		kspControl(readControlByte())

