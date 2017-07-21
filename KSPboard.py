#### Justin Lee <justin at kitten dot pink>
# 
# kRPC Serial Front-End // v0.1 alpha
#
###
#
# License: GNU GPL v3 or higher.
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
	x = int(control.read())
	print(x)
	return x


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
KSP_control = {
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

# control dictionary mapping function
def kspControl(n):
	KSP_control[n]()


# loop

print('')
print('ready')

while True:
	if readControlByte() == 0:
		print('command')
		kspControl(readControlByte())

