#!/usr/bin/env python

import ctypes
import uhal
import random


def hook_debugger(debugger='gdb'):
    '''debugging helper, hooks debugger to running interpreter process
    '''
    print('Starting debugger')
    import os
    pid = os.spawnvp(os.P_NOWAIT, debugger, [debugger, '-q', 'python', str(os.getpid())])

uhal.setLogLevelTo(uhal.LogLevel.DEBUG)

ctypes.cdll.LoadLibrary('libuhallibs.so')

hook_debugger()

cm = uhal.ConnectionManager('file://${UHALLIBS_SHARE}/config/c.xml', ['ipbusaxi4lite-2.0', 'ipbusflx-2.0'])

u50 = cm.getDevice('u50-axi4lite-xdma')


for n in u50.getNodes():
    print('- ', n)

reg = u50.getNode('reg').read()
u50.dispatch()
print(f'"reg" current value {hex(reg)}')

print("Generating random number")
x = random.getrandbits(32)
print(f"New random value: {hex(x)}")


print("Writing 'reg'")
u50.getNode('reg').write(x)
u50.dispatch()

print("Reading back 'reg'")
reg = u50.getNode('reg').read()
u50.dispatch()
print(f"'reg' current value {hex(reg)}")
