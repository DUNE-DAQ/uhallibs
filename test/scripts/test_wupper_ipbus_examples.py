#!/usr/bin/env python

import ctypes
import uhal
import random


def hook_debugger(debugger='gdb'):
    """debugging helper, hooks debugger to running interpreter process
    """
    print('Starting debugger')
    import os
    pid = os.spawnvp(os.P_NOWAIT, debugger, [debugger, '-q', 'python', str(os.getpid())])

uhal.setLogLevelTo(uhal.LogLevel.DEBUG)

ctypes.cdll.LoadLibrary('libwupper-toybox.so')
cm = uhal.ConnectionManager('file://${WUPPER_TOYBOX_SHARE}/config/c.xml', ['ipbusflx-2.0', 'ipbusflx-2.0'])

flx0 = cm.getDevice('flx-0-ipb')

for n in flx0.getNodes():
    print('- ', n)

reg = flx0.getNode('reg').read()
flx0.dispatch()
print(f'"reg" current value {hex(reg)}')

print("Generating random number")
x = random.getrandbits(32)
print(f"New random value: {hex(x)}")


print("Writing 'reg'")
flx0.getNode('reg').write(x)
flx0.dispatch()

print("Reading back 'reg'")
reg = flx0.getNode('reg').read()
flx0.dispatch()
print(f"'reg' current value {hex(reg)}")
