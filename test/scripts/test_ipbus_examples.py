#!/usr/bin/env python

import ctypes
import uhal
import random

uhal.setLogLevelTo(uhal.LogLevel.DEBUG)

ctypes.cdll.LoadLibrary('libwupper-toybox.so')

cm = uhal.ConnectionManager('file://${WUPPER_TOYBOX_SHARE}/config/c.xml', ['ipbusflx-2.0'])

flx_dev = cm.getDevice('flx-2-ipb')

for n in flx_dev.getNodes():
    print('- ', n)

reg = flx_dev.getNode('reg').read()
flx_dev.dispatch()
print(f'"reg" current value {hex(reg)}')

print("Generating random number")
x = random.getrandbits(32)
print(f"New random value: {hex(x)}")


print("Writing 'reg'")
flx_dev.getNode('reg').write(x)
flx_dev.dispatch()

print("Reading back 'reg'")
reg = flx_dev.getNode('reg').read()
flx_dev.dispatch()
print(f"'reg' current value {hex(reg)}")
