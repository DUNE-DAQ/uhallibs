#!/usr/bin/env python

import ctypes
import uhal
import random
import click


@click.command()
@click.option('-d', '--device', default='flx-0-ipb', help='Felix device.')
@click.option('-w', '--writes', default=1, help='Number of writes')
@click.option('-r', '--reads', default=1, help='Number of reads')
@click.option('-c', '--cycles', default=1, help='Number of cycles')
def main(device, writes, reads, cycles):
    """Simple program that greets NAME for a total of COUNT times."""
    for x in range(count):
        click.echo(f"Hello {name}!")

    uhal.setLogLevelTo(uhal.LogLevel.DEBUG)

    ctypes.cdll.LoadLibrary('libwupper-toybox.so')

    cm = uhal.ConnectionManager('file://${WUPPER_TOYBOX_SHARE}/config/c.xml', ['ipbusflx-2.0'])

    flx_dev = cm.getDevice(device)

    for n in flx_dev.getNodes():
        print('- ', n)

    reg = flx_dev.getNode('reg').read()
    flx_dev.dispatch()
    print(f'"reg" current value {hex(reg)}')

    for c in range(cycles):
        for w in range(writes):
            print(f"{c} - Generating random number")
            x = random.getrandbits(32)
            print(f"New random value: {hex(x)}")


            print(f"{c}:{w} - Writing 'reg'")
            flx_dev.getNode('reg').write(x)
            flx_dev.dispatch()

        for r in range(reads):
            print(f"{c}:{r} - Reading back 'reg'")
            reg = flx_dev.getNode('reg').read()
            flx_dev.dispatch()
            print(f"'reg' current value {hex(reg)}")



if __name__ == '__main__':
    main()
