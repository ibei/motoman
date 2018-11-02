#!/usr/bin/env python
#coding: utf-8
from evdev import InputDevice
from select import select

def detectInputKey():
    dev = InputDevice('/dev/input/event4')
    while True:
        select([dev], [], [])
        for event in dev.read():
            print "code:%s value:%s" % (event.code, event.value)


if __name__ == '__main__':
    detectInputKey()
