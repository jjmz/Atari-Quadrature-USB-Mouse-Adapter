#!/usr/bin/env python3
'''
Python script to report on joysticks

Requires: pygame

Copyright (C) Simon D. Levy 2018

This file is part of PySticks.

PySticks is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.
This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this PySticks.  If not, see <http:#www.gnu.org/licenses/>.
'''

from sys import stdout

import pygame

# Initialize pygame for joystick support
pygame.display.init()
pygame.joystick.init()
controller = pygame.joystick.Joystick(0)
controller.init()

while True:

    # Get next pygame event
    pygame.event.pump()

    stdout.write('%s | Axes: ' % controller.get_name())

    for k in range(controller.get_numaxes()):
        stdout.write('%d:%+2.2f ' % (k, controller.get_axis(k)))
    stdout.write(' | Buttons: ')
    for k in range(controller.get_numbuttons()):
        stdout.write('%d:%d ' % (k, controller.get_button(k)))
    stdout.write('\n')
