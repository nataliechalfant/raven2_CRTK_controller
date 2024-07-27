"""
Raven II Dual Platform Controller: control software for the Raven II robot. Copyright © 2023-2024 Yun-Hsuan Su,
Natalie Chalfant, Mai Bui, Sean Fabrega, and the Mount Holyoke Intelligent Medical Robotics Laboratory.

This file is a part of Raven II Dual Platform Controller.

Raven II Dual Platform Controller is free software: you can redistribute it and/or modify it under the terms of the
GNU Lesser General Public License as published by the Free Software Foundation, either version 3 of the License,
or (at your option) any later version.

Raven II Dual Platform Controller is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with Raven II Dual Platform Controller.
If not, see <http://www.gnu.org/licenses/>.

ambf_xbox_controller.py

date: May 13, 2024
author: Natalie Chalfant, Mai Bui, Sean Fabrega
"""

import time

import inputs
from inputs import get_gamepad
from inputs import devices
import math
import threading


class xbox_inputs(object):
    """
    Author: Natalie Chalfant
    Gets the state of xbox controller inputs using the inputs library. Modified from
    https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python
    """
    MAX_TRIG_VAL = math.pow(2, 10)
    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self):
        self.gamepad = devices.gamepads[0]
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self.Back_returned = False
        self.Start_returned = False

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def get_lj_x(self):
        return self.LeftJoystickX

    def get_lj_y(self):
        return self.LeftJoystickY

    def get_rj_x(self):
        return self.RightJoystickX

    def get_rj_y(self):
        return self.RightJoystickY

    def get_lt(self):
        return self.LeftTrigger

    def get_rt(self):
        return self.RightTrigger

    def get_lb(self):
        return self.LeftBumper

    def get_rb(self):
        return self.RightBumper

    def get_back(self):
        # only return true once for each press
        if self.Back:
            if self.Back_returned:
                return 0
            else:
                self.Back_returned = True
                return 1
        else:
            self.Back_returned = False
            return 0

    def get_start(self):
        # only return true once for each press
        if self.Start:
            if self.Start_returned:
                return 0
            else:
                self.Start_returned = True
                return 1
        else:
            self.Start_returned = False
            return 0

    def read(self): # return the buttons/triggers that you care about in this methode
        lx = self.LeftJoystickX
        ly = self.LeftJoystickY
        lt = self.LeftTrigger
        lb = self.LeftBumper

        rx = self.RightJoystickX
        ry = self.RightJoystickY
        rt = self.RightTrigger
        rb = self.RightBumper
        return [[lx, ly, lt, lb], [rx, ry, rt, rb],
                [self.A, self.B, self.X, self.Y, self.Back, self.Start]]

    def rumble(self, left, right, time):
        try:
            self.gamepad.set_vibration(left, right, time)
        except OSError:
            print("no space left on device")

    def _monitor_controller(self):
        while True:
            try:
                events = get_gamepad()
            except inputs.UnknownEventCode:
                print("unknown event")
                events = None

            if events is not None:
                for event in events:
                    if event.code == 'ABS_Y':
                        self.LeftJoystickY = event.state / xbox_inputs.MAX_JOY_VAL # normalize between -1 and 1
                    elif event.code == 'ABS_X':
                        self.LeftJoystickX = event.state / xbox_inputs.MAX_JOY_VAL # normalize between -1 and 1
                    elif event.code == 'ABS_RY':
                        self.RightJoystickY = event.state / xbox_inputs.MAX_JOY_VAL # normalize between -1 and 1
                    elif event.code == 'ABS_RX':
                        self.RightJoystickX = event.state / xbox_inputs.MAX_JOY_VAL # normalize between -1 and 1
                    elif event.code == 'ABS_Z':
                        self.LeftTrigger = event.state / xbox_inputs.MAX_TRIG_VAL # normalize between 0 and 1
                    elif event.code == 'ABS_RZ':
                        self.RightTrigger = event.state / xbox_inputs.MAX_TRIG_VAL # normalize between 0 and 1
                    elif event.code == 'BTN_TL':
                        self.LeftBumper = event.state
                    elif event.code == 'BTN_TR':
                        self.RightBumper = event.state
                    elif event.code == 'BTN_SOUTH':
                        self.A = event.state
                    elif event.code == 'BTN_NORTH':
                        self.X = event.state  # previously switched with X
                    elif event.code == 'BTN_WEST':
                        self.Y = event.state  # previously switched with Y
                    elif event.code == 'BTN_EAST':
                        self.B = event.state
                    elif event.code == 'BTN_THUMBL':
                        self.LeftThumb = event.state
                    elif event.code == 'BTN_THUMBR':
                        self.RightThumb = event.state
                    elif event.code == 'BTN_SELECT':
                        self.Back = event.state
                    elif event.code == 'BTN_START':
                        self.Start = event.state
                    elif event.code == 'BTN_TRIGGER_HAPPY1':
                        self.LeftDPad = event.state
                    elif event.code == 'BTN_TRIGGER_HAPPY2':
                        self.RightDPad = event.state
                    elif event.code == 'BTN_TRIGGER_HAPPY3':
                        self.UpDPad = event.state
                    elif event.code == 'BTN_TRIGGER_HAPPY4':
                        self.DownDPad = event.state


if __name__ == '__main__':
    joy = xbox_inputs()
    # joy.rumble(1, 1, 100)

    while True:
        # print(joy.read())
        print(joy.get_back(), joy.get_start())
        time.sleep(1)
