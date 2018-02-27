#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2018 Universidad de Costa Rica, Autonomous Robots
# and Cognitive Systems Laboratory (ARCOS-lab)
# Author: Daniel Garcia-Vaglio <degv364@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
"""This creates a scene logger.

Opens the same ports required for visualization
but it logs the info instead of drawing it.
"""
from __future__ import print_function

from builtins import zip
from builtins import str
from builtins import map
from builtins import range
import time
import sys
import signal
import yarp
import optparse
import pickle
from numpy import array

from arcospyu.control.control_loop import Controlloop

sys.path.append("../hand_cart")
sys.path.append("../config_data/lwr")


def configParse(argv):
    optparse.SUPPRESS_HELP
    parser = optparse.OptionParser(
        "usage: %prog [options]", add_help_option=False)
    parser.add_option("-h", "--help", action="help")
    parser.add_option(
        "-r",
        "--robot",
        dest="robot",
        default="lwr",
        type="string",
        help="".join([
            "Tell the robot name so that the correct Articulated",
            "Tree is loaded"
        ]))
    parser.add_option(
        "--arm_left",
        action="store_true",
        dest="arm_left",
        default=False,
        help="Enable the configuration and simulation of a left arm")
    parser.add_option(
        "--arm_right",
        action="store_true",
        dest="arm_right",
        default=False,
        help="Enable the configuration and simulation or a right arm")
    parser.add_option(
        "--hand_left",
        action="store_true",
        dest="hand_left",
        default=False,
        help="Enable the configuration and simulation of a left hand")
    parser.add_option(
        "--hand_right",
        action="store_true",
        dest="hand_right",
        default=False,
        help="Enable the configuration and simulation of a right hand")
    parser.add_option(
        "-a",
        "--config_dir_arms",
        dest="config_dir_arms",
        default="robot_descriptions/tum-rosie/kinematics/lwr/",
        type="string",
        help="Arms configuration directory")
    parser.add_option(
        "-d",
        "--config_dir_hands",
        dest="config_dir_hands",
        default="robot_descriptions/tum-rosie/kinematics/sahand/",
        type="string",
        help="Hands configuration directory")
    (options, args) = parser.parse_args(argv[1:])
    return (options, args)


class VisSinkLoop(Controlloop):
    def set_up(self):
        self.current_time_for_log = 0

        self.log = {}

        self.options, args = configParse(sys.argv)

        self.add_log({"id": "light0", "type": "Light", "pos": [10, 10, 10, 1]})
        self.add_log({
            "id": "light1",
            "type": "Light",
            "pos": [-10, 10, 10, 1]
        })
        self.add_log({"id": "light2", "type": "Light", "pos": [0, -10, 10, 1]})
        self.add_log({"id": "light3", "type": "Light", "pos": [0, 0, -10, 1]})

        self.add_log({
            "id": "camera_center",
            "type": "Disk",
            "color": [0.5, 0.5, 0.5],
            "color_reflex": ([1, 1, 1], 50),
            "visibility": False
        })

        self.add_log({
            "id": "world_frame",
            "type": "Frame",
            "scale": [0.3, 0.3, 0.3]
        })

        # All required info to get the robot descriptions (unpickable)
        kin_desc = [
            self.options, self.options.config_dir_arms,
            self.options.config_dir_hands
        ]

        self.add_log({
            "id": "arm_right",
            "type": "Articulated",
            "kinematic_description": kin_desc
        })

        self.add_log({
            "id": "arm_left",
            "type": "Articulated",
            "kinematic_description": kin_desc
        })

        self.add_log({
            "id": "hand_right",
            "type": "Articulated",
            "kinematic_description": kin_desc
        })

        self.add_log({
            "id": "hand_left",
            "type": "Articulated",
            "kinematic_description": kin_desc
        })

        self.add_log({
            "id":
            "tree",
            "type":
            "Articulated_tree",
            "children": ["arm_right", "arm_left", "hand_right", "hand_left"]
        })

        yarpbaseportname = "/" + self.options.robot + "/roboviewer"
        yarp.Network.init()
        cstyle = yarp.ContactStyle()
        cstyle.persistent = True

        if self.options.arm_right:
            self.rarm_qin_port = yarp.BufferedPortBottle()
            self.rarm_qin_port.open(yarpbaseportname + "/r_arm_qin")
            yarp.Network.connect("".join([
                "/", self.options.robot, "/right/bridge/encoders"
            ]), "".join([yarpbaseportname, "/r_arm_qin"]), cstyle)
        if self.options.arm_left:
            self.larm_qin_port = yarp.BufferedPortBottle()
            self.larm_qin_port.open("".join([yarpbaseportname, "/l_arm_qin"]))
            yarp.Network.connect("".join([
                "/", self.options.robot, "/left/bridge/encoders"
            ]), "".join([yarpbaseportname, "/l_arm_qin"]), cstyle)

        # objects port
        self.objects_port = yarp.BufferedPortBottle()
        self.objects_port.open("".join([yarpbaseportname, "/objects:i"]))
        self.objects_port.setStrict()
        self.objects_dict = {}

        if self.options.hand_right:
            self.right_hand_port = yarp.BufferedPortBottle()
            self.right_hand_port.open("/roboviewer_right_hand/hand_client0")
            self.right_hand_port_ctrl = yarp.BufferedPortBottle()
            self.right_hand_port_ctrl.open(
                "/roboviewer_right_hand/hand_client0/ctrl")

            yarp.Network.connect("/sahand0/out",
                                 "/roboviewer_right_hand/hand_client0")
            yarp.Network.connect("/roboviewer_right_hand/hand_client0",
                                 "/sahand1/in")
            yarp.Network.connect("/roboviewer_right_hand/hand_client0/ctrl",
                                 "/sahand/cmd")

        if self.options.hand_left:
            self.left_hand_port = yarp.BufferedPortBottle()
            self.left_hand_port.open("/roboviewer_left_hand/hand_client1")
            self.left_hand_port_ctrl = yarp.BufferedPortBottle()
            self.left_hand_port_ctrl.open(
                "/roboviewer_left_hand/hand_client1/ctrl")

            yarp.Network.connect("/sahand1/out",
                                 "/roboviewer_left_hand/hand_client1")
            yarp.Network.connect("/roboviewer_left_hand/hand_client1",
                                 "/sahand1/in")
            yarp.Network.connect("/roboviewer_left_hand/hand_client1/ctrl",
                                 "/sahand/cmd")

    def set_params(self, params):
        pass

    def process(self):
        self.current_time_for_log = time.time()
        if self.options.arm_right:
            rarm_qin_bottle = self.rarm_qin_port.read(False)
            if rarm_qin_bottle:
                rarm_qin = list(
                    map(yarp.Value.asDouble,
                        list(
                            map(rarm_qin_bottle.get,
                                list(range(rarm_qin_bottle.size()))))))
                self.add_log({"id": "arm_right", "set_angles": rarm_qin})
        if self.options.arm_left:
            larm_qin_bottle = self.larm_qin_port.read(False)
            if larm_qin_bottle:
                larm_qin = list(
                    map(yarp.Value.asDouble,
                        list(
                            map(larm_qin_bottle.get,
                                list(range(larm_qin_bottle.size()))))))
                self.add_log({"id": "arm_right", "set_angles": larm_qin})
        if self.options.hand_right:
            self.get_hand_info(self.right_hand_port, "right")

        if self.options.hand_left:
            self.get_hand_info(self.left_hand_port, "left")

        # objects port processing
        while self.objects_port.getPendingReads() != 0:
            objects_bottle = self.objects_port.read(True)
            if objects_bottle:
                objects_bottles = list(
                    map(yarp.Value.asList,
                        list(
                            map(objects_bottle.get,
                                list(range(objects_bottle.size()))))))
                for object_bottle in objects_bottles:
                    object_id = object_bottle.get(0).asInt()
                    object_prop_name = object_bottle.get(1).toString()
                    if object_prop_name == "type":
                        object_prop_data = object_bottle.get(2).toString()
                    else:
                        object_prop_data = array(
                            list(
                                map(yarp.Value.asDouble,
                                    list(
                                        map(object_bottle.get,
                                            list(range(
                                                object_bottle.size()))[2:])))))
                    # Properties are: type (sphere, frame, arrow, box), pose
                    # (16 values), color (three values), scale (three values),
                    # etc
                    self.add_log({
                        "id": object_id,
                        "object_prop_name": object_prop_name,
                        "object_prop_data": object_prop_data
                    })

    def get_hand_info(self, hand_port, side_str):
        angles_field = list(range(3))
        speeds_field = list(range(3, 6))
        torques_field = list(range(6, 9))
        THUMB_ANGLE_FIELD = 1
        THUMB = 0
        FIRST_FINGER = 1
        MIDDLE_FINGER = 2
        RING_FINGER = 3
        PINKY_FINGER = 4
        all_finger_list = [
            THUMB, FIRST_FINGER, MIDDLE_FINGER, RING_FINGER, PINKY_FINGER
        ]

        bottle = hand_port.read()
        # reading data from server (all fingers always)
        for fingerbottle, finger in zip(
                list(
                    map(yarp.Value.asList,
                        list(map(bottle.get, list(range(2, bottle.size())))))),
                all_finger_list):
            if finger == THUMB:
                thumb_angle = bottle.get(
                    THUMB_ANGLE_FIELD).asDouble()  # for thumb position
                thumb_speed = 0.0  # TODO: Calculate thumb speed
                self.add_log({
                    "id": "".join(["Hand", side_str]),
                    "finger": finger,
                    "angles": ([thumb_angle] + list(
                        map(yarp.Value.asDouble,
                            list(map(fingerbottle.get, angles_field))))),
                    "speeds": ([thumb_speed] + list(
                        map(yarp.Value.asDouble,
                            list(map(fingerbottle.get, speeds_field))))),
                    "torques":
                    array(
                        list(
                            map(yarp.Value.asDouble,
                                list(map(fingerbottle.get, torques_field)))))
                })
            else:
                self.add_log({
                    "id": "".join(["Hand", side_str]),
                    "finger": finger,
                    "angles":
                    list(
                        map(yarp.Value.asDouble,
                            list(map(fingerbottle.get, angles_field)))),
                    "speeds":
                    list(
                        map(yarp.Value.asDouble,
                            list(map(fingerbottle.get, speeds_field)))),
                    "torques":
                    array(
                        list(
                            map(yarp.Value.asDouble,
                                list(map(fingerbottle.get, torques_field)))))
                })

    def add_log(self, elements):
        # FIXME: We should use a "real" logging solution
        # But that stores python objects, not just text.
        self.log[self.current_time_for_log] = elements

    def terminate_handler(self, signum, stack_frame):
        print(("Catched signal", signum))
        log_file = open("".join(["log_file_", str(time.time())]), "w")
        pickle.dump(self.log, log_file)
        log_file.close()
        yarp.Network.fini()
        sys.exit()


loop = VisSinkLoop(15.)
signal.signal(signal.SIGTERM, loop.terminate_handler)
signal.signal(signal.SIGINT, loop.terminate_handler)
loop.set_up()
loop.loop()
loop.store()
