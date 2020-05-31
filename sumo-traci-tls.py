#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2020 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

# Modified
# @file    sumo-traci-tls.py
# @auther  Ahmed Hendawy - Yonohub Team
# @date    21.05.2020

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import time

os.environ['SUMO_HOME']='/usr/share/sumo'

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

# YonoArc Utils
from yonoarc_utils.header import set_timestamp

# Messages
from yonoarc_msgs.msg import Float64
from geometry_msgs.msg import Point
from std_msgs.msg import Header


def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 30
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)




class sumo:
    def on_start(self):
        os.system('(rm /tmp/.X1-lock; Xvfb :1 -screen 0 1280x720x16 & while sleep 2; do x11vnc -display :1 -noxrecord -xkb; done & DISPLAY=":1" startxfce4& cd /noVNC && ./utils/launch.sh --vnc localhost:5900) &')
        generate_routefile()
        sumoBinary = checkBinary('sumo-gui')
        traci.start(['env', 'DISPLAY=:1',sumoBinary, "-c", "data/cross.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
        
       
        
    def run(self):
        self.run_scenario()

    def run_scenario(self):
        """execute the TraCI control loop"""
        step = 0
        vId=""
        # we start with phase 2 where EW has green
        traci.trafficlight.setPhase("0", 2)
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            vIds=traci.vehicle.getIDList()
            if step == 0:
                vId=vIds[0]
            if vId in vIds:
                header=Header()
                set_timestamp(header,time.time())
                position=traci.vehicle.getPosition(vId)
                speed=traci.vehicle.getSpeed(vId)
                acceleration=traci.vehicle.getAcceleration(vId)
                lateral_speed=traci.vehicle.getLateralSpeed(vId)
                waiting_time=traci.vehicle.getWaitingTime(vId)
                pos=Point()
                pos.x,pos.y,pos.z=position[0],position[1],0.0
                vel,acc,lvel,wt=Float64(),Float64(),Float64(),Float64()
                vel.header,vel.data=header,speed
                acc.header,acc.data=header,acceleration
                lvel.header,lvel.data=header,lateral_speed
                wt.header,wt.data=header,waiting_time
                self.publish("pos",pos)
                self.publish("vel",vel)
                self.publish("acc",acc)
                self.publish("lateral_speed",lvel)
                self.publish("waiting_time",wt)
            if traci.trafficlight.getPhase("0") == 2:
                # we are not already switching
                if traci.inductionloop.getLastStepVehicleNumber("0") > 0:
                    # there is a vehicle from the north, switch
                    traci.trafficlight.setPhase("0", 3)
                else:
                    # otherwise try to keep green for EW
                    traci.trafficlight.setPhase("0", 2)
            step += 1
        traci.close()
        sys.stdout.flush()