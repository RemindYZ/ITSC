from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import random
import sys
import numpy as np

# check up the environment, import python modules from $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME' after installing sumo!")

# import python modules from sumo
from sumolib import checkBinary
import traci
import traci.constants as tc

class SumoSim():
    def __init__(self, cfg_file, net_file, router=None, nogui=False):
        self.cfg_file = cfg_file
        self.net_file = net_file
        self.router = router
        if not self.router:
            self.generate_route_file()
    
    def generate_route_file(self):
        pass