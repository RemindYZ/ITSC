from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import random
import sys
import numpy as np
import matplotlib.pyplot as plt

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
    def __init__(self, cfg_file, route_file, nogui=False, log_out="log"):
        self.cfg_file = cfg_file
        self.route_file = route_file
        self.generate_route_file()

        if nogui:
            self.sumoBinary = checkBinary('sumo')
            self.cmd_lines = [self.sumoBinary, "--start", "-c", self.cfg_file,"--tripinfo-output", "tripinfo.xml"]
            self.load_cmd = ["--start", "-c", self.cfg_file]
        else:
            self.sumoBinary = checkBinary('sumo-gui')
            # --start, --quit-on-end
            self.cmd_lines = [self.sumoBinary, "-c", self.cfg_file,"--tripinfo-output", "tripinfo.xml"]
            self.load_cmd = ["-c", self.cfg_file]
        
        self.log_out = log_out
        if not os.path.exists(log_out):
            os.makedirs(log_out)

        self.phases_file = os.path.join(log_out, 'phases.npy')
        self.queue_file = os.path.join(log_out, 'queue.npy')

        self.detect_length = 200
        self.road_length = 250
        self.n_road = 4
        self.n_lane = 3
        # self.margin = 14
        self.max_green_time = [20,15,35,20]
        self.min_green_time = 3
        self.alpha = [0.4,0.3,0.2,0.1]
        self.phase2road_lane_index = {0:[[3,4],[0,1]],2:[[3,4],[2]],4:[[1,2],[0,1]],6:[[1,2],[2]]}
        self.interval_time = 6
        # self.max_sim_time = 360000000
        
        traci.start(self.cmd_lines)
        # context subscriptions, subscribe vehicles near the intersection
        traci.junction.subscribeContext("0", tc.CMD_GET_VEHICLE_VARIABLE, self.detect_length,
                                        [tc.VAR_LANE_ID, tc.VAR_ROAD_ID,
                                         tc.VAR_WAITING_TIME, tc.VAR_LANEPOSITION,
                                         tc.VAR_SPEED, tc.VAR_ACCELERATION])
    
    def generate_route_file(self, pns=1.0/8.0, pew=1.0/4.0, psn=1.0/8.0, pwe=1.0/4.0, plt=1.0/4.0, prt=1.0/4.0):
        self.seed(40)
        num_time_step = 3600  # number of time steps
        routes = [{'route': 'n2s', 'prob': pns},
                  {'route': 'n2e', 'prob': pns*plt},
                  {'route': 'n2w', 'prob': pns*prt},
                  {'route': 'e2w', 'prob': pew},
                  {'route': 'e2s', 'prob': pew*plt},
                  {'route': 'e2n', 'prob': pew*prt},
                  {'route': 's2n', 'prob': psn},
                  {'route': 's2w', 'prob': psn*plt},
                  {'route': 's2e', 'prob': psn*prt},
                  {'route': 'w2e', 'prob': pwe},
                  {'route': 'w2n', 'prob': pwe*plt},
                  {'route': 'w2s', 'prob': pwe*prt}]

        head = ['<routes>',
                '<vType id="stdSKVehicle" accel="2.6" decel="4.5" sigma="0.5" '
                'length="5" minGap="2.5" maxSpeed="22.22" guiShape="passenger"/>',
                '<route id="n2s" edges="4fi 4si 3o" />',
                '<route id="n2e" edges="4fi 4si 2o" />',
                '<route id="n2w" edges="4fi 4si 1o" />',
                '<route id="e2w" edges="2fi 2si 1o" />',
                '<route id="e2s" edges="2fi 2si 3o" />',
                '<route id="e2n" edges="2fi 2si 4o" />',
                '<route id="s2n" edges="3fi 3si 4o" />',
                '<route id="s2w" edges="3fi 3si 1o" />',
                '<route id="s2e" edges="3fi 3si 2o" />',
                '<route id="w2e" edges="1fi 1si 2o" />',
                '<route id="w2n" edges="1fi 1si 4o" />',
                '<route id="w2s" edges="1fi 1si 3o" />']

        tail = "</routes>"
        with open(self.route_file, "w") as output_file:
            print(os.linesep.join(head), file=output_file)
            cur_veh = 0
            for i in range(num_time_step):
                for route in routes:
                    if random.uniform(0, 1) < route['prob']:
                        print('<vehicle id="%s_%i" type="%s" route="%s" depart="%i" departLane="random" '
                              'departSpeed="random"/>' % (route['route'], cur_veh, 'stdSKVehicle',
                                                          route['route'], i), file=output_file)
                        cur_veh += 1
            print(tail, file=output_file)
    
    def seed(self, seed=None):
        if seed:
            random.seed(seed)

    def run(self):
        step = 0
        Queue_Length = np.zeros((self.n_road,self.n_lane,1))
        Phase = [2]
        # we start with phase 2 where EW has green
        traci.trafficlight.setPhase("0", 2)
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            step += 1
            Phase.append(traci.trafficlight.getPhase('0'))
            cx_res = traci.junction.getContextSubscriptionResults("0")
            print(cx_res)
            if not cx_res:
                ql_step = np.zeros((self.n_road,self.n_lane,1))
                Queue_Length = np.concatenate((Queue_Length,ql_step),axis=2)
                continue
            ql_step = np.zeros((self.n_road,self.n_lane,1))
            for vid, mes in cx_res.items():
                if mes[tc.VAR_LANE_ID].__contains__('i'):
                    rid,lid=[int(x) for x in mes[tc.VAR_LANE_ID].split('si_')]
                    if mes[tc.VAR_SPEED] < 1:
                        ql_step[rid-1,lid,0]+=1
            Queue_Length = np.concatenate((Queue_Length,ql_step),axis=2)
        
        # np.save(self.queue_file, Queue_Length)
        # np.save(self.phases_file, Phase)

        X=np.arange(0,Queue_Length.shape[2],1)
        plt.figure(1)
        for i in range(self.n_road):
            for j in range(self.n_lane):
                plt.subplot(4, 3, 3*i+j+1)
                plt.plot(X,Queue_Length[i,j,:])
        plt.show()
        traci.close()
        sys.stdout.flush()
    
    def get_message(self, cx_res):
        if not cx_res:
            return [np.zeros((self.n_road,self.n_lane,1)), None]
        else:
            message = [[[],[],[]],[[],[],[]],[[],[],[]],[[],[],[]]]
            ql_step = np.zeros((self.n_road,self.n_lane,1))
            for vid, mes in cx_res.items():
                if mes[tc.VAR_LANE_ID].__contains__('i'):
                    rid,lid=[int(x) for x in mes[tc.VAR_LANE_ID].split('si_')]
                    message[rid-1][lid].append([mes[tc.VAR_LANEPOSITION],mes[tc.VAR_SPEED],mes[tc.VAR_ACCELERATION]])
                    if mes[tc.VAR_SPEED] < 1:
                        ql_step[rid-1,lid,0]+=1
            for i in range(self.n_road):
                for j in range(self.n_lane):
                    message[i][j].sort(key=lambda x:self.road_length-x[0])
            return [ql_step, message]
    
    # def get_reward(self, Message, current_phase):
    #     if current_phase%2!=0: 
    #     # yellow phase
    #         return None
    #     else:
    #         ql_step, message = Message
    #         Reward = np.zeros((self.n_road,self.n_lane,2))
    #         for i in self.n_road:
    #             for j in self.n_lane:

    #         rid,lid = self.phase2road_lane_index[current_phase]
    #         return None




if __name__=="__main__":
    Sim=SumoSim("data/net.sumocfg","data/net.rou.xml")
    Sim.run()