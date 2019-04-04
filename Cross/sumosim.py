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
        self.pns, self.pew, self.psn, self.pwe, self.plt, self.prt = 0.125,0.25,0.125,0.25,0.25,0.25
        self.p_lane = [
            [self.pwe*(1+self.prt)/2,self.pwe*(1+self.prt)/2,self.pwe*self.plt],
            [self.pew*(1+self.prt)/2,self.pew*(1+self.prt)/2,self.pew*self.plt],
            [self.psn*(1+self.prt)/2,self.psn*(1+self.prt)/2,self.psn*self.plt],
            [self.pns*(1+self.prt)/2,self.pns*(1+self.prt)/2,self.pns*self.plt]
        ]
        self.cfg_file = cfg_file
        self.route_file = route_file
        self.flow_param = 1
        self.generate_route_file(self.pns, self.pew, self.psn, self.pew, self.plt, self.prt)
        

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
        self.min_green_time = [11,6,22,11]
        self.alpha = [0.6,0.25,0.1,0.05]
        self.phase2road_lane_index = {0:[[3,4],[0,1]],2:[[3,4],[2]],4:[[1,2],[0,1]],6:[[1,2],[2]]}
        self.interval_time = 25
        self.sat_flow_rate = 1600
        self.threshod_speed = 1
        # self.max_sim_time = 360000000
        
        traci.start(self.cmd_lines)
        # context subscriptions, subscribe vehicles near the intersection
        traci.junction.subscribeContext("0", tc.CMD_GET_VEHICLE_VARIABLE, self.detect_length,
                                        [tc.VAR_LANE_ID, tc.VAR_ROAD_ID,
                                         tc.VAR_WAITING_TIME, tc.VAR_LANEPOSITION,
                                         tc.VAR_SPEED, tc.VAR_ACCELERATION])
    
    def generate_route_file(self, pns, pew, psn, pwe, plt, prt):
        self.seed(40)
        num_time_step = 3600  # number of time steps
        routes = [{'route': 'n2s', 'prob': pns*self.flow_param},
                  {'route': 'n2e', 'prob': pns*plt*self.flow_param},
                  {'route': 'n2w', 'prob': pns*prt*self.flow_param},
                  {'route': 'e2w', 'prob': pew*self.flow_param},
                  {'route': 'e2s', 'prob': pew*plt*self.flow_param},
                  {'route': 'e2n', 'prob': pew*prt*self.flow_param},
                  {'route': 's2n', 'prob': psn*self.flow_param},
                  {'route': 's2w', 'prob': psn*plt*self.flow_param},
                  {'route': 's2e', 'prob': psn*prt*self.flow_param},
                  {'route': 'w2e', 'prob': pwe*self.flow_param},
                  {'route': 'w2n', 'prob': pwe*plt*self.flow_param},
                  {'route': 'w2s', 'prob': pwe*prt*self.flow_param}]

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
        last_step = 0
        Queue_Length = np.zeros((self.n_road,self.n_lane,1))
        Phase = [2]
        # we start with phase 2 where EW has green
        traci.trafficlight.setPhase("0", 2)
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            step += 1
            current_phase = traci.trafficlight.getPhase('0')
            if current_phase!=Phase[-1]:
                print(current_phase, step-last_step-4)
                last_step=step
            Phase.append(current_phase)
            cx_res = traci.junction.getContextSubscriptionResults("0")
            # print("current:",current_phase)
            if step>=last_step and current_phase%2==0:
                Message = self.get_message(cx_res)
                Reward = self.get_reward(Message, current_phase)
                Bool =self.get_action(Reward, current_phase)

                if Bool and step-last_step-3>=self.min_green_time[int(current_phase/2)]:
                    print(current_phase,"total time:",step-last_step-3)
                    traci.trafficlight.setPhase("0",(current_phase+1)%8)
                    last_step=step
                elif step-last_step>=self.max_green_time[int(current_phase/2)]:
                    print(current_phase, "total time:",step-last_step-3)
                    traci.trafficlight.setPhase("0",(current_phase+1)%8)
                    last_step=step
                else:
                    traci.trafficlight.setPhase("0",current_phase)



            # print(cx_res)
            if not cx_res:
                ql_step = np.zeros((self.n_road,self.n_lane,1))
                Queue_Length = np.concatenate((Queue_Length,ql_step),axis=2)
                continue
            ql_step = np.zeros((self.n_road,self.n_lane,1))
            for vid, mes in cx_res.items():
                if mes[tc.VAR_LANE_ID].__contains__('i'):
                    rid,lid=[int(x) for x in mes[tc.VAR_LANE_ID].split('si_')]
                    if mes[tc.VAR_SPEED] < self.threshod_speed:
                        ql_step[rid-1,lid,0]+=1
            Queue_Length = np.concatenate((Queue_Length,ql_step),axis=2)            
        
        np.save(self.queue_file, Queue_Length)
        np.save(self.phases_file, Phase)

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
                    message[rid-1][lid].append([self.road_length-mes[tc.VAR_LANEPOSITION],mes[tc.VAR_SPEED],mes[tc.VAR_ACCELERATION]])
                    if mes[tc.VAR_SPEED] < self.threshod_speed:
                        ql_step[rid-1,lid,0]+=1
            for i in range(self.n_road):
                for j in range(self.n_lane):
                    message[i][j].sort(key=lambda x:x[0])
            return [ql_step, message]
    
    def get_reward(self, Message, current_phase):
        if current_phase%2!=0 or not Message[1]: 
        # yellow phase
            return None
        else:
            ql_step, message = Message
            Reward = np.zeros((self.n_road,self.n_lane,2))
            arrival = np.zeros((self.n_road,self.n_lane))
            for r in range(self.n_road):
                for l in range(self.n_lane):
                    for m in message[r-1][l]:
                        if m[0]<=200 and m[1] > self.threshod_speed and m[1]+m[2]*self.interval_time < self.threshod_speed:
                            arrival[r-1,l] += 1
            rid,lid = self.phase2road_lane_index[current_phase]
            out = np.zeros((self.n_road,self.n_lane))
            for r in rid:
                for l in lid:
                    for m in message[r-1][l]:
                        if m[1]*self.interval_time+m[2]/2*(self.interval_time**2)>m[0]:
                            out[r-1,l] += 1
            Reward[:,:,0] = ql_step[:,:,0] + arrival - out

            next_phase = lambda x:x+2 if x < 6 else x-6
            nrid,nlid=self.phase2road_lane_index[next_phase(current_phase)]
            out = np.zeros((self.n_road,self.n_lane))
            for r in nrid:
                for l in nlid:
                    for m in message[r-1][l]:
                        if m[0]<2.6/2*((self.interval_time-3)**2):
                            out[r-1,l] += 1
            Reward[:,:,1] = ql_step[:,:,0] + arrival - out
            return Reward
        
    def get_action(self, Reward, current_phase):
        # true:swith;false:keep
        if Reward is None or current_phase%2!=0:
            return False
        else:
            reward_keep, reward_switch = self.max_reward(Reward)
            keep,switch=0,0
            if current_phase == 0:
                keep = [self.alpha[0]*reward_keep[0],self.alpha[1]*reward_keep[1]+self.alpha[2]*reward_keep[2]+self.alpha[3]*reward_keep[3]]
                switch = [self.alpha[0]*reward_switch[0],self.alpha[1]*reward_switch[1]+self.alpha[2]*reward_switch[2]+self.alpha[3]*reward_switch[3]]
            elif current_phase == 2:
                keep = [self.alpha[0]*reward_keep[1],self.alpha[1]*reward_keep[2]+self.alpha[2]*reward_keep[3]+self.alpha[3]*reward_keep[0]]
                switch = [self.alpha[0]*reward_switch[1],self.alpha[1]*reward_switch[2]+self.alpha[2]*reward_switch[3]+self.alpha[3]*reward_switch[0]]
            elif current_phase == 4:
                keep = [self.alpha[0]*reward_keep[2],self.alpha[1]*reward_keep[3]+self.alpha[2]*reward_keep[0]+self.alpha[3]*reward_keep[1]]
                switch = [self.alpha[0]*reward_switch[2],self.alpha[1]*reward_switch[3]+self.alpha[2]*reward_switch[0]+self.alpha[3]*reward_switch[1]]
            else:
                keep = [self.alpha[0]*reward_keep[3],self.alpha[1]*reward_keep[0]+self.alpha[2]*reward_keep[1]+self.alpha[3]*reward_keep[2]]
                switch = [self.alpha[0]*reward_switch[3],self.alpha[1]*reward_switch[0]+self.alpha[2]*reward_switch[1]+self.alpha[3]*reward_switch[2]]
            # print(keep, switch)
            if keep[0] > 1:
                return False
            elif sum(keep) <= sum(switch):
                return False
            else:
                return True

    def max_reward(self, Reward):
        reward_keep = [max(np.max(Reward[2:,0:2,0]),0),max(np.max(Reward[2:,2,0]),0),
                max(np.max(Reward[0:2,0:2,0]),0),max(np.max(Reward[0:2,2,0]),0)
                ]
        reward_switch = [max(np.max(Reward[2:,0:2,1]),0),max(np.max(Reward[2:,2,1]),0),
                max(np.max(Reward[0:2,0:2,1]),0),max(np.max(Reward[0:2,2,1]),0)
                ]
        return reward_keep,reward_switch




if __name__=="__main__":
    Sim=SumoSim("data/net.sumocfg","data/net.rou.xml")
    Sim.run()