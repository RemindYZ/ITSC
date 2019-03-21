from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import random
import sys
import numpy as np
from rl.core import Env
from rl.core import Space

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


class SumoEnv(Env):
    """sumo environment definition, extend rl.core.Env"""
    def __init__(self, cfg_file, route_file, router=None, observed_prob=1.0, nogui=False, log_out="log/inter"):
        self.cfg_file = cfg_file
        self.route_file = route_file
        self.router = router
        if not self.router:
            self.generate_route_file()
        else:
            self.router.generate_route_file(self.route_file)

        self.observable_probability = observed_prob
        self.observable_id = set()
        self.rix = [0, 2, 1, 3]

        if nogui:
            self.sumoBinary = checkBinary('sumo')
            self.cmd_lines = [self.sumoBinary, "--start", "-c", self.cfg_file]
            self.load_cmd = ["--start", "-c", self.cfg_file]
        else:
            self.sumoBinary = checkBinary('sumo-gui')
            # --start, --quit-on-end
            self.cmd_lines = [self.sumoBinary, "-c", self.cfg_file]
            self.load_cmd = ["-c", self.cfg_file]

        self.log_out = log_out
        if not os.path.exists(log_out):
            os.makedirs(log_out)

        self.log_file = os.path.join(log_out, 'metrics.log')
        self.phases_file = os.path.join(log_out, 'phases.log')

        # constants
        self.road_length = 500  # m
        self.margin = 14.65  # m
        self.max_speed = 16.67  # m/s

        # divide each lane into 32 cell
        self.length = 256
        self.eps = 2
        self.veh_length = 5  # m
        self.cell_length = 16  # m
        self.max_n = 8.0
        self.n_lanes = 4
        self.n_sects = 4
        self.region = self.length + self.margin - self.eps
        self.height = self.length // self.cell_length
        self.width = self.n_sects
        self.index2green_phases = [0, 2, 4, 6]
        self.green_phases2index = {0: 0, 2: 1, 4: 2, 6: 3}
        self.phases_length = 8
        self.max_green_time = 60  # s
        self.min_green_time = 6  # s
        self.action_space = PhasesSpace()
        # max_simulation_time, ms
        self.max_sim_time = 3600000

        # the value need to be reset in reset()
        self.epoch_num = 0
        self.n_decision_step = 0
        self.cycle = 0
        self.phase_line = []
        self.observable_id = set()
        self.arrived_num = 0
        self.total_waiting_time = 0
        self.last_jam_vl = 0
        self.last_green_time = 0

        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start(self.cmd_lines)
        # context subscriptions, subscribe vehicles near the intersection
        traci.junction.subscribeContext("0", tc.CMD_GET_VEHICLE_VARIABLE, self.region,
                                        [tc.VAR_LANE_ID, tc.VAR_ROAD_ID,
                                         tc.VAR_WAITING_TIME, tc.VAR_LANEPOSITION,
                                         tc.VAR_SPEED])

    def step(self, action):
        # definition = traci.trafficlight.getCompleteRedYellowGreenDefinition('tlstatic')
        self.n_decision_step += 1
        current_phase = traci.trafficlight.getPhase('tlstatic')
        current_index = self.green_phases2index[current_phase]  # type: int
        next_phase = self.index2green_phases[(action + current_index) % len(self.index2green_phases)]
        if next_phase == current_phase:
            # current phase continues 2s
            traci.trafficlight.setPhase('tlstatic', current_phase)
            _ = self.simulate_one_step()
            flag = self.simulate_one_step()

            self.last_green_time += 2

            if self.last_green_time >= self.max_green_time:
                next_phase = self.index2green_phases[(current_index + 1)
                                                     % len(self.index2green_phases)]
                flag = self.change2phase(current_phase, next_phase)
        else:
            flag = self.change2phase(current_phase, next_phase)

        cx_res = traci.junction.getContextSubscriptionResults("0")
        obs = self.get_observations(cx_res)
        # remain modification
        reward = self.get_reward(cx_res)

        # if traci.simulation.getCurrentTime() > self.max_sim_time:  # ms
        # print("MinExpected Number: %d" % traci.simulation.getMinExpectedNumber())
        if not flag:
            with open(self.log_file, 'a') as log:
                # write sim time /t arrived num /t aver waiting time /n
                log.write('epoch: %d, time_step: %d, decision_step: %d, arrived_num: %d, awt: %.4f\n' %
                          (self.epoch_num, traci.simulation.getCurrentTime(), self.n_decision_step,
                           self.arrived_num, float(self.total_waiting_time) / float(self.arrived_num)))
            # returns a tuple (observation, reward, done, info). "Done" indicates whether the simulation has end.
            self.epoch_num += 1
            return obs, reward, True, {}
        else:
            return obs, reward, False, {}

    def change2phase(self, current_phase, next_phase):
        """shift the current green phase to next green phase, with min duration time 6s
        if current_phase == next_phase, reset the current_phase"""
        if current_phase == next_phase:
            traci.trafficlight.setPhase('tlstatic', current_phase)
            return True
        traci.trafficlight.setPhase('tlstatic', (current_phase + 1) % self.phases_length)
        if current_phase == 0:
            self.phase_line = ['epoch: %d, cycle: %d, green_phase: %d'
                               % (self.epoch_num, self.cycle, self.last_green_time)]
        elif current_phase == 6:
            self.phase_line.append(str(self.last_green_time))
            with open(self.phases_file, 'a') as ph_file:
                if len(self.phase_line) == len(self.index2green_phases):
                    ph_file.write(','.join(self.phase_line) + '\n')
            self.cycle += 1
        else:
            self.phase_line.append(str(self.last_green_time))
        self.last_green_time = 0
        # reset the traffic lights to the action green phases
        while self.last_green_time < self.min_green_time:  # min duration time 6s
            flag = self.simulate_one_step()
            if not flag:
                return False
            if traci.trafficlight.getPhase('tlstatic') == next_phase:
                self.last_green_time += 1
            else:
                self.last_green_time = 0
        return True

    def simulate_one_step(self):
        if traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            for i in range(1, self.n_sects + 1):
                for j in range(0, self.n_lanes):
                    det = 'e2det_{}i_{}'.format(i, j)
                    self.total_waiting_time += traci.lanearea.getLastStepHaltingNumber(det)

            self.arrived_num += traci.simulation.getArrivedNumber()
            id_list = traci.simulation.getDepartedIDList()
            for ix in id_list:
                if (ix not in self.observable_id) and random.uniform(0, 1) < self.observable_probability:
                    self.observable_id.add(ix)
            aid_list = traci.simulation.getArrivedIDList()
            for aid in aid_list:
                if aid in self.observable_id:
                    self.observable_id.remove(aid)
            return True
        else:
            return False

    def get_observations(self, cx_res):
        """get vehicle position and speed matrix from context subscription, plus the current phase as one hot vector"""
        current_phase = traci.trafficlight.getPhase('tlstatic')
        if current_phase not in self.green_phases2index:
            current_index = random.randint(0, len(self.index2green_phases) - 1)
        else:
            current_index = self.green_phases2index[current_phase]
        phase = np.zeros(len(self.index2green_phases))
        phase[current_index] = 1
        position = np.zeros((self.height, self.width))
        speed = np.zeros((self.height, self.width))
        if not cx_res:
            return np.array([position, speed, phase])
        for vid, mes in cx_res.iteritems():
            if (vid in self.observable_id) and mes[tc.VAR_LANE_ID].__contains__('i'):
                rid, lid = [int(x) for x in mes[tc.VAR_LANE_ID].split('i_')]
                cid = int((self.road_length - self.margin - mes[tc.VAR_LANEPOSITION]) / self.cell_length)
                assert cid >= 0
                assert cid < self.height
                ix = self.rix[rid - 1]
                position[cid][ix] += 1
                speed[cid][ix] = speed[cid][ix] + (mes[tc.VAR_SPEED] - speed[cid][ix]) / position[cid][ix]
        position /= self.max_n
        speed /= self.max_speed
        return np.array([position, speed, phase])

    def get_reward(self, cx_res):
        """get change of the total queuing vehicle number between two state as reward."""
        if not cx_res:
            reward = self.last_jam_vl
            self.last_jam_vl = 0
            return reward
        jam_vl = 0
        for i in range(1, self.n_sects + 1):
            for j in range(0, self.n_lanes):
                det = 'e2det_{}i_{}'.format(i, j)
                jam_vl += traci.lanearea.getJamLengthVehicle(det)
        reward = self.last_jam_vl - jam_vl
        self.last_jam_vl = jam_vl
        return reward

    def reset(self):
        # context subscriptions
        # traci.close(wait=True)
        # Reset before simulationOneStep
        self.clear_metrics()
        if not self.router:
            self.generate_route_file()
        else:
            self.router.generate_route_file(self.route_file)

        # traci.start(self.cmd_lines)
        traci.load(self.load_cmd)
        traci.junction.subscribeContext("0", traci.constants.CMD_GET_VEHICLE_VARIABLE, self.region,
                                        [traci.constants.VAR_LANE_ID, traci.constants.VAR_ROAD_ID,
                                         traci.constants.VAR_WAITING_TIME, traci.constants.VAR_LANEPOSITION,
                                         traci.constants.VAR_SPEED])
        # reset the traffic lights to the action green phases
        while self.last_green_time < self.min_green_time:
            self.simulate_one_step()
            if traci.trafficlight.getPhase('tlstatic') in self.green_phases2index:
                self.last_green_time += 1
            else:
                self.last_green_time = 0
        cx_res = traci.junction.getContextSubscriptionResults("0")
        return self.get_observations(cx_res)

    def clear_metrics(self):
        self.n_decision_step = 0
        self.cycle = 0
        self.phase_line = []
        self.observable_id = set()
        self.arrived_num = 0
        self.total_waiting_time = 0
        self.last_jam_vl = 0
        self.last_green_time = 0

    def close(self):
        traci.close(wait=True)
        self.epoch_num = 0
        self.clear_metrics()

    def generate_route_file(self, pns=1.0/4.0, pew=1.0/8.0, psn=1.0/4.0, pwe=1.0/8.0, plt=1.0/4.0, prt=1.0/4.0):
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
                '<route id="n2s" edges="51i 1i 3o 53o" />',
                '<route id="n2e" edges="51i 1i 2o 52o" />',
                '<route id="n2w" edges="51i 1i 4o 54o" />',
                '<route id="e2w" edges="52i 2i 4o 54o" />',
                '<route id="e2s" edges="52i 2i 3o 53o" />',
                '<route id="e2n" edges="52i 2i 1o 51o" />',
                '<route id="s2n" edges="53i 3i 1o 51o" />',
                '<route id="s2w" edges="53i 3i 4o 54o" />',
                '<route id="s2e" edges="53i 3i 2o 52o" />',
                '<route id="w2e" edges="54i 4i 2o 52o" />',
                '<route id="w2n" edges="54i 4i 1o 51o" />',
                '<route id="w2s" edges="54i 4i 3o 53o" />']

        tail = "</routes>"

        # print("pns: %.2f, pew: %.2f, psn: %.2f, pwe: %.2f, plt: %.2f, prt: %.2f" % (pns, pew, psn, pwe, plt, prt))
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
            random.seed(seed)  # make tests reproducible

    def render(self, mode='human', close=False):
        pass

    def configure(self, *args, **kwargs):
        pass


class PhasesSpace(Space):

    def __init__(self):
        self.phases = [0, 1]

    def sample(self, seed=None):
        """sample phase actions."""
        ix = random.randint(0, len(self.phases) - 1)
        return self.phases[ix]

    def contains(self, x):
        return self.phases.__contains__(x)


class Router(object):

    def __init__(self):
        self.seed = None

    def set_seed(self, seed=42):
        if seed:
            self.seed = seed
            random.seed(seed)

    def generate_route_file(self, output_file):
        """Generate the .rou.xml file."""
        raise NotImplementedError()
