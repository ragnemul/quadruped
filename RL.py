import numpy as np
from robot import Robot
from physical_robot import Physical_robot
import time
import json


####### REINFORCEMENT LEARNING

class StateQ:


    def __init__(self, robot):
        self.robot = robot
        self.state = "start"
        self.isEnd = False
        self.executed_13 = False
        self.executed_24 = False
        self.determine = True


    def giveReward(self):
        # default reward
        r = -5

        if (self.state == "OK"):
            r = 20
        if (self.state == "KO"):
            r = -1

        return r


    def nxtPosition(self, action):
        return action



    def checkTermination(self):

        r = 0       # default reward
        if (self.state == "Q" and self.executed_24 and self.executed_13):
            c = self.robot.get_IMU_pos(self.robot.ts1, self.robot.ts2, self.robot.sensors_log, 1)
            c_gyro = self.robot.get_IMU_gyro(self.robot.ts1, self.robot.ts2, self.robot.sensors_log, 1)
            c_r = sum(map(lambda x: round(x) <= 12, abs(np.rad2deg(c[0:2]))))        # 5, 10
            c_gyro_r = sum(map(lambda x: round(x) <= 14, abs(np.rad2deg(c_gyro))))  # 10

            if (c_r >= 2 or c_gyro_r >= 2) and (self.robot.ts2-self.robot.ts1 <= 2):
            #if (c_gyro_r >= 2):
                self.isEnd = True
                self.state = "OK"
                r = 100
            else:
                self.isEnd = True
                self.state = "KO"
                r = -1
            return r

        # condición de perdida de soporte
        ls = 0
        for i in range(10):
            if (self.robot.lack_of_support()):
                ls = ls + 1
        if (ls >= 4):
            time.sleep(1)
            if (self.robot.lack_of_support()):
                self.isEnd = True
                self.state = "KO"
                r = -5
                return r

        # condición de excepción en IK. Posición no alcanzable
        if (self.robot.excp_cond()):
            self.robot.clear_exception()
            self.isEnd = True
            self.state = "KO"
            r = -5
            return r

        return r


class AgentQ:
    actions = None
    Q_offset_004 = [-0.0, 0.859029241215959, -1.4572817484913592,
                    -0.02045307717180855, -0.7976700097005335, 1.452168479198407,
                    0.0051132692929521375, 0.8845955876807198, -1.4317154020265985,
                    -0.05113269292952137, -0.853915971923007, 1.4726215563702156]
    my_funcs = None
    lr = 0.2
    exp_rate = 0.3
    states = []
    jsonfile = "state_values.json"
    decay_gamma = 0.9

    def __init__(self, robot):
        self.State = StateQ(robot)
        self.robot = robot
        self.actions = ["Q",
                        "1&3",
                        "2&4",
                        "FW",
                        "BW",
                        "L",
                        "R"
                        # "FW-R",
                        # "FW-L",
                        # "BW-R",
                        # "BW-L"
                        ]
        self.my_funcs = {"Q": robot.move_offset,  # self.Q_offset_004
                         "1&3": robot.offset_legs13,  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                         "2&4": robot.offset_legs24,  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                         "FW": robot.offset_knees_and_hip,  # (-10, 0)
                         "BW": robot.offset_knees_and_hip,  # (10, 0)
                         "L": robot.offset_knees_and_hip,  # (0, 10)
                         "R": robot.offset_knees_and_hip  # (0, -10)
                         # "FW-R": robot.offset_knees_and_hip,  # (-10, -10)
                         # "FW-L": robot.offset_knees_and_hip,  # (-10, 10)
                         # "BW-R": robot.offset_knees_and_hip,  # (10, -10)
                         # "BW-L": robot.offset_knees_and_hip  # (10, 10)
                         }

        self.Q_values = {"start": {},
                             "Q": {},  # self.Q_offset_004
                             "1&3": {},  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                             "2&4": {},  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                             "FW": {},  # (-10, 0)
                             "BW": {},  # (10, 0)
                             "L": {},  # (0, 10)
                             "R": {},  # (0, -10)
                         # "FW-R": {},  # (-10, -10)
                         # "FW-L": {},  # (-10, 10)
                         # "BW-R": {},  # (10, -10)
                         # "BW-L": {},  # (10, 10)
                         "KO": {},  # Condición de error en el objetivo
                         "OK": {}  # Se logra el objetivo
                         }

        self.stateM = {"start": {"FW": "FW", "2&4":"2&4", "1&3":"1&3", 'BW':'BW'},
                         "Q": {},  # self.Q_offset_004
                         "1&3": {},  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                         "2&4": {},  # (0, -0.04, 0.03, 0, 0, -0.03, 5)
                         "FW": {},  # (-10, 0)
                         "BW": {},  # (10, 0)
                         "L": {},  # (0, 10)
                         "R": {},  # (0, -10)
                         # "FW-R": {},  # (-10, -10)
                         # "FW-L": {},  # (-10, 10)
                         # "BW-R": {},  # (10, -10)
                         # "BW-L": {},  # (10, 10)
                         "KO": {},  # Condición de error en el objetivo
                         "OK": {}  # Se logra el objetivo
                         }

        for s in self.Q_values:
            for a in self.actions:
                    self.Q_values[s][a] = 0


        self.ts1 = None
        self.ts2 = None
        self.sensors_log = None
        self.pose = None

        self.l = []
        try:
            with open(self.jsonfile, "r") as file:
                self.l = json.load(file)
            self.Q_values = self.l[len(self.l) - 1]
        except:
            print("Se ha creado el fichero json")

    def __del__(self):
        print("Deleting agent...")

    def reset_step(self, Q):
        self.State.state = "start"
        self.states.clear()
        self.State.isEnd = False
        self.State.executed_24 = False
        self.State.executed_13 = False
        if (self.robot.excp_cond() == True):
            self.robot.clear_exception()

        self.robot.disable_torque(self.robot.get_all_joints_idx())
        self.robot.enable_torque(self.robot.get_all_joints_idx())
        self.robot.move_offset(Q, 5)
        self.robot.offset_legs13(0, 0, 0.03, 0, 0, -0.03, 5)
        self.robot.offset_legs24(0, 0, 0.03, 0, 0, -0.03, 5)
        self.robot.move_offset(Q, 5)
        return [None, None, None, None]



    def chooseAction(self, Qlibre):
        # choose action with most expected value
        mx_nxt_reward = 0      # 0 -0.999
        action = ""

        if np.random.uniform(0, 1) <= self.exp_rate:
            action = np.random.choice(self.actions)
            if (Qlibre == False):
                while ((self.State.executed_13 != True or self.State.executed_24 != True) and action == "Q"):
                    action = np.random.choice(self.actions)
        else:
            # greedy action
            total_actions = self.actions.copy()
            if (Qlibre == False):
                if (self.State.executed_13 != True or self.State.executed_24 != True):
                    total_actions.remove("Q")

            for a in total_actions:
                # if the action is deterministic
                # Se toma el valor del estado correspondiente a la acción
                current_state = self.State.state
                nxt_reward = self.Q_values[current_state][a]

                # se elige la acción con más valor.
                if nxt_reward >= mx_nxt_reward:
                    action = a
                    mx_nxt_reward = nxt_reward

            if (action == ''):
                action = np.random.choice(self.actions)

        return action


    def findAction(self, action):
        if ([s[1] for s in self.states].count(action) > 1):
            r = True
        else:
            r = False
        return r


    def runAction(self, action):
        position = self.State.nxtPosition(action)

        #if (position == "Q" and self.findAction("Q") == False):
        if (position == "start"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs["Q"](self.robot.Qi(), 5, 45)
        elif (position == "Q" and self.State.executed_13 and self.State.executed_24):
           time.sleep(0.5)
           [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](self.robot.Qi(), 5, 45)
        elif (position == "1&3" and self.findAction("1&3") == False):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](0, -0.04, 0.03, 0, 0, -0.03, 5)
            self.State.executed_13 = True
        elif (position == "2&4" and self.findAction("2&4") == False):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](0, -0.04, 0.03, 0, 0, -0.03, 5)
            self.State.executed_24 = True
        elif (position == "FW"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](-10, 0)
        elif (position == "FW-R"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](-10, -10)
        elif (position == "FW-L"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](-10, 10)
        elif (position == "L"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](0, 10)
        elif (position == "R"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](0, -10)
        elif (position == "BW"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](10, 0)
        elif (position == "BW-R"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](10, -10)
        elif (position == "BW-L"):
            [self.ts1, self.ts2, self.sensors_log, self.pose] = self.my_funcs[position](10, 10)

        time.sleep(0.5)  # 0.25
        self.State.state = position


    def trainning(self, rounds=200):
        self.robot.move_offset(self.robot.Qi(), 5)
        self.robot.offset_legs13(0, 0, 0.03, 0, 0, -0.03, 5)
        self.robot.offset_legs24(0, 0, 0.03, 0, 0, -0.03, 5)
        self.robot.move_offset(self.robot.Qi(), 5)

        i = 0
        while i < rounds:
            # to the end of game back propagate reward
            if self.State.isEnd:
                # back propagate
                #reward = self.State.giveReward()
                reward = r
                # explicitly assign end state to reward values
                for a in self.actions:
                    self.Q_values[self.State.state][a] = reward

                print("Game End Reward", reward)

                for s in reversed(self.states):
                    current_q_value = self.Q_values[s[0]][s[1][0]]
                    reward = current_q_value + self.lr * (s[1][1] + self.decay_gamma * reward - current_q_value)
                    self.Q_values[s[0]][s[1][0]] = round(reward, 3)

                self.l.append(self.Q_values.copy())
                with open(self.jsonfile, mode='w') as f:
                    json.dump(self.l, f)

                self.reset_step(self.robot.Qi())
                i += 1
            else:
                action = self.chooseAction(True)
                # append trace
                self.states.append([(self.State.state), [action,0]])
                print("current position {} action {}".format(self.State.state, action))

                # by taking the action, it reaches the next state
                self.runAction(action)

                # mark is end. r tiene la recompensa del estado final.
                r = self.State.checkTermination()

                # immediate reward
                #self.states[len(self.states) - 1][1][1] = r

                print("nxt state", self.State.state)
                print("---------------------")
        return


    def play(self,n):
        self.robot.move_offset(self.robot.Qi())
        for i in range(n):
            self.State.state = 'start'
            while (self.State.state != 'Q'):
                s = self.Q_values[self.State.state]
                self.State.state = max(s, key=s.get)
                self.runAction(self.State.state)
        return