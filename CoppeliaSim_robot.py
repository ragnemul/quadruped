import sim
import numpy as np
import time


class CoppeliaSim:
    __ip = None
    __port = None
    __clientID = None
    __CoM = None
    __CoP = None
    __tip = None

    __all_joints = [__joint1_1, __joint1_2, __joint1_3,
                    __joint2_1, __joint2_2, __joint2_3,
                    __joint3_1, __joint3_2, __joint3_3,
                    __joint4_1, __joint4_2, __joint4_3] = range(12)

    __all_fs = [__fs1, __fs2, __fs3, __fs4] = range(4)

    __cadera = None
    __leg1 = None
    __leg2 = None
    __leg3 = None
    __leg4 = None

    __fs = None

    __MOVING_STATUS_THRESHOLD = 1       # moving threshold in degrees

    # mask for adapting sense rotation joints
    __joint_pos_mask = np.array([1, 1, 1,
                                 1, 1, 1,
                                 1, 1, 1,
                                 1, 1, 1])


    def __init__(self, ip='127.0.0.1', port=19999):
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.__ip = ip
        self.__port = port

        # joints Ids
        self.__clientID, self.__CoM, self.__CoP, self.__tip, self.__fs, self.__joints = self.connect()

        self.__cadera = [self.__joint1_1, self.__joint2_1, self.__joint3_1, self.__joint4_1]
        self.__rodillas = [self.__joint1_2, self.__joint2_2, self.__joint3_2, self.__joint4_2]
        self.__leg1 = [self.__joint1_1, self.__joint1_2, self.__joint1_3]
        self.__leg2 = [self.__joint2_1, self.__joint2_2, self.__joint2_3]
        self.__leg3 = [self.__joint3_1, self.__joint3_2, self.__joint3_3]
        self.__leg4 = [self.__joint4_1, self.__joint4_2, self.__joint4_3]

        return

    def __del__(self):
        return

    def connect(self):
        self.__clientID = sim.simxStart(self.__ip, self.__port, True, True, 2000, 5)  # Conectarse
        if self.__clientID == 0:
            print("conectado a", self.__port)

            # Requerimos los manejadores para las articulaciones y el Dummy

            retCode, CoM = sim.simxGetObjectHandle(self.__clientID, 'CoM', sim.simx_opmode_blocking)
            retCode, CoP = sim.simxGetObjectHandle(self.__clientID, 'CoP', sim.simx_opmode_blocking)
            retCode, tip = sim.simxGetObjectHandle(self.__clientID, 'tip', sim.simx_opmode_blocking)
            retCode, fs1 = sim.simxGetObjectHandle(self.__clientID, 'FS1', sim.simx_opmode_blocking)
            retCode, fs2 = sim.simxGetObjectHandle(self.__clientID, 'FS2', sim.simx_opmode_blocking)
            retCode, fs3 = sim.simxGetObjectHandle(self.__clientID, 'FS3', sim.simx_opmode_blocking)
            retCode, fs4 = sim.simxGetObjectHandle(self.__clientID, 'FS4', sim.simx_opmode_blocking)

            retCode, joint1_1 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint1_1', sim.simx_opmode_blocking)
            retCode, joint1_2 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint1_2', sim.simx_opmode_blocking)
            retCode, joint1_3 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint1_3', sim.simx_opmode_blocking)

            retCode, joint2_1 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint2_1', sim.simx_opmode_blocking)
            retCode, joint2_2 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint2_2', sim.simx_opmode_blocking)
            retCode, joint2_3 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint2_3', sim.simx_opmode_blocking)

            retCode, joint3_1 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint3_1', sim.simx_opmode_blocking)
            retCode, joint3_2 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint3_2', sim.simx_opmode_blocking)
            retCode, joint3_3 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint3_3', sim.simx_opmode_blocking)

            retCode, joint4_1 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint4_1', sim.simx_opmode_blocking)
            retCode, joint4_2 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint4_2', sim.simx_opmode_blocking)
            retCode, joint4_3 = sim.simxGetObjectHandle(self.__clientID, 'MTB_joint4_3', sim.simx_opmode_blocking)

            return self.__clientID, \
                   CoM, \
                   CoP, \
                   tip, \
                   np.array ([fs1,fs2,fs3,fs4]),\
                   np.array([joint1_1, joint1_2, joint1_3,
                             joint2_1, joint2_2, joint2_3,
                             joint3_1, joint3_2, joint3_3,
                             joint4_1, joint4_2, joint4_3])

        else:
            print("no se pudo conectar")

        return


    # Establece la posición de las articulaciones expresadas en radianes
    def setJoints(self, J, Q):
        retCode = sim.simxSetJointTargetPosition(self.__clientID, J[0], Q[0], sim.simx_opmode_oneshot)
        retCode = sim.simxSetJointTargetPosition(self.__clientID, J[1], Q[1], sim.simx_opmode_oneshot)
        retCode = sim.simxSetJointTargetPosition(self.__clientID, J[2], Q[2], sim.simx_opmode_oneshot)

        return retCode


    def get_CoM_position_and_orientation(self):
        [r, p] = sim.simxGetObjectPosition(self.__clientID, self.__CoM, -1, sim.simx_opmode_blocking)
        [r, o] = sim.simxGetObjectOrientation(self.__clientID, self.__CoM, -1, sim.simx_opmode_blocking)
        return [r, p, o]


    def get_pCoM(self):
        [r, p] = sim.simxGetObjectPosition(self.__clientID, self.__CoM, -1, sim.simx_opmode_blocking)
        return p


    def get_oCoM(self):
        [r, o] = sim.simxGetObjectOrientation(self.__clientID, self.__CoM, -1, sim.simx_opmode_blocking)
        return o


    # joints: np.array con los manejadores de las articulaciones a mover
    # radians: los valores de los radianes a mover
    def set_joints_position(self, joints, radians):
        for j, r in zip(joints, radians):
            r = sim.simxSetJointTargetPosition(self.__clientID, j, r, sim.simx_opmode_oneshot)
        return r


    def default_position(self):
        # Default position for all joints
        #self.set_joints_position(self.__joints[self.__all_joints], [0] * 12)
        self.move_joints(self.__all_joints,[0]*12)
        return


    def get_joints_position_leg1(self):
        pos = self.get_joints_position(self.__leg1)
        #pos = self.mask_joint_positions(pos, self.__leg1)
        return pos


    def get_joints_position_leg2(self):
        pos = self.get_joints_position(self.__leg2)
        #pos = self.mask_joint_positions(pos, self.__leg2)
        return pos


    def get_joints_position_leg3(self):
        pos = self.get_joints_position(self.__leg3)
        #pos = self.mask_joint_positions(pos, self.__leg3)
        return pos


    def get_joints_position_leg4(self):
        pos = self.get_joints_position(self.__leg4)
        #pos = self.mask_joint_positions(pos, self.__leg4)
        return pos


    # returns the position of joints specified by joints_idx
    def get_joints_position(self, joints_idx):
        n = len(joints_idx)
        r = list(map(sim.simxGetJointPosition, [self.__clientID] * n, self.__joints[joints_idx],
                     [sim.simx_opmode_blocking] * n))

        # getting the position in radians (second item in tuples)
        pos = [x[1] for x in r]

        # mask the joints orientation
        pos = self.mask_joint_positions(pos, joints_idx)
        return pos


    # set the joints positions specified by joints_idx
    def move_joints(self, joints_idx, goal_pos, wait=5):
        ts1 = time.time()

        elements = len(joints_idx)
        if (elements != len(goal_pos)):
            print ("set_position: lengths of joints_idx and target_positions mismatch")
            quit()

        present_pos = self.get_joints_position(joints_idx)
        # grados entre posición actual y objetivo
        g = [abs(np.rad2deg(g) - np.rad2deg(p)) for g, p in zip(goal_pos, present_pos)]

        # velocidad de giro de los motores a mover
        v = [250]*elements

        # tiempo de giro estimado según velocidad v y grados a recorred g
        t_out = [g / vj  for vj, g in zip(v, g)]

        sim.simxPauseCommunication(self.__clientID, 1)
        r = list(map(sim.simxSetJointTargetPosition,
                     [self.__clientID] * elements,
                     self.__joints[joints_idx],
                     goal_pos,
                     [sim.simx_opmode_oneshot] * elements))
        sim.simxPauseCommunication(self.__clientID, 0);

        if (wait > 0):
            # waits for completion of movement
            finish = [False] * elements
            t_ini = time.time()
            while finish.count(True) < elements:
                # gets the raw present position of the motors
                pos = self.get_joints_position(joints_idx)
                for i in range(elements):
                    if not ((abs(goal_pos[i] - pos[i]) > np.deg2rad(self.__MOVING_STATUS_THRESHOLD))):
                        finish[i] = True
                    else:
                        if time.time() - t_ini > t_out[i] * wait:
                            print("Possible Overload in joint ", joints_idx[i], "! correcting position")
                            sim.simxSetJointTargetPosition(self.__clientID, self.__joints[joints_idx[i]], pos[i], sim.simx_opmode_oneshot)
                            finish[i] = True
                        else:
                            finish[i] = False
                    #finish[i] = not ((abs(goal_pos[i] - pos[i]) > np.deg2rad(self.__MOVING_STATUS_THRESHOLD)))

            pos = self.get_joints_position(joints_idx)
            IMUlog = []
            ts2 = time.time()
        return [ts1, ts2, IMUlog, pos]


        # move the joints specified by joints_idx the radians specified by Q
        def move_joints_radians(self, Q, joints_idx):
            self.move_joints(joints_idx, Q)
            return


    def move_offset(self, q):
        q_masked = self.mask_joint_positions(q, self.__all_joints)
        self.move_joints(self.__all_joints, q_masked)
        return


    def get_fs(self):
        fs_idx = self.__all_fs

        n = len(fs_idx)
        r = list(map(sim.simxReadForceSensor, [self.__clientID] * n, self.__fs[fs_idx],
                     [sim.simx_opmode_blocking] * n))

        print (r)

        # getting the position in radians (second item in tuples)
        fs = [x[2][2] for x in r]

        return fs


    def get_leg1_idx(self):
        return self.__leg1


    def get_leg2_idx(self):
        return self.__leg2


    def get_leg3_idx(self):
        return self.__leg3


    def get_leg4_idx(self):
        return self.__leg4


    def get_leg_idx(self, n):
        if (n == 1):
            return self.__leg1
        if (n == 2):
            return self.__leg2
        if (n == 3):
            return self.__leg3
        if (n == 4):
            return self.__leg4



    def get_hip_idx(self):
        return self.__cadera


    def get_knees_idx(self):
        return self.__rodillas

    def get_all_joints_idx(self):
        return self.__all_joints


    def set_CoP_position(self, pos):
        r = sim.simxSetObjectPosition (self.__clientID, self.__CoP, -1, pos, sim.simx_opmode_blocking)
        return r


    def move_leg(self, i, Q):
        if i == 1:
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg1,self.mask_joint_positions(Q, self.__leg1))
            #self.set_joints_position(self.__joints[self.__leg1], self.mask_joint_positions(Q, self.__leg1))
        elif i == 2:
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg2, self.mask_joint_positions(Q, self.__leg2))
            #self.set_joints_position(self.__joints[self.__leg2], self.mask_joint_positions(Q, self.__leg2))
        elif i == 3:
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg3, self.mask_joint_positions(Q, self.__leg3))
            #self.set_joints_position(self.__joints[self.__leg3], self.mask_joint_positions(Q, self.__leg3))
        elif i == 4:
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg4, self.mask_joint_positions(Q, self.__leg4))
            #self.set_joints_position(self.__joints[self.__leg4], self.mask_joint_positions(Q, self.__leg4))
        return [ts1, ts2, imu_log, pos]



    def desplaza_cadera (self,d):
        self.move_joints(self.__cadera, np.deg2rad([-d, d, d, -d]))
        return


    def desplaza_rodillas (self,d):
        self.move_joints(self.__rodillas, np.deg2rad([d, -d, -d, d]))
        return


    # Desplaza las rodillas sumando d grados a las posiciones KD (Knees Degrees)
    def knees_offset(self, KD, d):
        d = np.float(d)
        self.move_joints(self.__rodillas, np.deg2rad([KD[0] + d, KD[1] - d, KD[2] - d, KD[3] + d]))
        return


    # Desplaza las rodillas sumando d grados a las posiciones KD (Knees Degrees)
    def hip_offset(self, HD, d):
        d = np.float(d)
        self.move_joints(self.__cadera, np.deg2rad([HD[0] - d, HD[1] + d, HD[2] + d, HD[3] - d]))
        return


    # adapts the sense rotation of the joints for the physical robot
    def mask_joint_positions(self, q, joints_idx):
        mask = self.__joint_pos_mask[joints_idx]
        pos = [x * y for x, y in zip(mask, q)]

        return pos


    def get_joints_handle(self, idx):
        return self.__joints[tuple(idx)]


    def set_moving_speed(self, value, idx = __all_joints):
        # sim.simxPauseCommunication(self.__clientID, 1)
        #
        # for i in idx:
        #     a=sim.simxSetJointTargetVelocity(self.__clientID,self.__joints[i],value,sim.simx_opmode_oneshot)
        #     sim.simxReset
        #
        # sim.simxPauseCommunication(self.__clientID, 0)
        pass
        return


    def get_low_speed(self):
        pass
        return

    def get_high_speed(self):
        pass
        return

    def degrees_to_value(self,d):
        return d

    def set_Q_goal (self, Q):
        goal = Q
        return goal

    def knees_hip_offset(self, KD, Koff, HD, Hoff):
        Koff = np.float(Koff)
        Qk = [KD[0] + Koff, KD[1] - Koff, KD[2] - Koff, KD[3] + Koff]

        Hoff = np.float(Hoff)
        Qh = [HD[0] - Hoff, HD[1] + Hoff, HD[2] + Hoff, HD[3] - Hoff]

        self.set_moving_speed(self.get_low_speed(), self.get_knees_idx() + self.get_hip_idx())

        self.move_joints(self.get_knees_idx() + self.get_hip_idx(), list(map(
            np.deg2rad, self.mask_joint_positions(Qk + Qh, self.get_knees_idx() + self.get_hip_idx()))
        ))
        self.set_moving_speed(self.get_high_speed(), self.get_knees_idx() + self.get_hip_idx())

        return








