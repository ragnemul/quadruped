import platform
import time

if platform.system() != 'Darwin':
    # from FS import FS
    from sensors import SENSORS

from U2D2 import U2D2
from motor import Motor
import numpy as np

from dynamixel_sdk import *  # Uses Dynamixel SDK library


class AllOf:
    def __init__(self, elements):
        self.elements = elements

    def __getattr__(self, attr):
        def on_all(*args, **kwargs):
            for obj in self.elements:
                getattr(obj, attr)(*args, **kwargs)

        return on_all


class Physical_robot:
    __ADDR_AX_GOAL_POSITION = 30
    __LEN_AX_GOAL_POSITION = 2
    __ADDR_AX_PRESENT_POSITION = 36
    __DXL_MOVING_STATUS_THRESHOLD = 15  # Dynamixel moving status threshold
    __PROTOCOL_VERSION = 1  # Protocol version = 1 for AX

    __device = None
    __portHandler = None
    __packetHandler = None
    __motores = None
    __sensors = SENSORS(name='imu')
    # __fs = FS()

    __all_joints = [__joint1_1, __joint1_2, __joint1_3,
                    __joint2_1, __joint2_2, __joint2_3,
                    __joint3_1, __joint3_2, __joint3_3,
                    __joint4_1, __joint4_2, __joint4_3] = range(12)

    __cadera = None
    __leg1 = None
    __leg2 = None
    __leg3 = None
    __leg4 = None

    __u2d2 = None

    # mask for align sense rotation joints with simulation
    __joint_pos_mask = np.array([-1, -1, -1,
                                 1, -1, -1,
                                 -1, -1, -1,
                                 1, -1, -1])

    __sleep = 0.2
    __low_speed = 55
    __high_speed = 148

    # Default device name when running from MAC using D2U2 is device = '/dev/tty.usbserial-FT4TFQEI'
    # When running from the RPI with the D2AUX, the device is '/dev/ttyACM0'
    def __init__(self, device='/dev/ttyACM0'):
        self.__device = device

        self.__u2d2 = U2D2(self.__device)
        self.__portHandler = self.__u2d2.get_portHandler()
        self.__packetHandler = self.__u2d2.get_packetHandler()

        self.__sensors.start()
        self.__sensors.reset_offset()

        # self.__fs.start()

        # joints Ids
        self.__joints = np.array(range(1, 12 + 1))

        self.__cadera = [self.__joint1_1, self.__joint2_1, self.__joint3_1, self.__joint4_1]
        self.__rodillas = [self.__joint1_2, self.__joint2_2, self.__joint3_2, self.__joint4_2]
        self.__pies = [self.__joint1_3, self.__joint2_3, self.__joint3_3, self.__joint4_3
                       ]
        self.__leg1 = [self.__joint1_1, self.__joint1_2, self.__joint1_3]
        self.__leg2 = [self.__joint2_1, self.__joint2_2, self.__joint2_3]
        self.__leg3 = [self.__joint3_1, self.__joint3_2, self.__joint3_3]
        self.__leg4 = [self.__joint4_1, self.__joint4_2, self.__joint4_3]

        self.__motores = np.array([Motor(self.__packetHandler, self.__portHandler, i) for i in range(1, 12 + 1)])

        self.enable_torque()

    # Deleting the Robot object
    def __del__(self):
        print("Terminating robot instance")
        AllOf(self.__motores).disable_torque()
        self.__u2d2.close_port()

        # Terminating the IMU thread
        print("Terminating IMU thread")
        self.__sensors.kill()

        print("Physical robot class stopped!\n")

    def readIMU(self):
        return self.__sensors.readIMU()

    # returns the value according to the degrees
    def degrees_to_value(self, d):
        value = (150 + d) * 1024 / 300
        return int(value)

    # returns the degrees according to the motor position value
    def value_to_degrees(self, v):
        degrees = (v / 1024 * 300) - 150
        return degrees

    def enable_torque(self, idx=__all_joints):
        AllOf(self.__motores[idx]).enable_torque()
        AllOf(self.__motores[idx]).set_max_torque(1023)
        AllOf(self.__motores[idx]).set_moving_speed(self.__high_speed)
        return

    def disable_torque(self, idx=__all_joints):
        AllOf(self.__motores[idx]).disable_torque()
        return

    def set_moving_speed(self, value=512, idx=__all_joints):
        AllOf(self.__motores[idx]).set_moving_speed(value)
        return

    def set_moving_speed_list(self, v, idx=__all_joints):
        for i in idx:
            self.__motores[i].set_moving_speed(v[i])
        return

    # mueve la pata i (1..4), los grados especificados en Q
    def setJoints(self, i, Q):
        if i == 1 or i == 3:
            Q = [-Q[0], -Q[1], -Q[2]]
        elif i == 2 or i == 4:
            Q = [Q[0], -Q[1], -Q[2]]

        # idx of firts joint of the leg
        i = (i - 1) * 3
        dxl_error = self.__motores[i].set_goal_position(self.degrees_to_value(Q[0]))
        dxl_error = self.__motores[i + 1].set_goal_position(self.degrees_to_value(Q[1]))
        dxl_error = self.__motores[i + 2].set_goal_position(self.degrees_to_value(Q[2]))

        return dxl_error

    # joints: np.array con los manejadores de las articulaciones a mover
    # radians: los valores de los radianes a mover
    def set_joints_position(self, joints, radians):
        for j, r in zip(joints, radians):
            j.set_goal_position(int(self.degrees_to_value(np.degrees(r))))
        time.sleep(self.__sleep)

        return r

    def get_joints_position_leg1(self):
        q = list(
            map(lambda motor: np.deg2rad(self.value_to_degrees(motor.get_present_position()[0])),
                self.__motores[self.__leg1]))
        return [-q[0], -q[1], -q[2]]

    def get_joints_position_leg2(self):
        q = list(
            map(lambda motor: np.deg2rad(self.value_to_degrees(motor.get_present_position()[0])),
                self.__motores[self.__leg2]))
        return [q[0], -q[1], -q[2]]

    def get_joints_position_leg3(self):
        q = list(
            map(lambda motor: np.deg2rad(self.value_to_degrees(motor.get_present_position()[0])),
                self.__motores[self.__leg3]))
        return [-q[0], -q[1], -q[2]]

    def get_joints_position_leg4(self):
        q = list(
            map(lambda motor: np.deg2rad(self.value_to_degrees(motor.get_present_position()[0])),
                self.__motores[self.__leg4]))
        return [q[0], -q[1], -q[2]]

    def move_leg(self, i, Q):
        if i == 1:
            # self.set_joints_position(self.__motores[self.__leg1], self.mask_joint_positions(Q, self.__leg1))
            Q = self.mask_joint_positions(Q, self.__leg1)
            Q = list(map(self.degrees_to_value, np.rad2deg(Q)))
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg1, Q)
        elif i == 2:
            # self.move_joints(self.__leg2, self.mask_joint_positions(Q, self.__leg2))
            Q = self.mask_joint_positions(Q, self.__leg2)
            Q = list(map(self.degrees_to_value, np.rad2deg(Q)))
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg2, Q)
        elif i == 3:
            # self.move_joints(self.__leg3, self.mask_joint_positions(Q, self.__leg3))
            Q = self.mask_joint_positions(Q, self.__leg3)
            Q = list(map(self.degrees_to_value, np.rad2deg(Q)))
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg3, Q)
        elif i == 4:
            # self.move_joints(self.__leg4, self.mask_joint_positions(Q, self.__leg4))
            Q = self.mask_joint_positions(Q, self.__leg4)
            Q = list(map(self.degrees_to_value, np.rad2deg(Q)))
            [ts1, ts2, imu_log, pos] = self.move_joints(self.__leg4, Q)
        return [ts1, ts2, imu_log, pos]

    def default_position(self):
        # Default position for all joints
        # list(map(lambda motor: motor.set_goal_position(self.degrees_to_value(0)), self.__motores))

        self.set_moving_speed(self.__low_speed)
        motores_position = self.move_joints(self.__all_joints, [self.degrees_to_value(0)] * 12)
        self.set_moving_speed(self.__high_speed)

        return motores_position

        """
        for i in self.__motores:
            thread = threading.Thread(target = i.set_goal_position, args = (512,))
            thread.start()
            time.sleep(1)

        """

        # for i in self.__motores:
        #    _thread.start_new_thread(i.set_goal_position,(512,))
        #    time.sleep(0.3)
        """
        processes = []
        for i in self.__motores:
            time.sleep(0.5)
            p = mp.Process(target=i.set_goal_position, args=(512,))
            processes.append(p)
            p.start()

        for process in processes:
            process.join()
        """

    # get the radians position of the joints
    def get_joints_position(self, joints_idx):

        q = list(
            map(lambda motor: np.deg2rad(self.value_to_degrees(motor.get_present_position()[0])),
                self.__motores[joints_idx]))
        pos = self.mask_joint_positions(q, joints_idx)

        return pos

    # adapts the sense rotation of the joints for the physical robot
    def mask_joint_positions(self, q, joints_idx):
        mask = self.__joint_pos_mask[joints_idx]
        pos = [x * y for x, y in zip(mask, q)]

        return pos



    def move_offset_coord(self, goal, present_positions = None):
        q_act = self.get_joints_position(self.get_all_joints_idx())
        q_act_masked = self.mask_joint_positions(q_act, self.__all_joints)
        present_positions = list(map(self.degrees_to_value, np.rad2deg(q_act_masked)))

        q_goal_masked = self.mask_joint_positions(goal, self.__all_joints)
        goal_positions = list(map(self.degrees_to_value, np.rad2deg(q_goal_masked)))

        # distancias a recorrer por cada articulación, en unidades del motor
        d = [abs(g - p) for g, p in zip(goal_positions, present_positions)]

        # velocidades a usar en cada articulación
        v = [round(r * self.__low_speed / max(d)) for r in d]

        # velocidad a 1 si tenemos un 0
        v = [1 if v_i == 0 else v_i for v_i in v]

        self.set_moving_speed_list(v, self.__all_joints)

        [self.ts1, self.ts2, self.sensors_log, self.pose] = self.move_joints_coord(self.__all_joints, goal_positions)

        # wait for completion
        # V (g/sg) = v * 0.111 r/1m * 1m/60sg * 360g/1r
        # t = d * 0.29 g / V g/sg = d / V * 0.43 sg.

        # primer elemento diferente de 0
        i = np.nonzero(np.array(d))[0][0]

        # tiempo de espera en segundos
        t = d[i] / v[i] * 0.43
        time.sleep(t * 0.8)

        return [self.ts1, self.ts2, self.sensors_log, self.pose, goal_positions, v, t]


    def exec_move(self, goal, v, t):
        self.set_moving_speed_list(v, self.__all_joints)
        [self.ts1, self.ts2, self.sensors_log, self.pose] = self.move_joints_coord(self.__all_joints, goal)
        time.sleep(t * 0.8)

        return [self.ts1, self.ts2, self.sensors_log, self.pose, goal, v, t]


    # q in radians
    def move_offset(self, q, wait=5, speed=__low_speed):
        # remaping the sense of the joints for the pyhiscal robot
        """
        q = [-q[0], -q[1], -q[2],
             q[3], -q[4], -q[5],
             -q[6], -q[7], -q[8],
             q[9], -q[10], -q[11]]
        """

        self.set_moving_speed(speed, self.__all_joints)

        q = self.mask_joint_positions(q, self.__all_joints)
        goal_positions = list(map(self.degrees_to_value, np.rad2deg(q)))

        [self.ts1, self.ts2, self.sensors_log, self.pose] = self.move_joints(self.__all_joints, goal_positions, wait)

        self.set_moving_speed(self.__high_speed)
        return [self.ts1, self.ts2, self.sensors_log, self.pose]

    # move synchronously the joints specified by joints_idx to the motor positions values specified by positions
    def move_joints(self, joints_idx, goal_pos, wait=5):
        ts1 = time.time()

        elements = len(joints_idx)
        if (elements != len(goal_pos)):
            return -1

        present_pos = list(map(lambda motor: motor.get_present_position()[0], self.__motores[joints_idx]))
        # grados entre posición actual y objetivo
        g = [abs(self.value_to_degrees(g) - self.value_to_degrees(p)) for g, p in zip(goal_pos, present_pos)]

        # velocidad de giro de los motores a mover
        v = list(map(lambda motor: motor.get_moving_speed()[0], self._Physical_robot__motores[joints_idx]))

        # tiempo de giro estimado según velocidad v y grados a recorred g
        t_out = [g / vj * 0.111 * 360 / 60 for vj, g in zip(v, g)]

        # Initialize GroupSyncWrite instance
        groupSyncWrite = GroupSyncWrite(self.__portHandler, self.__packetHandler, self.__ADDR_AX_GOAL_POSITION,
                                        self.__LEN_AX_GOAL_POSITION)

        # Adding the position for each joint
        for joint_idx, goal_position in zip(joints_idx, goal_pos):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position))]
            dxl_addparam_result = groupSyncWrite.addParam(self.__joints[joint_idx], param_goal_position)

            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed in joint" % self.__joints[joint_idx])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        if (wait > 0):
            # waits for completion of movement
            finish = [False] * elements

            t_ini = time.time()
            while finish.count(True) < elements:
                # gets the raw present position of the motors
                pos = list(map(lambda motor: motor.get_present_position()[0], self.__motores[joints_idx]))
                for i in range(elements):
                    if not ((abs(goal_pos[i] - pos[i]) > self.__DXL_MOVING_STATUS_THRESHOLD)):
                        finish[i] = True
                    else:
                        if time.time() - t_ini > t_out[i] * wait:
                            print("Possible Overload in joint ", joints_idx[i], "! correcting position")
                            # self._Physical_robot__motores[joints_idx[i]].set_max_torque(1023)
                            # self._Physical_robot__motores[joints_idx[i]].enable_torque()
                            self._Physical_robot__motores[joints_idx[i]].set_goal_position(pos[i])
                            finish[i] = True
                        else:
                            finish[i] = False

        pos = self.get_joints_position(joints_idx)
        IMUlog = self.__sensors.get_log()
        ts2 = time.time()

        return [ts1, ts2, IMUlog, pos]


    # move synchronously the joints specified by joints_idx to the motor positions values specified by positions
    def move_joints_coord(self, joints_idx, goal_pos):
        ts1 = time.time()

        # Initialize GroupSyncWrite instance
        groupSyncWrite = GroupSyncWrite(self.__portHandler, self.__packetHandler, self.__ADDR_AX_GOAL_POSITION,
                                            self.__LEN_AX_GOAL_POSITION)

        # Adding the position for each joint
        for joint_idx, goal_position in zip(joints_idx, goal_pos):
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(goal_position)), DXL_HIBYTE(DXL_LOWORD(goal_position))]
            dxl_addparam_result = groupSyncWrite.addParam(self.__joints[joint_idx], param_goal_position)

            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed in joint" % self.__joints[joint_idx])
                quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.__packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        pos = self.get_joints_position(joints_idx)
        IMUlog = self.__sensors.get_log()
        ts2 = time.time()

        return [ts1, ts2, IMUlog, pos]

    # move the joints specified by joints_idx the radians specified by Q
    def move_joints_radians(self, Q, joints_idx):
        self.move_joints(joints_idx, list(map(
            self.degrees_to_value, np.rad2deg(self.mask_joint_positions(Q, joints_idx)))
        ))
        return

    def get_fs(self):
        fs = self.__sensors.get_fsr()
        return fs

    def get_leg1_idx(self):
        return self.__leg1

    def get_leg2_idx(self):
        return self.__leg2

    def get_leg3_idx(self):
        return self.__leg3

    def get_leg4_idx(self):
        return self.__leg4

    def get_all_joints_idx(self):
        return self.__all_joints

    def get_pCoM(self):
        return [0, 0, 0]

    def set_CoP_position(self, pos):
        pass
        return

    def get_CoM_position_and_orientation(self):
        r = 0
        p_CoM = [0, 0, 0]  # [0,0,0] for the physical robot
        o_CoM = self.__sensors.readIMU()
        o_CoM = [o_CoM[0], o_CoM[1], 0]
        # o_CoM = [0, 0, 0]
        return r, p_CoM, o_CoM

    def desplaza_cadera(self, d):
        self.set_moving_speed(self.__low_speed)
        [ts1, ts2, imu_log, pose] = self.move_joints(self.__cadera, list(map(self.degrees_to_value, [d, d, -d, -d])))
        self.set_moving_speed(self.__high_speed)
        return [ts1, ts2, imu_log, pose]

    def desplaza_rodillas(self, d):
        self.set_moving_speed(self.__low_speed)
        [ts1, ts2, imu_log, pose] = self.move_joints(self.__rodillas, list(map(self.degrees_to_value, [-d, d, d, -d])))
        self.set_moving_speed(self.__high_speed)
        return [ts1, ts2, imu_log, pose]

    def desplaza_pies(self, d):
        self.set_moving_speed(self.__low_speed)
        [ts1, ts2, imu_log, pose] = self.move_joints(self.__pies, list(map(self.degrees_to_value, [-d, d, d, -d])))
        self.set_moving_speed(self.__high_speed)
        return [ts1, ts2, imu_log, pose]

    # Desplaza las rodillas sumando d grados a las posiciones KD (Knees Degrees)
    def knees_offset(self, KD, d):
        self.set_moving_speed(self.__low_speed)
        d = np.float(d)
        Q = [KD[0] + d, KD[1] - d, KD[2] - d, KD[3] + d]
        self.move_joints(self.get_knees_idx(), list(map(
            self.degrees_to_value, self.mask_joint_positions(Q, self.get_knees_idx()))
        ))
        self.set_moving_speed(self.__high_speed)
        return

    def knees_hip_offset_old(self, QD, KD, Koff, HD, Hoff):
        Koff = np.float(Koff)
        Qk = [KD[0] + Koff, KD[1] - Koff, KD[2] - Koff, KD[3] + Koff]
        for i, v in zip(self.get_knees_idx(), Qk):
            QD[i] = v

        Hoff = np.float(Hoff)
        Qh = [HD[0] - Hoff, HD[1] + Hoff, HD[2] + Hoff, HD[3] - Hoff]
        for i, v in zip(self.get_hip_idx(), Qh):
            QD[i] = v

        self.set_moving_speed(self.__low_speed, self.get_all_joints_idx())

        self.move_joints(self.get_all_joints_idx(), list(map(
            self.degrees_to_value, self.mask_joint_positions(QD, self.get_all_joints_idx()))
        ))
        self.set_moving_speed(self.__high_speed, self.get_all_joints_idx())

        return

    def knees_hip_offset(self, KD, Koff, HD, Hoff):
        ts1 = time.time()

        Koff = np.float(Koff)
        Qk = [KD[0] + Koff, KD[1] - Koff, KD[2] - Koff, KD[3] + Koff]

        Hoff = np.float(Hoff)
        Qh = [HD[0] - Hoff, HD[1] + Hoff, HD[2] + Hoff, HD[3] - Hoff]

        self.set_moving_speed(self.get_low_speed(), self.get_knees_idx() + self.get_hip_idx())

        self.move_joints(self.get_knees_idx() + self.get_hip_idx(), list(map(
            self.degrees_to_value, self.mask_joint_positions(Qk + Qh, self.get_knees_idx() + self.get_hip_idx()))
        ))
        self.set_moving_speed(self.get_high_speed(), self.get_knees_idx() + self.get_hip_idx())

        pos = self.get_joints_position(self.get_all_joints_idx())
        sensors_log = self.__sensors.get_log()
        ts2 = time.time()

        return [ts1, ts2, sensors_log, pos]

    # Desplaza las rodillas sumando d grados a las posiciones KD (Knees Degrees)
    def hip_offset(self, HD, d):
        self.set_moving_speed(self.__low_speed)
        d = np.float(d)
        Q = [HD[0] - d, HD[1] + d, HD[2] + d, HD[3] - d]
        self.move_joints(self.get_hip_idx(), list(map(
            self.degrees_to_value, self.mask_joint_positions(Q, self.get_hip_idx()))
        ))
        self.set_moving_speed(self.__high_speed)
        return

    def get_joints_handle(self, idx):
        return self.__motores[idx]

    def get_hip_idx(self):
        return self.__cadera

    def get_knees_idx(self):
        return self.__rodillas

    def get_feet_idx(self):
        return self.__pies

    # adapts Qi radians to joints values: rad -> degrees -> engine value
    def set_Q_goal(self, Qi):
        Qi_goal = list(map(self.degrees_to_value, np.rad2deg(Qi)))
        return Qi_goal

    def get_low_speed(self):
        return self.__low_speed

    def get_high_speed(self):
        return self.__high_speed

    def get_leg_idx(self, n):
        if (n == 1):
            return self.__leg1
        if (n == 2):
            return self.__leg2
        if (n == 3):
            return self.__leg3
        if (n == 4):
            return self.__leg4

    def sensors_log(self):
        return self.__sensors.get_log()
