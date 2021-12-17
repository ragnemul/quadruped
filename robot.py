import time

import numpy as np
import kinematics as kinematics
from interpolation import Interpolation
from sympy.geometry import *
#import matplotlib.pyplot as plt
from sympy import *
from sympy.abc import x

class AllObf:
    def __init__(self, elements):
        self.elements = elements

    def __getattr__(self, attr):
        def on_all(*args, **kwargs):
            for obj in self.elements:
                getattr(obj, attr)(*args, **kwargs)

        return on_all


# from CoppeliaSim_robot import CoppeliaSim
# class Robot(CoppeliaSim):
from physical_robot import Physical_robot
class Robot (Physical_robot):

    __Qi = None
    ts1 = None
    ts2 = None
    sensors_log = None
    pose = None
    __IK_exception = False
    __interpolation = None

    # Default device name when running from MAC using D2U2 is device = '/dev/tty.usbserial-FT4TFQEI'
    # When running from the RPI with the D2AUX, the device is '/dev/ttyACM0'
    def __init__(self, Q):
        super().__init__()
        kinematics.__exception = False
        self.__Qi = Q
        self.__interpolation = Interpolation()

    # Deleting the Robot object
    def __del__(self):
        print ("Deleting robot class!\n")
        super().__del__()
        print("Robot class stopped!\n")

    def __raise_exception(self):
        self.__IK_exception = True
        return

    def clear_exception(self):
        self.__IK_exception = False
        kinematics.clear_exception()
        return

    def excp_cond(self):
        return self.__IK_exception


    def Qi(self):
        return self.__Qi

    # Obtiene los ángulos de giro para cada articulación considerando que todos los efectores se mueven el offset
    # especificado por x,y,z
    def offset4(self, Off):

        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Q = self.get_joints_position(self.get_all_joints_idx())
        [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4] = kinematics.FK4(p_CoM, o_CoM, Q)

        P = [-p_CoM[0] + x1 + Off[0], -p_CoM[1] + y1 + Off[1], -p_CoM[2] + z1 + Off[2],
             -p_CoM[0] + x2 + Off[3], -p_CoM[1] + y2 + Off[4], -p_CoM[2] + z2 + Off[5],
             -p_CoM[0] + x3 + Off[6], -p_CoM[1] + y3 + Off[7], -p_CoM[2] + z3 + Off[8],
             -p_CoM[0] + x4 + Off[9], -p_CoM[1] + y4 + Off[10], -p_CoM[2] + z4 + Off[11]]

        Q = kinematics.IK4(o_CoM, P)
        return Q







    # Los cambios con respecto a move son los siguientes:
    # La pata que se está moviendo solo baja su posición en el útimo tramo, en lugar de mover hacia delante y abajo
    def move_3241(self, n):
        # vector of movements
        Q = []

        # posición inicial
        #self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03)
        Qr0 = self.get_joints_position(self.get_all_joints_idx())
        Q.append(Qr0)

        up = 0.05
        x_off = 0.04
        x_off = 0.06

        # Pata 3
        # self.desplaza_cadera(30)
        #self.move_offset(self.offset4([-0.06, 0, 0] * 4))
        Qt1 = self.offset4([-x_off, 0, 0] + [-x_off, 0, 0.02] + [-x_off, 0, 0]*2)
        #Qt1 = self.offset4([-x_off, 0, 0] + [-x_off, 0, 0] + [-x_off, 0, 0] * 2)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [3, 0.01, up, -0.02]

        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 2
        # Se prueba a ver si este despazamiento de rodillas hacia Y positiva
        # hace menos forzada la posición de la pata 2 cuando se esté movimiento la pata 1
        # ---
        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        self.knees_hip_offset(QdKnees, 10, QdHip, 0)
        Qt1 = self.get_joints_position(self.get_all_joints_idx())
        Q.append(Qt1)
        # ---

        [leg, y, z1, z2] = [2, 0.01, up + 0.01, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 4
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, -5, QdHip, -60)

        # self.desplaza_cadera(-30)
        Qt1 = self.offset4([x_off*2, 0, 0.02] + [x_off*2, 0, 0] * 3)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [4, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 1
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, 10, QdHip, -10)

        [leg, y, z1, z2] = [1, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        self.move_offset(Qr0)
        Q.append(Qr0)

        for i in range(n-1):
            for Qi in Q:
                #self.move_offset(Qi, 0, 75)
                self.move_offset(Qi, 5, 75)


        time.sleep(1)
        return

        # Los cambios con respecto a move son los siguientes:
        # La pata que se está moviendo solo baja su posición en el útimo tramo, en lugar de mover hacia delante y abajo


    def get_offset_legs24 (self, xi, yi, zi, xf, yf, zf):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Qi = (self.get_joints_position(self.get_all_joints_idx()))

        [x1_i_off, y1_i_off, z1_i_off] = [xi, yi, zi]
        [x3_i_off, y3_i_off, z3_i_off] = [xi, yi, zi]

        [x1_f_off, y1_f_off, z1_f_off] = [xf, yf, zf]
        [x3_f_off, y3_f_off, z3_f_off] = [xf, yf, zf]

        # first offset calc.
        [x1i, y1i, z1i] = kinematics.FK(2, p_CoM, o_CoM,[ Qi[3], Qi[4], Qi[5]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0i, q1i, q2i] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1i + x1_i_off, -p_CoM[1] + y1i + y1_i_off, -p_CoM[2] + z1i + z1_i_off])

        [x3i, y3i, z3i] = kinematics.FK(4, p_CoM, o_CoM, [Qi[9], Qi[10], Qi[11]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6i, q7i, q8i] = kinematics.IK(4, o_CoM, [-p_CoM[0] + x3i + x3_i_off, -p_CoM[1] + y3i + y3_i_off, -p_CoM[2] + z3i + z3_i_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # First offset to (xi,yi,zi)
        # Qi[3:6] = [q0i, q1i, q2i]
        # Qi[9:12] = [q6i, q7i, q8i]



        # second offset calc.
        [x1f, y1f, z1f] = kinematics.FK(2, p_CoM, o_CoM, [q0i, q1i, q2i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0f, q1f, q2f] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1f + x1_f_off, -p_CoM[1] + y1f + y1_f_off, -p_CoM[2] + z1f + z1_f_off])

        [x3f, y3f, z3f] = kinematics.FK(4, p_CoM, o_CoM, [q6i, q7i, q8i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6f, q7f, q8f] = kinematics.IK(4, o_CoM, [-p_CoM[0] + x3f + x3_f_off, -p_CoM[1] + y3f + y3_f_off, -p_CoM[2] + z3f + z3_f_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # Second offset to (xi,yi,zi)
        # Qf[3:6] = [q0f, q1f, q2f]
        # Qf[9:12] = [q6f, q7f, q8f]

        # Do the movements
        idx = self.get_leg2_idx().copy()[0:3]
        idx.extend(self.get_leg4_idx()[0:3])

        return [idx, [q0i, q1i, q2i] + [q6i, q7i, q8i], [q0f, q1f, q2f] + [q6f, q7f, q8f]]



    def trot(self, n):
        # vector of movements
        Q = []

        # posición inicial
        # self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03)
        Qr0 = self.get_joints_position(self.get_all_joints_idx())
        Q.append(Qr0)

        up = 0.05
        x_off = 0.04
        x_off = 0.06

        Qt0 = self.offset4([0, 0, 0]*4)
        self.move_offset(Qt0)

        Q1 = self.offset4([0,0.02,-0.02] + [0,-0.02, 0.02] + [0, 0.02, -0.02] + [0,-0.02,0.02])
        self.move_offset(Q1)
        Q2 = self.offset4([0, 0.02, 0] + [0, -0.02, 0] + [0, 0.02, 0] + [0, -0.02, 0])
        self.move_offset(Q2)
        Q4 = self.offset4([0,0.02,+0.02]+[0,-0.02,-0.02]+[0,0.02,+0.02]+[0,-0.02,-0.02])

        Qt0 = self.offset4([0, -0.04, 0] + [0, 0, 0] + [0, -0.04, 0] + [0, 0, 0])
        self.move_offset(Qt0)

        # i_goal: primer movimiento (hacia arriba) de patas 2 y 4
        # f_goal: segundo movimiento (hacia abajo) de paras 2 y 4
        # idx: indice de los elementos de las patas 2 y 4
        [leg1, leg2, leg3, leg4, y, z1, z2] = [1, 2, 3, 4, 0.02, -0.02, 0.02]
        [leg1, leg2, leg3, leg4, y, z1, z2] = [1, 2, 3, 4, 0.02, -0.02, 0.02]

        # levanto patas 2 y 4
        idx, i_goal, f_goal = self.get_offset_legs24(0, -y*2, 0.03, 0, +y, -0.03)
        # Desplazamiento de avance de 1 y 3
        Qt1 = Qt2 = self.offset4([0, 0, 0] * 4)
        # Se inserta el levantamiento de 1 y 3
        Qt1[self.get_leg_idx(leg2)[0]:self.get_leg_idx(leg2)[2] + 1] = i_goal[0:3]
        Qt1[self.get_leg_idx(leg4)[0]:self.get_leg_idx(leg4)[2] + 1] = i_goal[3:6]
        self.move_offset(Qt1, 0, 0)

        Qt2 = self.offset4([0, y*4, z1] + [0, 0, 0] + [0, y*4, z1] + [0, 0, 0])
        self.move_offset(Qt2, 0, 512)

        Qt3 = self.offset4([0, y, z2] + [0, 0, 0] + [0, y, z2] + [0, 0, 0])
        Qt3[self.get_leg_idx(leg2)[0]:self.get_leg_idx(leg2)[2] + 1] = f_goal[0:3]
        Qt3[self.get_leg_idx(leg4)[0]:self.get_leg_idx(leg4)[2] + 1] = f_goal[3:6]
        self.move_offset(Qt3, 0, 256)

        # Al poner la siguiente linea despues de aplicar Qt1, se mueve
        #Qt2 = self.offset4([0, 0.02, z2] + [0, 0, 0] + [0, 0.02, z2] + [0, 0, 0])




        wait = False
        self.set_moving_speed(512)
        [ts1, ts2, imu_log_up, pos_up] = self.move_joints(idx, i_goal, wait)
        [ts3, ts4, imu_log_down, pos_down] = self.move_joints(idx, f_goal, wait)
        self.set_moving_speed(self.get_low_speed())





        # Pata 3
        # self.desplaza_cadera(30)
        # self.move_offset(self.offset4([-0.06, 0, 0] * 4))
        Qt1 = self.offset4([-x_off, 0, 0] + [-x_off, 0, 0.02] + [-x_off, 0, 0] * 2)
        # Qt1 = self.offset4([-x_off, 0, 0] + [-x_off, 0, 0] + [-x_off, 0, 0] * 2)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [3, 0.01, up, -0.02]

        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y * 2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 2
        # Se prueba a ver si este despazamiento de rodillas hacia Y positiva
        # hace menos forzada la posición de la pata 2 cuando se esté movimiento la pata 1
        # ---
        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        self.knees_hip_offset(QdKnees, 10, QdHip, 0)
        Qt1 = self.get_joints_position(self.get_all_joints_idx())
        Q.append(Qt1)
        # ---

        [leg, y, z1, z2] = [2, 0.01, up + 0.01, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y * 2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 4
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, -5, QdHip, -60)

        # self.desplaza_cadera(-30)
        Qt1 = self.offset4([x_off * 2, 0, 0.02] + [x_off * 2, 0, 0] * 3)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [4, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y * 2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 1
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, 10, QdHip, -10)

        [leg, y, z1, z2] = [1, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y * 2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        self.move_offset(Qr0)
        Q.append(Qr0)

        for i in range(n - 1):
            for Qi in Q:
                # self.move_offset(Qi, 0, 75)
                self.move_offset(Qi, 5, 75)

        time.sleep(1)
        return


    # seguir diagrama 5.33 e
    # desplazo hacia la derecha desde posición inicial
    # aplico tiempo t3: subo 1, desplazo el resto y poso 1
    # luego se mueve 4
    # desplazo hacia la izquierda
    # levanto 2 ...
    # levante 3...
    def move_1324(self, n):
        Q = []
        Qi = self.get_joints_position(self.get_all_joints_idx())
        z = 0.05

        # posición inicial
        Q0 = self.offset4([0,-0.02,0] + [0,0.04,0] + [0,0.06,0] + [0,0.02,0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q0)

        # Pata 3
        # t0-t1
        Q11 = self.offset4([0, 0.01, 0] + [0, 0.01, 0] + [0, -0.01, z+0.02] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q11)
        Q12 = self.offset4([0, 0.01, 0] + [0, 0.01, 0] + [0, -0.01, -z-0.02] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q12)

        # self.CoP_offset(2)
        [ts1, ts2, imu_log_Q, pos_Q] = self.offset_knees_and_hip(20, 30)

        # Pata 2
        # t1-t2
        Q21 = self.offset4([0, 0.01, 0] + [0, -0.01, z+0.02] + [0, 0.01, 0] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q21)
        Q22 = self.offset4([0, 0.01, 0] + [0, -0.01, -z-0.02] + [0, 0.01, 0] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q22)

        [ts1, ts2, imu_log_Q, pos_Q] = self.offset_knees_and_hip(-20, -30)
        [ts1, ts2, imu_log_Q, pos_Q] = self.offset_knees_and_hip(0, -30)

        # Pata 4
        # t2-t3
        Q31 = self.offset4([0, 0.01, 0] + [0, 0.01, 0] + [0, 0.01, 0] + [0, -0.01, z])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q31)
        Q32 = self.offset4([0, 0.01, 0] + [0, 0.01, 0] + [0, 0.01, 0] + [0, -0.01, -z])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q32)

        Qi = self.get_joints_position(self.get_all_joints_idx())
        self.offset_knees_and_hip(0, 30)

        # Pata 1
        # t3-t4
        Q41 = self.offset4([0, -0.01, z] + [0, 0.01, 0] + [0, 0.01, 0] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q41)
        Q42 = self.offset4([0, -0.01, -z] + [0, 0.01, 0] + [0, 0.01, 0] + [0, 0.01, 0])
        [ts1, ts2, imu_log_Q, pos_Q, goal_positions, v, t] = self.move_offset_coord(Q42)

        return


    # Los cambios con respecto a move son los siguientes:
    # La pata que se está moviendo solo baja su posición en el útimo tramo, en lugar de mover hacia delante y abajo

    # HAY QUE DEPURARLO!!!!

    def move_1423(self, n):
        # vector of movements
        Q = []

        # posición inicial
        #self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03)
        Qr0 = self.get_joints_position(self.get_all_joints_idx())
        Q.append(Qr0)

        up = 0.05
        x_off = 0.04
        x_off = 0.06

        # Pata 1
        # self.desplaza_cadera(30)
        #self.move_offset(self.offset4([-0.06, 0, 0] * 4))
        Qt1 = self.offset4([x_off, 0, 0] + [x_off, 0, 0.0] + [x_off, 0, 0]*2)
        #Qt1 = self.offset4([-x_off, 0, 0] + [-x_off, 0, 0] + [-x_off, 0, 0] * 2)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [1, 0.01, up, -0.02]

        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 4
        # Se prueba a ver si este despazamiento de rodillas hacia Y positiva
        # hace menos forzada la posición de la pata 2 cuando se esté movimiento la pata 1
        # ---
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(QdKnees, 10, QdHip, 0)
        # Qt1 = self.get_joints_position(self.get_all_joints_idx())
        # Q.append(Qt1)
        # ---

        [leg, y, z1, z2] = [4, 0.01, up + 0.01, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 2
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, -5, QdHip, -60)

        # self.desplaza_cadera(-30)
        #Qt1 = self.offset4([-x_off*2, 0, 0] + [-x_off*2, 0, 0.02] + [-x_off*2, 0, 0] * 2)
        Qt1 = self.offset4([-0.09, 0, 0] + [-0.09, 0, 0.02] + [-0.09, 0, 0] * 2)
        Q.append(Qt1)
        self.move_offset(Qt1)
        [leg, y, z1, z2] = [2, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        # Pata 3
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, 10, QdHip, -10)

        [leg, y, z1, z2] = [3, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y*2, z1]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -0, z2]
        Qt1 = self.offset4(Off)
        Q.append(Qt1)
        self.move_offset(Qt1)

        self.move_offset(Qr0)
        Q.append(Qr0)

        for i in range(n-1):
            for Qi in Q:
                #self.move_offset(Qi, 0, 75)
                self.move_offset(Qi, 5, 75)


        time.sleep(1)
        return




    def move(self):

        # posición inicial
        #self.offset_leg13(0, -0.04, 0.03, 0, 0, -0.03)
        Qr0 = self.get_joints_position(self.get_all_joints_idx())

        up = 0.05

        # Pata 3
        #self.desplaza_cadera(30)
        self.move_offset(self.offset4([-0.06,0,0]*4))
        [leg, y, z1, z2] = [3, 0.01, up, -0.02]
        Off = [0, y, 0]*4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z1]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z2]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        # Pata 2
        # Se prueba a ver si este despazamiento de rodillas hacia Y positiva
        # hace menos forzada la posición de la pata 2 cuando se esté movimiento la pata 1
        # ---
        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        self.knees_hip_offset(QdKnees, 10, QdHip, 0)
        # ---

        [leg, y, z1, z2] = [2, 0.01, up+0.01, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z1]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z2]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)


        # Pata 4
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, -5, QdHip, -60)

        #self.desplaza_cadera(-30)
        self.move_offset(self.offset4([0.12,0,0.02]+[0.12,0,0]*3))
        [leg, y, z1, z2] = [4, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z1]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z2]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        # Pata 1
        # Qr = self.get_joints_position(self.get_all_joints_idx())
        # Qd = np.rad2deg(Qr)
        # QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        # QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        # self.knees_hip_offset(Qd, QdKnees, 10, QdHip, -10)

        [leg, y, z1, z2] = [1, 0.01, up, -0.02]
        Off = [0, y, 0] * 4
        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z1]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        Off[self.get_leg_idx(leg)[0]:self.get_leg_idx(leg)[2] + 1] = [0, -y, z2]
        Qt1 = self.offset4(Off)
        self.move_offset(Qt1)

        self.move_offset(Qr0)

        return



    def offset(self, x, y, z):
        o_CoM = [0, 0, 0]

        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Q = self.get_joints_position(self.get_all_joints_idx())

        # leg1
        motores_position = self.get_joints_position_leg1()
        [x1, y1, z1] = kinematics.FK(1, p_CoM, o_CoM,[Q[0], Q[1], Q[2]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q1, q2, q3] = kinematics.IK(1, o_CoM,[-p_CoM[0] + x1 + x, -p_CoM[1] + y1 + y, -p_CoM[2] + z1 + z])

        # leg2
        motores_position = self.get_joints_position_leg2()
        [x2, y2, z2] = kinematics.FK(2, p_CoM, o_CoM, [Q[3], Q[4], Q[5]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q4, q5, q6] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x2 + x, -p_CoM[1] + y2 + y, -p_CoM[2] + z2 + z])

        # leg3
        motores_position = self.get_joints_position_leg3()
        [x3, y3, z3] = kinematics.FK(3, p_CoM, o_CoM,[Q[6], Q[7], Q[8]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q7, q8, q9] = kinematics.IK(3, o_CoM, [-p_CoM[0] + x3 + x, -p_CoM[1] + y3 + y, -p_CoM[2] + z3 + z])

        # leg4
        motores_position = self.get_joints_position_leg4()
        [x4, y4, z4] = kinematics.FK(4, p_CoM, o_CoM,[Q[9], Q[10], Q[11]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q10, q11, q12] = kinematics.IK(4, o_CoM,[-p_CoM[0] + x4 + x, -p_CoM[1] + y4 + y, -p_CoM[2] + z4 + z])

        if (kinematics.excp_cond() != True):
            self.move_offset([q1, q2, q3,
                              q4, q5, q6,
                              q7, q8, q9,
                              q10, q11, q12])
        else:
            print ("Unable to apply IK")
        return


    def offset_leg23 (self, xi, yi, zi, xf, yf, zf):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Qi = (self.get_joints_position(self.get_all_joints_idx()))

        # first offset calc.
        [x1i, y1i, z1i] = kinematics.FK(2, p_CoM, o_CoM,[ Qi[3], Qi[4], Qi[5]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0i, q1i, q2i] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1i + xi, -p_CoM[1] + y1i + yi, -p_CoM[2] + z1i + zi])

        [x3i, y3i, z3i] = kinematics.FK(3, p_CoM, o_CoM, [Qi[6], Qi[7], Qi[8]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6i, q7i, q8i] = kinematics.IK(3, o_CoM, [-p_CoM[0] + x3i + xi, -p_CoM[1] + y3i + yi, -p_CoM[2] + z3i + zi])

        # First offset to (xi,yi,zi)
        Qi[3:6] = [q0i, q1i, q2i]
        Qi[6:9] = [q6i, q7i, q8i]

        Qf = Qi.copy()

        Qi = self.mask_joint_positions(Qi, self.get_all_joints_idx())
        Qi_goal = self.set_Q_goal(Qi)


        # second offset calc.

        [x1f, y1f, z1f] = kinematics.FK(2, p_CoM, o_CoM, [q0i, q1i, q2i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0f, q1f, q2f] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1f + xf, -p_CoM[1] + y1f + yf, -p_CoM[2] + z1f + zf])

        [x3f, y3f, z3f] = kinematics.FK(3, p_CoM, o_CoM, [q6i, q7i, q8i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6f, q7f, q8f] = kinematics.IK(3, o_CoM, [-p_CoM[0] + x3f + xf, -p_CoM[1] + y3f + yf, -p_CoM[2] + z3f + zf])

        # Second offset to (xi,yi,zi)
        Qf[3:6] = [q0f, q1f, q2f]
        Qf[6:9] = [q6f, q7f, q8f]

        Qf = self.mask_joint_positions(Qf, self.get_all_joints_idx())
        Qf_goal = self.set_Q_goal(Qf)

        # Do the movements
        idx = self.get_leg1_idx().copy()[1:3]
        idx.extend(self.get_leg3_idx()[1:3])

        i_goal = Qi_goal[0:3].copy()[1:3]
        i_goal.extend(Qi_goal[6:9][1:3])

        f_goal = Qf_goal[0:3].copy()[1:3]
        f_goal.extend(Qf_goal[6:9][1:3])

        self.set_moving_speed(512)
        pos = self.move_joints(idx, i_goal)
        pos = self.move_joints(idx, f_goal)
        self.set_moving_speed(self.get_low_speed())
        return pos



    def offset_legs13 (self, xi, yi, zi, xf, yf, zf, wait=0, same=True):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Qi = (self.get_joints_position(self.get_all_joints_idx()))

        # Same direction for all the implied joints
        if (same):
            [x1_i_off, y1_i_off, z1_i_off] = [xi, yi, zi]
            [x3_i_off, y3_i_off, z3_i_off] = [xi, yi, zi]

            [x1_f_off, y1_f_off, z1_f_off] = [xf, yf, zf]
            [x3_f_off, y3_f_off, z3_f_off] = [xf, yf, zf]
        # Opposite direction for the implied joints
        else:
            [x1_i_off, y1_i_off, z1_i_off] = [xi, yi, zi]
            [x3_i_off, y3_i_off, z3_i_off] = [-xi, -yi, zi]

            [x1_f_off, y1_f_off, z1_f_off] = [xf, yf, zf]
            [x3_f_off, y3_f_off, z3_f_off] = [-xf, -yf, zf]


        # first offset calc.
        [x1i, y1i, z1i] = kinematics.FK(1, p_CoM, o_CoM,[ Qi[0], Qi[1], Qi[2]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0i, q1i, q2i] = kinematics.IK(1, o_CoM, [-p_CoM[0] + x1i + x1_i_off, -p_CoM[1] + y1i + y1_i_off, -p_CoM[2] + z1i + z1_i_off])

        [x3i, y3i, z3i] = kinematics.FK(3, p_CoM, o_CoM, [Qi[6], Qi[7], Qi[8]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6i, q7i, q8i] = kinematics.IK(3, o_CoM, [-p_CoM[0] + x3i + x3_i_off, -p_CoM[1] + y3i + y3_i_off, -p_CoM[2] + z3i + z3_i_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # First offset to (xi,yi,zi)
        Qi[0:3] = [q0i, q1i, q2i]
        Qi[6:9] = [q6i, q7i, q8i]

        Qf = Qi.copy()

        Qi = self.mask_joint_positions(Qi, self.get_all_joints_idx())
        Qi_goal = self.set_Q_goal(Qi)


        # second offset calc.

        [x1f, y1f, z1f] = kinematics.FK(1, p_CoM, o_CoM, [q0i, q1i, q2i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0f, q1f, q2f] = kinematics.IK(1, o_CoM, [-p_CoM[0] + x1f + x1_f_off, -p_CoM[1] + y1f + y1_f_off, -p_CoM[2] + z1f + z1_f_off])

        [x3f, y3f, z3f] = kinematics.FK(3, p_CoM, o_CoM, [q6i, q7i, q8i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6f, q7f, q8f] = kinematics.IK(3, o_CoM, [-p_CoM[0] + x3f + x3_f_off, -p_CoM[1] + y3f + y3_f_off, -p_CoM[2] + z3f + z3_f_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # Second offset to (xi,yi,zi)
        Qf[0:3] = [q0f, q1f, q2f]
        Qf[6:9] = [q6f, q7f, q8f]

        Qf = self.mask_joint_positions(Qf, self.get_all_joints_idx())
        Qf_goal = self.set_Q_goal(Qf)

        # Do the movements
        idx = self.get_leg1_idx().copy()[0:3]
        idx.extend(self.get_leg3_idx()[0:3])

        i_goal = Qi_goal[0:3].copy()[0:3]
        i_goal.extend(Qi_goal[6:9][0:3])

        f_goal = Qf_goal[0:3].copy()[0:3]
        f_goal.extend(Qf_goal[6:9][0:3])

        self.set_moving_speed(512)
        [ts1, ts2, imu_log_up, pos_up] = self.move_joints(idx, i_goal, wait)
        [ts3, ts4, imu_log_down, pos_down] = self.move_joints(idx, f_goal, wait)
        self.set_moving_speed(self.get_low_speed())

        [self.ts1, self.ts2, self.sensors_log, self.pose] = [ts1, ts4, imu_log_up + imu_log_down, pos_down]
        return [ts1, ts4, imu_log_up + imu_log_down, pos_down]


    def offset_legs24 (self, xi, yi, zi, xf, yf, zf, wait=0, same=True):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        Qi = (self.get_joints_position(self.get_all_joints_idx()))

        if (same):
            [x1_i_off, y1_i_off, z1_i_off] = [xi, yi, zi]
            [x3_i_off, y3_i_off, z3_i_off] = [xi, yi, zi]

            [x1_f_off, y1_f_off, z1_f_off] = [xf, yf, zf]
            [x3_f_off, y3_f_off, z3_f_off] = [xf, yf, zf]
        # Opposite direction for the implied joints
        else:
            [x1_i_off, y1_i_off, z1_i_off] = [xi, yi, zi]
            [x3_i_off, y3_i_off, z3_i_off] = [-xi, -yi, zi]

            [x1_f_off, y1_f_off, z1_f_off] = [xf, yf, zf]
            [x3_f_off, y3_f_off, z3_f_off] = [-xf, -yf, zf]

        # first offset calc.
        [x1i, y1i, z1i] = kinematics.FK(2, p_CoM, o_CoM,[ Qi[3], Qi[4], Qi[5]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0i, q1i, q2i] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1i + x1_i_off, -p_CoM[1] + y1i + y1_i_off, -p_CoM[2] + z1i + z1_i_off])

        [x3i, y3i, z3i] = kinematics.FK(4, p_CoM, o_CoM, [Qi[9], Qi[10], Qi[11]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6i, q7i, q8i] = kinematics.IK(4, o_CoM, [-p_CoM[0] + x3i + x3_i_off, -p_CoM[1] + y3i + y3_i_off, -p_CoM[2] + z3i + z3_i_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # First offset to (xi,yi,zi)
        Qi[3:6] = [q0i, q1i, q2i]
        Qi[9:12] = [q6i, q7i, q8i]

        Qf = Qi.copy()

        Qi = self.mask_joint_positions(Qi, self.get_all_joints_idx())
        Qi_goal = self.set_Q_goal(Qi)


        # second offset calc.

        [x1f, y1f, z1f] = kinematics.FK(2, p_CoM, o_CoM, [q0i, q1i, q2i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q0f, q1f, q2f] = kinematics.IK(2, o_CoM, [-p_CoM[0] + x1f + x1_f_off, -p_CoM[1] + y1f + y1_f_off, -p_CoM[2] + z1f + z1_f_off])

        [x3f, y3f, z3f] = kinematics.FK(4, p_CoM, o_CoM, [q6i, q7i, q8i])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q6f, q7f, q8f] = kinematics.IK(4, o_CoM, [-p_CoM[0] + x3f + x3_f_off, -p_CoM[1] + y3f + y3_f_off, -p_CoM[2] + z3f + z3_f_off])

        if (kinematics.excp_cond()):
            self.__raise_exception()
            return [None]*4

        # Second offset to (xi,yi,zi)
        Qf[3:6] = [q0f, q1f, q2f]
        Qf[9:12] = [q6f, q7f, q8f]

        Qf = self.mask_joint_positions(Qf, self.get_all_joints_idx())
        Qf_goal = self.set_Q_goal(Qf)

        # Do the movements
        idx = self.get_leg2_idx().copy()[0:3]
        idx.extend(self.get_leg4_idx()[0:3])

        i_goal = Qi_goal[3:6].copy()[0:3]
        i_goal.extend(Qi_goal[9:12][0:3])

        f_goal = Qf_goal[3:6].copy()[0:3]
        f_goal.extend(Qf_goal[9:12][0:3])

        """pos = self.move_joints(self.get_all_joints_idx(), Qi_goal)
        pos = self.move_joints(self.get_all_joints_idx(), Qf_goal)
        """


        # idx_fw = [1,7]                                              # idx de las rodillas leg1 & leg3
        # Qi = np.rad2deg(self.get_joints_position(idx_fw))           # posición de las rodillas leg1 & leg3
        # d = -10
        # d = np.float(d)
        #
        # # desplazamiento, máscara de posición y conversión a valores de motor
        # Q = list(map(self.degrees_to_value, self.mask_joint_positions([Qi[0] + d, Qi[1] - d], idx_fw)))
        #
        # idx_igoal_knees = idx.copy()
        # idx_igoal_knees.extend(idx_fw)          # se añaden los idx de las rodillas de leg1 & 3
        # i_goal.extend(Q)        # se añaden los valores de avance de las rodillas de leg1 & 3
        #
        # self.set_moving_speed(512)
        # pos = self.move_joints(idx_igoal_knees, i_goal, wait)
        # pos = self.move_joints(idx, f_goal, wait)
        # self.set_moving_speed(self.get_low_speed())

        self.set_moving_speed(512)
        [ts1, ts2, imu_log_up, pos_up] = self.move_joints(idx, i_goal, wait)
        [ts3, ts4, imu_log_down, pos_down] = self.move_joints(idx, f_goal, wait)
        self.set_moving_speed(self.get_low_speed())

        [self.ts1, self.ts2, self.sensors_log, self.pose] = [ts1, ts4, imu_log_up + imu_log_down, pos_down]
        return [ts1, ts4, imu_log_up + imu_log_down, pos_down]


    def offset_leg(self, leg, x, y, z):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        if (leg == 1):
            motores_position = self.get_joints_position(self.get_leg1_idx())
        elif (leg == 2):
            motores_position = self.get_joints_position(self.get_leg2_idx())
        elif (leg == 3):
            motores_position = self.get_joints_position(self.get_leg3_idx())
        elif (leg == 4):
            motores_position = self.get_joints_position(self.get_leg4_idx())

        [x1, y1, z1] = kinematics.FK(leg, p_CoM, o_CoM,[motores_position[0], motores_position[1], motores_position[2]])
        # [x1,y1,H+z1], con H la altura. Estando robot_v2 con CoM a 0.25 en z, y con la simulación parada.
        [q1, q2, q3] = kinematics.IK(leg, o_CoM, [-p_CoM[0] + x1 + x, -p_CoM[1] + y1 + y, -p_CoM[2] + z1 + z])

        [self.ts1, self.ts2, self.sensors_log, self.pose] = self.move_leg(leg, [q1, q2, q3])
        return  [self.ts1, self.ts2, self.sensors_log, self.pose]


    def get_foot_position(self, p_CoM, o_CoM, leg, joints_idx):
        q = self.get_joints_position(joints_idx)
        p = kinematics.FK(leg, p_CoM, o_CoM, [q[0], q[1], q[2]])
        return p


    def get_foots(self):
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()

        f1 = self.get_foot_position(p_CoM, o_CoM, 1, self.get_leg1_idx())
        f2 = self.get_foot_position(p_CoM, o_CoM, 2, self.get_leg2_idx())
        f3 = self.get_foot_position(p_CoM, o_CoM, 3, self.get_leg3_idx())
        f4 = self.get_foot_position(p_CoM, o_CoM, 4, self.get_leg4_idx())

        X = np.array([f1[0], f2[0], f3[0], f4[0]])
        X = X.astype(float)

        Y = np.array([f1[1], f2[1], f3[1], f4[1]])
        Y = Y.astype(float)
        return X, Y


    def calc_CoP(self, X, Y):
        fs = self.get_fs()
        F = np.array([[fs[0]], [fs[1]], [fs[2]], [fs[3]]])
        F = F.astype(float)

        CoP = np.matmul([X, Y], F) / sum(F)
        self.set_CoP_position([CoP[0][0], CoP[1][0], 0])

        return CoP


    def CoP_offset(self, leg):
        X, Y = self.get_foots()

        p1 = Point(X[0], Y[0], evaluate=False)
        self.set_CoP_position([X[0], Y[0], 0])
        p2 = Point(X[1], Y[1], evaluate=False)
        self.set_CoP_position([X[1], Y[1], 0])
        p3 = Point(X[2], Y[2], evaluate=False)
        self.set_CoP_position([X[2], Y[2], 0])
        p4 = Point(X[3], Y[3], evaluate=False)
        self.set_CoP_position([X[3], Y[3], 0])

        if (leg == 1):
            t = Triangle(p2, p3, p4)
        elif (leg == 2):
            t = Triangle(p1, p3, p4)
        elif (leg == 3):
            t = Triangle(p1, p2, p4)
        elif (leg == 4):
            t = Triangle(p1, p2, p3)

        CoP = self.calc_CoP(X,Y)

        CoPx = CoP[0][0]
        CoPy = CoP[1][0]
        Cx = t.centroid[0]
        Cy = t.centroid[1]

        self.set_CoP_position([Cx, Cy, 0])

        # plt.plot(X, Y, "ro")
        # plt.plot(CoPx, CoPy, "*")
        # plt.show(block=False)

        diff_x = abs(abs(CoPx) - abs(Cx))
        diff_y = abs(abs(CoPy) - abs(Cy))

        if (CoPx < Cx):
            diff_x = -diff_x
        if (CoPy < Cy):
            diff_y = -diff_y


        print("diferencia:", diff_x, diff_y)

        # check if CoP is inside the support polygon
        """dentro = t.encloses_point(Point(CoP[0][0], CoP[1][0], evaluate=False))
        print("Dentro:", dentro)
        """

        # Displace the CoP by moving the feets
        #self.offset(diff_x, diff_y, 0)


        # displacing the CoP by moving the hip and knees
        diff_x,diff_y = [-diff_x, -diff_y]
        [hip_off, knees_off] = [diff_x / 0.007 * 5, diff_y / 0.011 * 5]
        [Q_hip, Q_knees] = [np.rad2deg(self.get_joints_position(self.get_hip_idx())), np.rad2deg(self.get_joints_position(self.get_knees_idx()))]

        #Q = np.rad2deg(self.get_joints_position(self.get_all_joints_idx()))
        #self.knees_hip_offset_old(Q, Q_knees, knees_off, Q_hip, hip_off)
        self.knees_hip_offset(Q_knees, knees_off, Q_hip, hip_off)


        #self.hip_offset(Q_hip,hip_off)
        #self.knees_offset(Q_knees,knees_off)


        return


    # l1, l2: [0 = leg1:3=leg4]
    def move_CoP_trotting(self, l1,l2):
        X, Y = self.get_foots()
        CoP = self.calc_CoP(X,Y)

        CoPx = CoP[0][0]
        CoPy = CoP[1][0]
        # mid point of the segment between l1 and l2 legs
        [Cx,Cy] = [(X[l1] + X[l2]) / 2, (Y[l1] + Y[l2]) / 2]

        self.set_CoP_position([Cx, Cy, 0])

        diff_x = abs(abs(CoPx) - abs(Cx))
        diff_y = abs(abs(CoPy) - abs(Cy))

        if (CoPx < Cx):
            diff_x = -diff_x
        if (CoPy < Cy):
            diff_y = -diff_y

        print("diferencia:", diff_x, diff_y)


        # Displace the CoP
        # self.offset(diff_x, diff_y, 0)

        diff_x,diff_y = [-diff_x, -diff_y]
        [hip_off, knees_off] = [diff_x / 0.007 * 5, diff_y / 0.017 * 5]
        [Q_hip, Q_knees] = [np.rad2deg(self.get_joints_position(self.get_hip_idx())), np.rad2deg(self.get_joints_position(self.get_knees_idx()))]
        self.knees_hip_offset(Q_knees, knees_off, Q_hip, hip_off)
        return


    def offset_Q (self, Q, offset, idx):
        nQ = Q.copy()
        for i in idx:
            nQ[i] = nQ[i] + offset
        return nQ


    def gait_walk(self):

        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))

        # Pata 3
        #self.disable_torque(self.get_feet_idx())
        self.knees_hip_offset(QdKnees,-10,QdHip,20)
        #self.enable_torque(self.get_feet_idx())

        self.offset_leg(3, 0, 0, 0.03)
        self.offset_leg(3, 0, -0.04, 0)
        self.offset_leg(3, 0, 0, -0.02)

        # Pata 2
        self.desplaza_cadera(0)
        #self.disable_torque(self.get_feet_idx())
        self.knees_offset(QdKnees,10)
        #self.enable_torque(self.get_feet_idx())

        self.offset_leg(2, 0, 0, 0.03)
        self.offset_leg(2, 0, -0.04, 0)
        self.offset_leg(2, 0, 0, -0.02)

        # Pata 4
        self.desplaza_cadera(0)
        #self.disable_torque(self.get_feet_idx())
        self.knees_hip_offset(QdKnees, -10, QdHip, -20)
        #self.enable_torque(self.get_feet_idx())

        self.offset_leg(4, 0, 0, 0.03)
        self.offset_leg(4, 0, -0.04, 0)
        self.offset_leg(4, 0, 0, -0.02)

        # Pata 1
        self.desplaza_cadera(0)
        #self.disable_torque(self.get_feet_idx())
        self.knees_offset(QdKnees,10)
        #self.enable_torque(self.get_feet_idx())

        self.offset_leg(1, 0, 0, 0.03)
        self.offset_leg(1, 0, -0.04, 0)
        self.offset_leg(1, 0, 0, -0.02)

        self.move_offset(Qr)

        return

    # Requiere offset (0,0,0.04)
    def gait_walk_1324(self):

        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))

        # Pata 1
        self.desplaza_cadera(-30)
        self.offset_leg(1, 0, 0, 0.03)
        self.offset_leg(1, 0, -0.04, 0)
        self.offset_leg(1, 0, 0, -0.02)

        # Pata 3
        self.desplaza_cadera(30)
        self.offset_leg(3, 0, 0, 0.03)
        self.offset_leg(3, 0, -0.04, 0)
        self.offset_leg(3, 0, 0, -0.02)

        # Pata 2
        self.offset_leg(2, 0, 0, 0.03)
        self.offset_leg(2, 0, -0.04, 0)
        self.offset_leg(2, 0, 0, -0.02)

        # Pata 4
        self.knees_hip_offset(QdKnees, -10, QdHip, -30)
        self.offset_leg(4, 0, 0, 0.03)
        self.offset_leg(4, 0, -0.04, 0)
        self.offset_leg(4, 0, 0, -0.02)

        self.move_offset(Qr)

        return


    def gait_walk_3241(self,n):

        # posición inicial
        self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03)

        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))

        for i in range (n):
            # Pata 3
            self.desplaza_cadera(30)
            self.offset_leg(3, 0, 0, 0.03)
            self.offset_leg(3, 0, -0.04, 0)
            self.offset_leg(3, 0, 0, -0.02)

            # Pata 2
            self.offset_leg(2, 0, 0, 0.03)
            self.offset_leg(2, 0, -0.04, 0)
            self.offset_leg(2, 0, 0, -0.02)

            # Pata 4
            self.knees_hip_offset(QdKnees, -10, QdHip, -30)
            self.offset_leg(4, 0, 0, 0.03)
            self.offset_leg(4, 0, -0.04, 0)
            self.offset_leg(4, 0, 0, -0.02)

            # Pata 1
            self.knees_hip_offset(QdKnees, 10, QdHip, -30)
            self.offset_leg(1, 0, 0, 0.03)
            self.offset_leg(1, 0, -0.04, 0)
            self.offset_leg(1, 0, 0, -0.02)

            self.move_offset(Qr)

        return


    def gait_walk_3241_new(self,n):

        # posición inicial
        self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03, 5)
        time.sleep(1)

        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))

        for i in range (n):
            # Pata 3
            #self.desplaza_cadera(30)
            self.offset_leg(3, 0, 0, 0.04)
            self.offset_leg(3, 0, -0.04, 0)
            self.offset_leg(3, 0, 0, -0.03)

            # Pata 2
            self.offset_leg(2, 0, 0, 0.04)
            self.offset_leg(2, 0, -0.04, 0)
            self.offset_leg(2, 0, 0, -0.03)

            # Pata 4
            #self.knees_hip_offset(QdKnees, -10, QdHip, -30)
            self.offset_leg(4, 0, 0, 0.04)
            self.offset_leg(4, 0, -0.04, 0)
            self.offset_leg(4, 0, 0, -0.03)

            # Pata 1
            self.knees_hip_offset(QdKnees, 10, QdHip, -30)
            self.offset_leg(1, 0, 0, 0.04)
            self.offset_leg(1, 0, -0.04, 0)
            self.offset_leg(1, 0, 0, -0.03)

            self.move_offset(Qr)

        return



    def displace_leg(self, leg, y, z):
        self.CoP_offset(leg)
        self.offset_leg(leg, 0, 0, z)
        self.offset_leg(leg, 0, -y, 0)
        self.offset_leg(leg, 0, 0, -z + 0.01)
        return


    def gait_walk_CoP_3241(self, n):
        # posición inicial
        self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03, 5)
        time.sleep(1)
        Q = (self.get_joints_position(self.get_all_joints_idx()))
        for i in range(n):
            h = 0.02
            Qn = self.get_joints_position(self.get_knees_idx())
            self.displace_leg(3,0.04,h)
            self.desplaza_cadera(0)

            self.displace_leg(2, 0.04, h)
            self.desplaza_cadera(0)

            self.displace_leg(4, 0.04, h)
            self.desplaza_cadera(0)

            self.displace_leg(1, 0.04, h)

            self.move_offset(Q)
        return

    def gait_walk_CoP_1324(self, n):
        Q = (self.get_joints_position(self.get_all_joints_idx()))
        for i in range(n):
            h = 0.02
            Qn = self.get_joints_position(self.get_knees_idx())
            self.displace_leg(1,0.04,h)
            self.desplaza_cadera(0)

            self.displace_leg(3, 0.04, h)
            self.desplaza_cadera(0)

            self.displace_leg(2, 0.04, h)
            self.desplaza_cadera(0)

            self.displace_leg(4, 0.04, h)

            self.move_offset(Q)
        return


    def gait_walk_CoP_4231(self, n):
        Q = (self.get_joints_position(self.get_all_joints_idx()))
        for i in range(n):
            h = 0.02
            Qn = self.get_joints_position(self.get_knees_idx())
            self.displace_leg(4,0.04,h)
            #self.desplaza_cadera(0)

            self.displace_leg(2, 0.04, h)
            #self.desplaza_cadera(0)

            self.displace_leg(3, 0.04, h)
            #self.desplaza_cadera(0)

            self.displace_leg(1, 0.04, h)

            self.move_offset(Q)
        return


    def gait_walk_CoP(self, n):
        Q = (self.get_joints_position(self.get_all_joints_idx()))
        for i in range(n):
            h = 0.02
            Qn = self.get_joints_position(self.get_knees_idx())
            self.displace_leg(3,0.04,h)
            self.desplaza_cadera(0)

            self.displace_leg(4, 0.04, h)
            self.desplaza_cadera(0)

            self.displace_leg(1, 0.04, h)
            self.desplaza_cadera(0)

            self.desplaza_cadera(20)
            self.offset_leg(2, 0, 0, h)
            self.knees_offset(np.rad2deg(Qn), 0)
            self.offset_leg(2, 0, -0.04, 0)
            self.offset_leg(2, 0, 0, -h + 0.01)


            self.move_offset(Q)
        return



    def trotting_walk(self,n):
        Q = self.get_joints_position(self.get_all_joints_idx())
        QKnees = self.get_joints_position(self.get_knees_idx())

        for i in range(n):

            # Ubicar el CoM en la linea que une las patas 24
            #self.knees_offset(np.rad2deg(QKnees), 1)
            #self.move_CoP_trotting(0,2)

            Qr = self.get_joints_position(self.get_all_joints_idx())
            QdKnees = np.rad2deg([Qr[x] for x in self.get_knees_idx()])
            QdHip = np.rad2deg([Qr[x] for x in self.get_hip_idx()])
            self.knees_hip_offset(QdKnees, -10, QdHip, 0)

            [ts1, ts2, imu_log_13, pos_13] = self.offset_legs13(0, -0.04, 0.03, 0, 0, -0.03, 5)
            a = self.get_IMU_pos(ts1, ts2, imu_log_13, 1)
            a_gyro = self.get_IMU_gyro(ts1, ts2, imu_log_13, 1)

            # Ubicar el CoM en la linea que une las patas 13
            #self.knees_offset(np.rad2deg(QKnees), -9)
            #self.move_CoP_trotting(1,3)

            Qr = self.get_joints_position(self.get_all_joints_idx())
            QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
            QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
            self.knees_hip_offset(QdKnees, -10, QdHip, 0)

            [ts3, ts4, imu_log_24, pos_24] = self.offset_legs24(0, -0.04, 0.03, 0, 0, -0.03, 5)
            b = self.get_IMU_pos(ts3, ts4, imu_log_24, 1)
            b_gyro = self.get_IMU_gyro(ts3, ts4, imu_log_24, 1)

            [ts5, ts6, imu_log_Q, pos_Q] = self.move_offset(Q, 0, 75)
            c = self.get_IMU_pos(ts5, ts6, imu_log_Q, 1)
            c_gyro = self.get_IMU_gyro(ts5, ts6, imu_log_Q, 1)
            #[ts5, ts6, imu_log_Q, pos_Q] = self.move_offset(Q, 5)

        return





    def offset_knees_and_hip(self, knees_d, hip_d):
        Qr = self.get_joints_position(self.get_all_joints_idx())
        QdKnees = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        QdHip = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        [self.ts1, self.ts2, self.sensors_log, self.pose] = self.knees_hip_offset(QdKnees, knees_d, QdHip, hip_d)
        return [self.ts1, self.ts2, self.sensors_log, self.pose]






    # Efectua el giro sin considerar desplazamiento sobre circunferencia de las patas
    def turn(self, Q0):
        x_off = 0.04
        up = 0.06
        self.move_offset(self.offset4([-x_off, 0, 0] * 2 + [x_off, 0, 0] * 2))
        self.offset_legs13(x_off, 0, up, 0, 0, -up, 0, False)
        self.offset_legs24(x_off, 0, up, 0, 0, -up, 0, False)

        # return to standard position
        self.move_offset(Q0)

        return



    def up_down_legs(self, Q):
        yz_off = 0.04
        [x1_off,y1_off,z1_off, x2_off,y2_off,z2_off, x3_off,y3_off,z3_off, x4_off,y4_off,z4_off] = [
            0,-yz_off,yz_off, 0,-yz_off,yz_off, 0,yz_off,yz_off, 0,yz_off,yz_off]

        leg = 1

        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
        [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4] = kinematics.FK4(p_CoM, o_CoM, Q)

        P = [-p_CoM[0] + x1 + x1_off, -p_CoM[1] + y1 + y1_off, -p_CoM[2] + z1 + z1_off,
             -p_CoM[0] + x2 + x2_off, -p_CoM[1] + y2 + y2_off, -p_CoM[2] + z2 + z2_off,
             -p_CoM[0] + x3 + x3_off, -p_CoM[1] + y3 + y3_off, -p_CoM[2] + z3 + z3_off,
             -p_CoM[0] + x4 + x4_off, -p_CoM[1] + y4 + y4_off, -p_CoM[2] + z4 + z4_off]

        Qik = kinematics.IK4(o_CoM, P)

        self.set_moving_speed(1023)
        for leg in (1,3,2,4):
            idx = self.get_leg_idx(leg).copy()[0:3]
            i_goal = Qik[idx[0]:idx[2] + 1]
            i_goal_masked = self.mask_joint_positions(i_goal, self.get_leg_idx(leg))
            f_goal = Q[idx[0]:idx[2] + 1]
            f_goal_masked = self.mask_joint_positions(f_goal, self.get_leg_idx(leg))

            self.move_joints(idx, self.set_Q_goal(i_goal_masked),0)
            self.move_joints(idx, self.set_Q_goal(f_goal_masked),0)
            time.sleep(0.5)

        self.set_moving_speed(self.get_low_speed())
        return


    # Turn displacing the feet over the circunference and displacing legs13 & leg24
    def turn_circ(self, d, move_individual_legs = True):
        b = -np.deg2rad(d)

        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
        Q = self.get_joints_position(self.get_all_joints_idx())
        [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4] = kinematics.FK4(p_CoM, o_CoM, Q)

        [d1,d2,d3,d4] = [np.sqrt(x1**2 + y1**2),
                         np.sqrt(x2**2 + y2**2),
                         np.sqrt(x3**2 + y3**2),
                         np.sqrt(x4**2 + y4**2)]

        [a1,a2,a3,a4] = [np.arctan2(y1,x1),
                         np.arctan2(y2,x2),
                         np.arctan2(y3,x3),
                         np.arctan2(y4,x4)]

        [x1n,y1n, x2n,y2n, x3n,y3n, x4n,y4n] = [d1*np.cos(a1+b), d1*np.sin(a1+b),
                                                d2*np.cos(a2+b), d2*np.sin(a2+b),
                                                d3*np.cos(a3+b), d3*np.sin(a3+b),
                                                d4*np.cos(a4+b), d4*np.sin(a4+b)]
        # first way to turn
        self.move_offset_coord(self.offset4([x1n-x1, y1n-y1, 0,
                                       x2n-x2, y2n-y2, 0,
                                       x3n-x3, y3n-y3, 0,
                                       x4n-x4, y4n-y4, 0]))

        # Move individual legs one by one
        if (move_individual_legs):
            self.up_down_legs(Q)
        # Move legs 1&3  and then legs 2&4, both simultaneously
        else:
            up = 0.04
            # También valdría subir las patas y colocarlas en sy posición según Q y bajarlas.
            [ts1,ts2,imu_log1,pos] = self.offset_legs13(x1 - x1n, y1 - y1n, up, 0, 0, -up, 5, False)
            [ts3,ts4,imu_log2,pos] = self.offset_legs24(x2 - x2n, y2 - y2n, up, 0, 0, -up, 5, False)
            [a,b]=[self.get_outliers(ts1,ts2,imu_log1,1,2),self.get_outliers(ts3,ts4,imu_log2,1,2)]
        [ts5,ts6,imu_log,pos,pose_values, v, t] = self.move_offset_coord(Q)
        return





    def get_sensors_log(self):
        return self.sensors_log()

    # get the outliers values in radians (idx=1) from ts1 and ts2 in l: [ts,[x,y,z],[x_accel, y_accel, z_accel]]
    # idx = 1 (pose), 2 (accel)
    def get_outliers(self, ts1, ts2, l, idx=1, n=3, complete=True):

        pose_ts = [[r[0], r[idx]] for r in l if ts2 >= r[0] >= ts1]

        # increments the population
        if (complete):
            reg = pose_ts[0]
            elements = int((ts2-ts1)*1648/2)
            # insert at the begin
            pose_ts = [reg]*elements + pose_ts
            # insert at the end
            reg[0] = ts2
            pose_ts = pose_ts + [reg]*elements

        pose_xyz = [r[1] for r in pose_ts]
        m_pose = np.mean(pose_xyz, axis=0, dtype=np.float64)
        sd_pose = np.std(pose_xyz, axis=0, dtype=np.float64)
        outliers_pose_x = [[s[0], s[1][0]] for s in pose_ts if
                           (s[1][0] <= (m_pose[0] - n * sd_pose[0]) or s[1][0] >= (m_pose[0] + n * sd_pose[0]))]
        outliers_pose_y = [[s[0], s[1][1]] for s in pose_ts if
                           (s[1][1] <= (m_pose[1] - n * sd_pose[1]) or s[1][1] >= (m_pose[1] + n * sd_pose[1]))]
        # outliers_pose_z = [[s[0], s[1][2]] for s in pose_ts if
        #                    (s[1][2] <= (m_pose[2] - n * sd_pose[2]) or s[1][2] >= (m_pose[2] + n * sd_pose[2]))]

        # position
        if (idx == 1):
            [imu_x, imu_y] = [[np.rad2deg(x[1]) for x in outliers_pose_x], [np.rad2deg(y[1]) for y in outliers_pose_y]]
        else:
            [imu_x, imu_y] = [[x[1] for x in outliers_pose_x], [y[1] for y in outliers_pose_y]]

        [imu_x_max, imu_y_max] = [max(imu_x, key=abs), max(imu_y, key=abs)]

        return [imu_x_max, imu_y_max]


    # desde ts1 a ts2, quitamos los valores extremos, y sacamos la amplitud máxima, que devolvemos para x a y
    def get_IMU_pos(self, ts1, ts2, l, n=1):

        pose_ts = [[r[0], r[1]] for r in l if ts2 >= r[0] >= ts1]
        pose_xyz = [r[1] for r in pose_ts]

        m_pose = np.mean(pose_xyz, axis=0, dtype=np.float64)
        sd_pose = np.std(pose_xyz, axis=0, dtype=np.float64)

        # values without the outliers
        pose_x = [s[0] for s in pose_xyz if
                  (s[0] >= (m_pose[0] - n * sd_pose[0]) and s[0] <= (m_pose[0] + n * sd_pose[0]))]
        pose_y = [s[1] for s in pose_xyz if
                  (s[1] >= (m_pose[1] - n * sd_pose[1]) and s[1] <= (m_pose[1] + n * sd_pose[1]))]
        pose_z = [s[2] for s in pose_xyz if
                  (s[2] >= (m_pose[2] - n * sd_pose[2]) and s[2] <= (m_pose[2] + n * sd_pose[2]))]
        # return [np.rad2deg(np.abs(pose_xyz).max(axis=0) - np.abs(pose_xyz).min(axis=0))]



        # [imu_x, imu_y, imu_z] = [[np.rad2deg(x[1]) for x in pose_x], [np.rad2deg(y[1]) for y in pose_y], [np.rad2deg(y[1]) for y in pose_z]]
        # [imu_x_max, imu_y_max, imu_z_max] = [max(imu_x, key=abs), max(imu_y, key=abs), max(imu_z, key=abs)]
        # [imu_x_min, imu_y_min, imu_z_min] = [min(imu_x, key=abs), min(imu_y, key=abs), min(imu_z, key=abs)]
        # return [imu_x_max - imu_x_min, imu_y_max - imu_y_min, imu_z_max - imu_z_min]

        # sacamos la diferencia entre el valor máximo y la referencia pose_xyz[0], así como la diferencia
        # entre el valor minimo y la diferencia
        # pose_x = [s[0] for s in pose_xyz if
        #           (s[0] >= (m_pose[0] - n * sd_pose[0]) and s[0] <= (m_pose[0] + n * sd_pose[0]))]
        #x = max(abs(max(pose_x)-pose_x[0]), abs(min(pose_x)-pose_x[0]))
        # 
        # pose_y = [s[1] for s in pose_xyz if
        #           (s[1] >= (m_pose[1] - n * sd_pose[1]) and s[1] <= (m_pose[1] + n * sd_pose[1]))]
        #y = max(abs(max(pose_y)-pose_y[0]), abs(min(pose_y)-pose_y[0]))
        # 
        # pose_z = [s[2] for s in pose_xyz if
        #           (s[2] >= (m_pose[2] - n * sd_pose[2]) and s[2] <= (m_pose[2] + n * sd_pose[2]))]
        #z = max(abs(max(pose_z)-pose_z[0]), abs(min(pose_z)-pose_z[0]))

        #[pose_x, pose_y, pose_z] = [[s[0] for s in pose_xyz], [s[1] for s in pose_xyz], [s[2] for s in pose_xyz]]
        [x, y, z] = [max(pose_x, key=abs), max(pose_y, key=abs), max(pose_z, key=abs)]
        return [x, y, z]

        return np.rad2deg([rx, ry, rz])


    # desde ts1 a ts2, quitamos los valores extremos, y sacamos la amplitud máxima, que devolvemos para x a y
    def get_IMU_gyro(self, ts1, ts2, l, n=1):

        giro_ts = [[r[0], r[2]] for r in l if ts2 >= r[0] >= ts1]
        gyro_xyz = [r[1] for r in giro_ts]
        m = np.mean(gyro_xyz, axis=0, dtype=np.float64)
        sd = np.std(gyro_xyz, axis=0, dtype=np.float64)

        # values without the outliers
        gyro_x = [s[0] for s in gyro_xyz if ((m[0] - n * sd[0]) <= s[0] <= (m[0] + n * sd[0]))]
        gyro_y = [s[1] for s in gyro_xyz if ((m[1] - n * sd[1]) <= s[1] <= (m[1] + n * sd[1]))]
        gyro_z = [s[2] for s in gyro_xyz if ((m[2] - n * sd[2]) <= s[2] <= (m[2] + n * sd[2]))]
        # [imu_x, imu_y, imu_z] = [[x[1] for x in gyro_x], [y[1] for y in gyro_y], [z[1] for z in gyro_z]]
        #
        # return [max(imu_x) - min(imu_x), max(imu_y) - min(imu_y), max(imu_z) - min(imu_z)]

        # ax = max(abs(max(gyro_x) - gyro_x[0]), abs(min(gyro_x) - gyro_x[0]))
        # ay = max(abs(max(gyro_y) - gyro_y[0]), abs(min(gyro_y) - gyro_y[0]))
        # az = max(abs(max(gyro_z) - gyro_z[0]), abs(min(gyro_z) - gyro_z[0]))

        # [gyro_x, gyro_y, gyro_z] = [[s[0] for s in gyro_xyz], [s[1] for s in gyro_xyz], [s[2] for s in gyro_xyz]]
        [ax, ay, az] = [max(gyro_x, key=abs), max(gyro_y, key=abs), max(gyro_z, key=abs)]
        return [ax, ay, az]


    # return True if at least one leg has a poor support
    def lack_of_support(self):
        fs = self.get_fs()
        fs_v = sum(map(lambda x: x <= 5, fs))

        imu_vals = sum(map(lambda x: x >= 16, abs(np.rad2deg(self.readIMU()[0:2]))))
        return (fs_v >= 2 or imu_vals >= 1)


    #
    #  Las siguientes funciones se crearon para optimizar los cálculos, o para estimar la posición del CoM en función
    #  del desplazamiento de la cadera o de las rodillas.
    #


    def symbolic_FK(self, pata):
        [x1, y1, z1] = kinematics.symbolic_forward_kinematics(pata)
        print("x=", x1, "\ny=", y1, "\nz=", z1)
        return x1, y1, z1


    def numeric_IK(self, pata, calc_FK = False):
        if (calc_FK):
            [eq_x,eq_y,eq_z] = self.symbolic_FK(pata)
        else:
            [eq_x, eq_y, eq_z] = [None]*3

        [xt, yt, zt] = [0.0393, -0.1215, 0.28]
        [xt,yt,zt] = [0.1746, -0.1984, 0.2612]
        [x,y,z] = kinematics.numeric_inverse_kinematics(xt, yt, zt, eq_x, eq_y, eq_z)
        return [x,y,z]


    def symbolic_IK(self,pata):
        kinematics.symbolic_inverse_kinematics(pata)
        return


    # registra el desplazamiento en x del CoM, desde offset(0,0,0.02) entre -40º y  40º, con saltos de +5º
    def medir_desplazamiento_cadera(self):
        x_ant, x_act = [0,0]
        # Desde posición offset(0,0,0.04) inicial (creo grados) hacia su izquierda, en incrementos de 5 grados hasta los 40 grados
        Qi = np.rad2deg(self.get_joints_position(self.get_hip_idx()))

        self.desplaza_cadera(-40)
        Q = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
        x_ant = x_act = abs(p_CoM[0])
        print("H:", Q, "CoM:", p_CoM, "Increm.:", x_act - x_ant)

        for i in range (-35,45,5):
            self.hip_offset(Q,5)
            x_ant = abs(p_CoM[0])
            Q = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
            r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
            x_act = abs(p_CoM[0])
            print("H:", Q, "CoM:", p_CoM, "Increm.:", x_act - x_ant)

        # vuelta a la posición original
        #self.set_joints_position(self.get_joints_handle([self.get_hip_idx()]), np.deg2rad(Qi))
        self.move_joints(self.get_hip_idx(), np.deg2rad(Qi))
        return


# registra el desplazamiento en x del CoM, desde offset(0,0,0.02) entre -40º y  40º, con saltos de +5º
    def medir_desplazamiento_cadera2(self):
        Q_ini = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        Q = Q_ini.copy()
        r, p_CoM_ini, o_CoM_ini = self.get_CoM_position_and_orientation()
        #X, Y = self.get_foots()
        #p_CoM_ini = self.calc_CoP(X,Y)
        self.knees_offset(np.rad2deg(self.get_joints_position(self.get_knees_idx())), 0)

        ant = 0
        act = p_CoM_ini[0] - p_CoM_ini[0]
        print ("x: ", act, "º:", -Q_ini[0] + Q_ini[0], " diff:", act-ant)
        ant = act
        step_hip = 5
        for i in range (step_hip, 35 + step_hip, step_hip):
            self.hip_offset(Q,step_hip)
            Q = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
            r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
            #X, Y = self.get_foots()
            #p_CoM = self.calc_CoP(X,Y)
            act = p_CoM[0] - p_CoM_ini[0]
            print("x: ", act, "º:", -Q[0] + Q_ini[0], " diff:", act-ant)
            ant = act



        self.default_position()
        self.offset4([0, 0, 0.02]*4)
        self.desplaza_rodillas(5)

        Q_ini = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
        Q = Q_ini.copy()
        r, p_CoM_ini, o_CoM_ini = self.get_CoM_position_and_orientation()
        #X, Y = self.get_foots()
        #p_CoM_ini = self.calc_CoP(X,Y)

        ant = 0
        act = p_CoM_ini[0] - p_CoM_ini[0]
        print("x: ", -act, "º:", -Q_ini[0] + Q_ini[0], " diff:", act - ant)
        ant = act
        step_hip = -step_hip
        for i in range(step_hip, -35 + step_hip, step_hip):
            self.hip_offset(Q, step_hip)
            Q = np.rad2deg(self.get_joints_position(self.get_hip_idx()))
            r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
            #X, Y = self.get_foots()
            #p_CoM = self.calc_CoP(X,Y)
            act = p_CoM[0] - p_CoM_ini[0]
            print("x: ", act, "º:", -Q[0] + Q_ini[0], " diff:", act - ant)
            ant = act

        return


    # registra el desplazamiento en x del CoM, desde offset(0,0,0.02) entre -40º y  40º, con saltos de +5º
    def medir_desplazamiento_rodillas(self):
        x_ant, x_act = [0, 0]


        # Desde posición offset(0,0,0.04) inicial (creo grados) hacia su izquierda, en incrementos de 5 grados hasta los 40 grados
        Qi = np.rad2deg(self.get_joints_position(self.get_knees_idx()))

        self.knees_offset(Qi,-30)
        Q = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
        r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
        x_ant = x_act = abs(p_CoM[1])
        print("H:", Q, "CoM:", p_CoM, "Increm.:", x_act - x_ant)

        for i in range(-25, 30, 5):
            self.knees_offset(Q, 5)
            x_ant = abs(p_CoM[1])
            Q = np.rad2deg(self.get_joints_position(self.get_knees_idx()))
            r, p_CoM, o_CoM = self.get_CoM_position_and_orientation()
            x_act = abs(p_CoM[1])
            print("H:", Q, "CoM:", p_CoM, "Increm.:", x_act - x_ant)

        # vuelta a la posición original
        #self.set_joints_position(self.get_joints_handle([self.get_knees_idx()]), np.deg2rad(Qi))
        self.move_joints(self.get_knees_idx(), np.deg2rad(Qi))
        return


    def interpolate(self, X, Y, t):
        self.__interpolation.interpolate(X,Y,t)

    def get_interpolated_points(self):
        return self.__interpolation.get_interpolated_points()