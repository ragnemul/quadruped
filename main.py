

from robot import Robot
from RL import *
import time
import numpy as np
from numpy import savetxt


if __name__ == "__main__":
    # Patas en codo interior abierto offset(0,0,0.04)
    Q_offset_004 = [-0.0, 0.859029241215959, -1.4572817484913592,
                    -0.02045307717180855, -0.7976700097005335, 1.452168479198407,
                    0.0051132692929521375, 0.8845955876807198, -1.4317154020265985,
                    -0.05113269292952137, -0.853915971923007, 1.4726215563702156]

    Q_offset_003 = [-0.0, 0.7107444317203471, -1.2578642460662257,
                    -0.015339807878856412, -0.66983827737673, 1.262977515359178,
                    0.0051132692929521375, 0.7107444317203471, -1.2578642460662257,
                    -0.030679615757712823, -0.6851780852555864, 1.2834305925309866]

    Q_offset_002 = [-0.0, 0.6033657765683522, -1.0175405892974754,
                    -0.02045307717180855, -0.5573463529317829, 1.0226538585904275,
                    0.0051132692929521375, 0.5931392379824479, -1.0124273200045233,
                    -0.025566346464760685, -0.5624596222247351, 1.0277671278833795]

    robot = Robot(Q_offset_004)



    # Demo interpolación con movimientos coordinados
    [ts1, ts2, sensors_log, pose, pose_values, v, t] = robot.move_offset_coord(Q_offset_004)
    Y = [0, 0.01, 0.02, 0.03]
    Z = [0, 0.03, 0.03, 0]
    robot.interpolate(Y,Z,0.005)

    [y,z] = robot.get_interpolated_points()

    Q = []
    for y_i, z_i in zip(y,z):
        Q.append(robot.offset4([0, y_i, z_i] + [0,0,0]*3))

    Qi = []
    for Q_i in Q:
        [ts1, ts2, sensors_log, pose, pose_values, v, t] = robot.move_offset_coord(Q_i,pose)
        Qi.append({'Q': pose_values, 'v': v, 't': round(t,2)})

    # # Se ejecutan los movimientos articulares coordinados que fueron grabados para tener mayor
    # # velocidad
    # for i in range(5):
    #     [ts1, ts2, sensors_log, pose, pose_values, v, t] = robot.move_offset_coord(Q_offset_004)
    #     for Q_i  in Qi:
    #         [ts1, ts2, sensors_log, pose, pose_values, v, t] = robot.exec_move(Q_i['Q'], Q_i['v'], Q_i['t'])



    # # Giro del robot
    # robot.move_offset_coord(Q_offset_004)
    # for i in range(5):
    #     robot.turn_circ(20, False)


    # # Demo offset
    # robot.move_offset_coord(Q_offset_004)
    # robot.move_offset_coord(robot.offset4([0.08, 0.07, 0.04] * 4))
    # robot.move_offset_coord(Q_offset_004)
    # robot.move_offset_coord(robot.offset4([-0.08, -0.07, 0.04] * 4))
    # robot.move_offset_coord(Q_offset_004)


    # # Demo CoP offset
    # robot.disable_torque(robot.get_all_joints_idx())
    # robot.enable_torque(robot.get_all_joints_idx())
    # robot.move_offset(Q_offset_004, 5)
    # robot.offset_legs13(0, 0, 0.03, 0, 0, -0.03, 5)
    # robot.offset_legs24(0, 0, 0.03, 0, 0, -0.03, 5)
    #
    # for i in range(4):
    #     leg = i+1
    #     robot.move_offset_coord(Q_offset_004, 5)
    #     robot.CoP_offset(leg)
    #     if (leg == 1 or leg == 2):
    #         robot.offset_leg(leg,0,-0.04, 0.04)
    #         time.sleep(1)
    #         robot.offset_leg(leg, 0, 0.04, -0.04)
    #     else:
    #         robot.offset_leg(leg, 0, 0.04, 0.04)
    #         time.sleep(1)
    #         robot.offset_leg(leg, 0, -0.04, -0.04)
    # robot.move_offset(Q_offset_004, 5)
    # time.sleep(5)

    # robot.default_position()
    # robot.symbolic_FK(1)
    #robot.numeric_IK(1)
    #robot.symbolic_IK(1)


    # # demo gateo
    # robot.move_offset_coord(Q_offset_004)
    # #robot.move_3241(5)
    # # robot.move_1423(2)
    # robot.move_1324(2)
    # #robot.gait_walk_3241_new(4)

    #robot.trot(2)

    time.sleep(1)


    # # demo play RL
    # robot.move_offset(Q_offset_004)
    # agent = AgentQ(robot)
    # #agent.trainning()
    # agent.play(10)



    #robot.default_position()
    #robot.move_offset(robot.offset4([0,0,0.04]*4))
    #robot.offset(0, 0, 0.04)
    # Patas delanteras codo interior abierto, traseras codo interior cerrado
    #Q = [0.025566346464760685, 0.8692557798018633, -1.4675082870772633, -0.010226538585904275, -0.8385761640441505, 1.4623950177843112, 0.04601942363656924, -0.8743690490948155, 1.4623950177843112, -0.030679615757712823, 0.9050486648525283, -1.4572817484913592]




    # robot.move_offset(Q_offset_003)
    # robot.gait_walk_1324()           # funciona mejor con offset(0,0,0.03)

    #[ts1,ts2,imu_log,pos] = robot.move_offset(Q_offset_004)
    #robot.offset(0,0,0.02)

    # robot.move_offset(Q_offset_004)
    # agent = AgentQ(robot)
    # agent.trainning()
    #agent.play(10)

    #robot.move_offset(Q_offset_004)

    #robot.gait_walk_CoP_4231(1)
    #robot.gait_walk_CoP_1324(2)



    # robot.move_offset(Q_offset_004)
    # robot.move2(2)
    #robot.trotting_walk(1)


    #robot.move_offset(Q_offset_004)
    #robot.trotting_walk(5)
    #robot.gait_walk_CoP(1)     # funciona mejor con offset(0,0,0.04)
    #robot.medir_desplazamiento_cadera2()
    # robot.medir_desplazamiento_rodillas()

    del robot

