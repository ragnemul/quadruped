import sympy as sp  # librería para cálculo simbólico
import numpy as np
from math import *
import sys

__exception = False

def __raise_exception():
    global __exception
    __exception = True
    return

def clear_exception():
    global __exception
    __exception = False
    return

def excp_cond():
    global __exception
    return __exception

# Definimos una función para construir las matrices de transformación
# en forma simbóĺica a partir de los parámetros D-H
def sym_transform_dh(theta, d, a, alpha):
    # theta y alpha en radianes
    # d y a en metros
    Rz = sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, 0],
                    [sp.sin(theta), sp.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    tz = sp.Matrix([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, d],
                    [0, 0, 0, 1]])
    ta = sp.Matrix([[1, 0, 0, a],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    Rx = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(alpha), -sp.sin(alpha), 0],
                    [0, sp.sin(alpha), sp.cos(alpha), 0],
                    [0, 0, 0, 1]])
    T = Rz * tz * ta * Rx
    return T


# realiza la transformación de un punto especificado por (x,y,z) para
# que pueda ser operado con matrices de transformación
def transform_p(x, y, z):
    T = sp.Matrix([[1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, 1, z],
                   [0, 0, 0, 1]])
    return T


# Realiza la cinemática directa del esquema CoppeliaSIM MTB_IK_v4_Florian_rotado_trasladado.ttt
# Recibe como parámetros los ángulos en radianes de las articulaciones
# Devuelve la posición del efector final
# alpha (a), beta (b), y gamma (g) son los ángulos de rotación de Dummy CoM
def forward_kinematics_old(pata, P, O, R):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [a, b, c] = O

    # Ángulos a girar
    [q1, q2, q3] = R

    qz = sp.symbols('qz')
    rz = sp.Matrix([[sp.cos(qz), -sp.sin(qz), 0, 0],
                    [sp.sin(qz), sp.cos(qz), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Rotación en Y de pi/2 para pasar del frame 0 al frame 1 (hip)
    qy = sp.symbols('qy')
    ry = sp.Matrix([[sp.cos(qy), 0, sp.sin(qy), 0],
                    [0, 1, 0, 0],
                    [-sp.sin(qy), 0, sp.cos(qy), 0],
                    [0, 0, 0, 1]
                    ])

    qx = sp.symbols('qx')
    rx = sp.Matrix([[1, 0, 0, 0],
                    [0, sp.cos(qx), -sp.sin(qx), 0],
                    [0, sp.sin(qx), sp.cos(qx), 0],
                    [0, 0, 0, 1]])

    # transformación debido a valores de la IMU con rotaciones consecutivas
    A00 = rx.subs({qx: a}) * ry.subs({qy: b}) * rz.subs({qz: c})

    # Matriz de desplazamiento pata2 del CoM hacia la articulación de la pierna
    if (pata == 1):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = rz.subs({qz: -sp.pi / 2}) * ry.subs({qy: sp.pi / 2})
    elif (pata == 2):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = rz.subs({qz: sp.pi / 2}) * ry.subs({qy: sp.pi / 2})
    elif (pata == 3):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = rz.subs({qz: sp.pi / 2}) * ry.subs({qy: sp.pi / 2})
    elif (pata == 4):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = rz.subs({qz: -sp.pi / 2}) * ry.subs({qy: sp.pi / 2})

    # Rotación sobre la cadera, traslación sobre X (l2) y rotación sobre eje X de π / 2
    A12 = sym_transform_dh(q1, 0, l2, sp.pi / 2)

    # Traslación sobre X de -l1 hacia la articulación 2
    A23 = sym_transform_dh(0, -l1, 0, 0)

    # Rotación de q2 sobre la articulación 2 y desplazamiento en X de l3
    A34 = sym_transform_dh(q2, 0, l3, 0)

    # Rotación de q3 sobre la articulación 3 y desplazamiento en X de l4
    A45 = sym_transform_dh(q3, 0, l4, 0)

    # Punto inicial
    Pi = transform_p(P[0], P[1], P[2])

    # Transformación total desde el punto O
    #T05 = sp.simplify(Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45)
    T05 = Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45

    x_tip = T05[0, 3]
    y_tip = T05[1, 3]
    z_tip = T05[2, 3]

    return x_tip, y_tip, z_tip


def Rx (a):
    r = sp.Matrix([[1, 0, 0, 0], [0, sp.cos(a), -sp.sin(a), 0], [0, sp.sin(a), sp.cos(a), 0], [0, 0, 0, 1]])
    return r

def Ry (b):
    r = sp.Matrix([[sp.cos(b), 0, sp.sin(b), 0], [0, 1, 0, 0], [-sp.sin(b), 0, sp.cos(b), 0], [0, 0, 0, 1]])
    return r


def Rz (c):
    r = sp.Matrix([[sp.cos(c), -sp.sin(c), 0, 0], [sp.sin(c), sp.cos(c), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return r


def forward_kinematics(pata, P, O, R):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [a, b, c] = O

    # Ángulos a girar
    [q1, q2, q3] = R


    # transformación debido a valores de la IMU con rotaciones consecutivas
    A00 = Rx(a) * Ry(b) * Rz(c)

    # Matriz de desplazamiento pata2 del CoM hacia la articulación de la pierna
    if (pata == 1):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(-sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 2):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 3):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 4):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(-sp.pi / 2) * Ry(sp.pi / 2)

    # Rotación sobre la cadera, traslación sobre X (l2) y rotación sobre eje X de π / 2
    A12 = sym_transform_dh(q1, 0, l2, sp.pi / 2)

    # Traslación sobre X de -l1 hacia la articulación 2
    A23 = sym_transform_dh(0, -l1, 0, 0)

    # Rotación de q2 sobre la articulación 2 y desplazamiento en X de l3
    A34 = sym_transform_dh(q2, 0, l3, 0)

    # Rotación de q3 sobre la articulación 3 y desplazamiento en X de l4
    A45 = sym_transform_dh(q3, 0, l4, 0)

    # Punto inicial
    Pi = transform_p(P[0], P[1], P[2])

    # Transformación total desde el punto O
    T05 = sp.simplify(Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45)

    #T05 = Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45

    x_tip = T05[0, 3]
    y_tip = T05[1, 3]
    z_tip = T05[2, 3]

    return x_tip, y_tip, z_tip



# Calcula la cinemática directa de los cuatro pies del robot.
def FK4(P, O, R):
    # Orientación del CoM
    [a, b, c] = O

    # Posiciones del CoM
    [p1, p2, p3] = P

    [q11, q12, q13, q21, q22, q23, q31, q32, q33, q41, q42, q43] = R

    [sa, sb, sc] = [sin(a), sin(b), sin(c)]
    [ca, cb, cc] = [cos(a), cos(b), cos(c)]
    [sq11, sq12, sq21, sq22, sq31, sq32, sq41, sq42] = [sin(q11), sin(q12), sin(q21), sin(q22), sin(q31), sin(q32), sin(q41), sin(q42)]
    [cq11, cq12, cq21, cq22, cq31, cq32, cq41, cq42] = [cos(q11), cos(q12), cos(q21), cos(q22), cos(q31), cos(q32), cos(q41), cos(q42)]
    [sq12q13, cq12q13, sq22q23, cq22q23, sq32q33, cq32q33, sq42q43, cq42q43] = \
        [sin(q12 + q13), cos(q12 + q13), sin(q22 + q23), cos(q22 + q23), sin(q32 + q33), cos(q32 + q33), sin(q42 + q43), cos(q42 + q43)]
    [sbsq11, sbcq11, scsq11, scsq11ca, sq11cbcc] = [sb * sq11, sb * cq11, sc * sq11, sc * sq11 * ca, sq11 * cb * cc]
    [sbsq21, sbcq21, scsq21, scsq21ca, sq21cbcc] = [sb * sq21, sb * cq21, sc * sq21, sc * sq21 * ca, sq21 * cb * cc]
    [sbsq31, sbcq31, scsq31, scsq31ca, sq31cbcc] = [sb * sq31, sb * cq31, sc * sq31, sc * sq31 * ca, sq31 * cb * cc]
    [sbsq41, sbcq41, scsq41, scsq41ca, sq41cbcc] = [sb * sq41, sb * cq41, sc * sq41, sc * sq41 * ca, sq41 * cb * cc]

    [sasbsc, cacb, cacc, sbcacc, sbscca, sasc, sacc, sacb, sccb] = [sa * sb * sc, ca * cb, ca * cc, sb * ca * cc, sb * sc * ca,
                                                                    sa * sc, sa * cc, sa * cb, sc * cb]
    [x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4] = [
        p1 + 0.03997 * sbsq11 - 0.0675 * sbcq11 * cq12 - 0.0905 * sbcq11 * cq12q13 - 0.0225 * sbcq11 - 0.0285 * sb + 0.0675 * sccb * sq12 + 0.0905 * sccb * sq12q13 + 0.1215 * sccb + 0.0675 * sq11cbcc * cq12 + 0.0905 * sq11cbcc * cq12q13 + 0.0225 * sq11cbcc + 0.03997 * cb * cc * cq11 + 0.054 * cb * cc,
        p2 + 0.0675 * sasbsc * sq12 + 0.0905 * sasbsc * sq12q13 + 0.1215 * sasbsc + 0.0675 * sacc * sbsq11 * cq12 + 0.0905 * sacc * sbsq11 * cq12q13 + 0.0225 * sacc * sbsq11 + 0.03997 * sacc * sb * cq11 + 0.054 * sacc * sb - 0.03997 * sa * sq11 * cb + 0.0675 * sacb * cq11 * cq12 + 0.0905 * sacb * cq11 * cq12q13 + 0.0225 * sacb * cq11 + 0.0285 * sacb + 0.0675 * scsq11ca * cq12 + 0.0905 * scsq11ca * cq12q13 + 0.0225 * scsq11ca + 0.03997 * sc * ca * cq11 + 0.054 * sc * ca - 0.0675 * sq12 * cacc - 0.0905 * sq12q13 * cacc - 0.1215 * cacc,
        p3 + 0.0675 * sa * scsq11 * cq12 + 0.0905 * sa * scsq11 * cq12q13 + 0.0225 * sasc * sq11 + 0.03997 * sasc * cq11 + 0.054 * sa * sc - 0.0675 * sa * sq12 * cc - 0.0905 * sacc * sq12q13 - 0.1215 * sacc - 0.0675 * sbscca * sq12 - 0.0905 * sbscca * sq12q13 - 0.1215 * sbscca - 0.0675 * sbsq11 * cacc * cq12 - 0.0905 * sbsq11 * cacc * cq12q13 - 0.0225 * sbsq11 * cacc - 0.03997 * sbcacc * cq11 - 0.054 * sbcacc + 0.03997 * sq11 * cacb - 0.0675 * cacb * cq11 * cq12 - 0.0905 * cacb * cq11 * cq12q13 - 0.0225 * cacb * cq11 - 0.0285 * cacb,

        p1 + 0.03997 * sbsq21 - 0.0675 * sbcq21 * cq22 - 0.0905 * sbcq21 * cq22q23 - 0.0225 * sbcq21 - 0.0285 * sb - 0.0675 * sccb * sq22 - 0.0905 * sccb * sq22q23 + 0.1215 * sccb - 0.0675 * sq21cbcc * cq22 - 0.0905 * sq21cbcc * cq22q23 - 0.0225 * sq21cbcc - 0.03997 * cb * cc * cq21 - 0.054 * cb * cc,
        p2 - 0.0675 * sasbsc * sq22 - 0.0905 * sasbsc * sq22q23 + 0.1215 * sasbsc - 0.0675 * sacc * sbsq21 * cq22 - 0.0905 * sacc * sbsq21 * cq22q23 - 0.0225 * sacc * sbsq21 - 0.03997 * sacc * sb * cq21 - 0.054 * sacc * sb - 0.03997 * sa * sq21 * cb + 0.0675 * sacb * cq21 * cq22 + 0.0905 * sacb * cq21 * cq22q23 + 0.0225 * sacb * cq21 + 0.0285 * sacb - 0.0675 * scsq21ca * cq22 - 0.0905 * scsq21ca * cq22q23 - 0.0225 * scsq21ca - 0.03997 * sc * ca * cq21 - 0.054 * sc * ca + 0.0675 * sq22 * cacc + 0.0905 * sq22q23 * cacc - 0.1215 * cacc,
        p3 - 0.0675 * sa * scsq21 * cq22 - 0.0905 * sa * scsq21 * cq22q23 - 0.0225 * sasc * sq21 - 0.03997 * sasc * cq21 - 0.054 * sa * sc + 0.0675 * sacc * sq22 + 0.0905 * sacc * sq22q23 - 0.1215 * sacc + 0.0675 * sbscca * sq22 + 0.0905 * sbscca * sq22q23 - 0.1215 * sbscca + 0.0675 * sbsq21 * cacc * cq22 + 0.0905 * sbsq21 * cacc * cq22q23 + 0.0225 * sbsq21 * cacc + 0.03997 * sbcacc * cq21 + 0.054 * sbcacc + 0.03997 * sq21 * cacb - 0.0675 * cacb * cq21 * cq22 - 0.0905 * cacb * cq21 * cq22q23 - 0.0225 * cacb * cq21 - 0.0285 * cacb,

        p1 + 0.03997 * sbsq31 - 0.0675 * sbcq31 * cq32 - 0.0905 * sbcq31 * cq32q33 - 0.0225 * sbcq31 - 0.0285 * sb - 0.0675 * sccb * sq32 - 0.0905 * sccb * sq32q33 - 0.1215 * sccb - 0.0675 * sq31cbcc * cq32 - 0.0905 * sq31cbcc * cq32q33 - 0.0225 * sq31cbcc - 0.03997 * cb * cc * cq31 - 0.054 * cb * cc,
        p2 - 0.0675 * sasbsc * sq32 - 0.0905 * sasbsc * sq32q33 - 0.1215 * sasbsc - 0.0675 * sa * sbsq31 * cc * cq32 - 0.0905 * sa * sbsq31 * cc * cq32q33 - 0.0225 * sacc * sbsq31 - 0.03997 * sacc * sb * cq31 - 0.054 * sacc * sb - 0.03997 * sa * sq31 * cb + 0.0675 * sacb * cq31 * cq32 + 0.0905 * sacb * cq31 * cq32q33 + 0.0225 * sacb * cq31 + 0.0285 * sacb - 0.0675 * scsq31ca * cq32 - 0.0905 * scsq31ca * cq32q33 - 0.0225 * scsq31ca - 0.03997 * sc * ca * cq31 - 0.054 * sc * ca + 0.0675 * sq32 * cacc + 0.0905 * sq32q33 * cacc + 0.1215 * cacc,
        p3 - 0.0675 * sa * scsq31 * cq32 - 0.0905 * sa * scsq31 * cq32q33 - 0.0225 * sasc * sq31 - 0.03997 * sasc * cq31 - 0.054 * sa * sc + 0.0675 * sacc * sq32 + 0.0905 * sacc * sq32q33 + 0.1215 * sacc + 0.0675 * sbscca * sq32 + 0.0905 * sbscca * sq32q33 + 0.1215 * sbscca + 0.0675 * sbsq31 * cacc * cq32 + 0.0905 * sbsq31 * cacc * cq32q33 + 0.0225 * sbsq31 * cacc + 0.03997 * sbcacc * cq31 + 0.054 * sbcacc + 0.03997 * sq31 * cacb - 0.0675 * cacb * cq31 * cq32 - 0.0905 * cacb * cq31 * cq32q33 - 0.0225 * cacb * cq31 - 0.0285 * cacb,

        p1 + 0.03997 * sbsq41 - 0.0675 * sbcq41 * cq42 - 0.0905 * sbcq41 * cq42q43 - 0.0225 * sbcq41 - 0.0285 * sb + 0.0675 * sccb * sq42 + 0.0905 * sccb * sq42q43 - 0.1215 * sccb + 0.0675 * sq41cbcc * cq42 + 0.0905 * sq41cbcc * cq42q43 + 0.0225 * sq41cbcc + 0.03997 * cb * cc * cq41 + 0.054 * cb * cc,
        p2 + 0.0675 * sasbsc * sq42 + 0.0905 * sasbsc * sq42q43 - 0.1215 * sasbsc + 0.0675 * sacc * sbsq41 * cq42 + 0.0905 * sacc * sbsq41 * cq42q43 + 0.0225 * sacc * sbsq41 + 0.03997 * sacc * sb * cq41 + 0.054 * sacc * sb - 0.03997 * sa * sq41 * cb + 0.0675 * sacb * cq41 * cq42 + 0.0905 * sacb * cq41 * cq42q43 + 0.0225 * sacb * cq41 + 0.0285 * sacb + 0.0675 * scsq41ca * cq42 + 0.0905 * scsq41ca * cq42q43 + 0.0225 * scsq41ca + 0.03997 * sc * ca * cq41 + 0.054 * sc * ca - 0.0675 * sq42 * cacc - 0.0905 * sq42q43 * cacc + 0.1215 * cacc,
        p3 + 0.0675 * sa * scsq41 * cq42 + 0.0905 * sa * scsq41 * cq42q43 + 0.0225 * sasc * sq41 + 0.03997 * sasc * cq41 + 0.054 * sa * sc - 0.0675 * sacc * sq42 - 0.0905 * sacc * sq42q43 + 0.1215 * sacc - 0.0675 * sbscca * sq42 - 0.0905 * sbscca * sq42q43 + 0.1215 * sbscca - 0.0675 * sbsq41 * cacc * cq42 - 0.0905 * sbsq41 * cacc * cq42q43 - 0.0225 * sbsq41 * cacc - 0.03997 * sbcacc * cq41 - 0.054 * sbcacc + 0.03997 * sq41 * cacb - 0.0675 * cacb * cq41 * cq42 - 0.0905 * cacb * cq41 * cq42q43 - 0.0225 * cacb * cq41 - 0.0285 * cacb
    ]
    return [x1,y1,z1, x2,y2,z2, x3,y3,z3, x4,y4,z4]


def FK (pata,P,O,R):
    # Orientación del CoM
    [a, b, c] = O

    # Ángulos a girar
    [q1, q2, q3] = R

    [p1,p2,p3] = P

    [sa, sb, sc] = [sin(a), sin(b), sin(c)]
    [ca, cb, cc] = [cos(a), cos(b), cos(c)]
    [sq1,sq2] = [sin(q1), sin(q2)]
    [cq1, cq2] = [cos(q1), cos(q2)]
    [sq2q3, cq2q3] = [sin(q2+q3), cos(q2+q3)]
    [sbsq1, sbcq1,sasbsc, cacb, cacc, sbcacc, sbscca, scsq1, scsq1ca,sq1cbcc,sasc, sacc, sacb,sccb] = [sb*sq1, sb*cq1, sa*sb*sc, ca*cb, ca*cc, sb*ca*cc, sb*sc*ca, sc*sq1, sc*sq1*ca, sq1*cb*cc, sa*sc, sa*cc, sa*cb,sc*cb]


    if (pata == 1):
        x = p1 + 0.03997*sbsq1 - 0.0675*sbcq1*cq2 - 0.0905*sbcq1*cq2q3 - 0.0225*sbcq1 - 0.0285*sb + 0.0675*sccb*sq2 + 0.0905*sccb*sq2q3 + 0.1215*sccb + 0.0675*sq1cbcc*cq2 + 0.0905*sq1cbcc*cq2q3 + 0.0225*sq1cbcc + 0.03997*cb*cc*cq1 + 0.054*cb*cc
        y = p2 + 0.0675*sasbsc*sq2 + 0.0905*sasbsc*sq2q3 + 0.1215*sasbsc + 0.0675*sacc*sbsq1*cq2 + 0.0905*sacc*sbsq1*cq2q3 + 0.0225*sacc*sbsq1 + 0.03997*sacc*sb*cq1 + 0.054*sacc*sb - 0.03997*sa*sq1*cb + 0.0675*sacb*cq1*cq2 + 0.0905*sacb*cq1*cq2q3 + 0.0225*sacb*cq1 + 0.0285*sacb + 0.0675*scsq1ca*cq2 + 0.0905*scsq1ca*cq2q3 + 0.0225*scsq1ca + 0.03997*sc*ca*cq1 + 0.054*sc*ca - 0.0675*sq2*cacc - 0.0905*sq2q3*cacc - 0.1215*cacc
        z = p3 + 0.0675*sa*scsq1*cq2 + 0.0905*sa*scsq1*cq2q3 + 0.0225*sasc*sq1 + 0.03997*sasc*cq1 + 0.054*sa*sc - 0.0675*sa*sq2*cc - 0.0905*sacc*sq2q3 - 0.1215*sacc - 0.0675*sbscca*sq2 - 0.0905*sbscca*sq2q3 - 0.1215*sbscca - 0.0675*sbsq1*cacc*cq2 - 0.0905*sbsq1*cacc*cq2q3 - 0.0225*sbsq1*cacc - 0.03997*sbcacc*cq1 - 0.054*sbcacc + 0.03997*sq1*cacb - 0.0675*cacb*cq1*cq2 - 0.0905*cacb*cq1*cq2q3 - 0.0225*cacb*cq1 - 0.0285*cacb
    if (pata == 2):
        x = p1 + 0.03997*sbsq1 - 0.0675*sbcq1*cq2 - 0.0905*sbcq1*cq2q3 - 0.0225*sbcq1 - 0.0285*sb - 0.0675*sccb*sq2 - 0.0905*sccb*sq2q3 + 0.1215*sccb - 0.0675*sq1cbcc*cq2 - 0.0905*sq1cbcc*cq2q3 - 0.0225*sq1cbcc - 0.03997*cb*cc*cq1 - 0.054*cb*cc
        y = p2 - 0.0675*sasbsc*sq2 - 0.0905*sasbsc*sq2q3 + 0.1215*sasbsc - 0.0675*sacc*sbsq1*cq2 - 0.0905*sacc*sbsq1*cq2q3 - 0.0225*sacc*sbsq1 - 0.03997*sacc*sb*cq1 - 0.054*sacc*sb - 0.03997*sa*sq1*cb + 0.0675*sacb*cq1*cq2 + 0.0905*sacb*cq1*cq2q3 + 0.0225*sacb*cq1 + 0.0285*sacb - 0.0675*scsq1ca*cq2 - 0.0905*scsq1ca*cq2q3 - 0.0225*scsq1ca - 0.03997*sc*ca*cq1 - 0.054*sc*ca + 0.0675*sq2*cacc + 0.0905*sq2q3*cacc - 0.1215*cacc
        z = p3 - 0.0675*sa*scsq1*cq2 - 0.0905*sa*scsq1*cq2q3 - 0.0225*sasc*sq1 - 0.03997*sasc*cq1 - 0.054*sa*sc + 0.0675*sacc*sq2 + 0.0905*sacc*sq2q3 - 0.1215*sacc + 0.0675*sbscca*sq2 + 0.0905*sbscca*sq2q3 - 0.1215*sbscca + 0.0675*sbsq1*cacc*cq2 + 0.0905*sbsq1*cacc*cq2q3 + 0.0225*sbsq1*cacc + 0.03997*sbcacc*cq1 + 0.054*sbcacc + 0.03997*sq1*cacb - 0.0675*cacb*cq1*cq2 - 0.0905*cacb*cq1*cq2q3 - 0.0225*cacb*cq1 - 0.0285*cacb
    if (pata == 3):
        x = p1 + 0.03997*sbsq1 - 0.0675*sbcq1*cq2 - 0.0905*sbcq1*cq2q3 - 0.0225*sbcq1 - 0.0285*sb - 0.0675*sccb*sq2 - 0.0905*sccb*sq2q3 - 0.1215*sccb - 0.0675*sq1cbcc*cq2 - 0.0905*sq1cbcc*cq2q3 - 0.0225*sq1cbcc - 0.03997*cb*cc*cq1 - 0.054*cb*cc
        y = p2 - 0.0675*sasbsc*sq2 - 0.0905*sasbsc*sq2q3 - 0.1215*sasbsc - 0.0675*sa*sbsq1*cc*cq2 - 0.0905*sa*sbsq1*cc*cq2q3 - 0.0225*sacc*sbsq1 - 0.03997*sacc*sb*cq1 - 0.054*sacc*sb - 0.03997*sa*sq1*cb + 0.0675*sacb*cq1*cq2 + 0.0905*sacb*cq1*cq2q3 + 0.0225*sacb*cq1 + 0.0285*sacb - 0.0675*scsq1ca*cq2 - 0.0905*scsq1ca*cq2q3 - 0.0225*scsq1ca - 0.03997*sc*ca*cq1 - 0.054*sc*ca + 0.0675*sq2*cacc + 0.0905*sq2q3*cacc + 0.1215*cacc
        z = p3 - 0.0675*sa*scsq1*cq2 - 0.0905*sa*scsq1*cq2q3 - 0.0225*sasc*sq1 - 0.03997*sasc*cq1 - 0.054*sa*sc + 0.0675*sacc*sq2 + 0.0905*sacc*sq2q3 + 0.1215*sacc + 0.0675*sbscca*sq2 + 0.0905*sbscca*sq2q3 + 0.1215*sbscca + 0.0675*sbsq1*cacc*cq2 + 0.0905*sbsq1*cacc*cq2q3 + 0.0225*sbsq1*cacc + 0.03997*sbcacc*cq1 + 0.054*sbcacc + 0.03997*sq1*cacb - 0.0675*cacb*cq1*cq2 - 0.0905*cacb*cq1*cq2q3 - 0.0225*cacb*cq1 - 0.0285*cacb
    if (pata == 4):
        x = p1 + 0.03997*sbsq1 - 0.0675*sbcq1*cq2 - 0.0905*sbcq1*cq2q3 - 0.0225*sbcq1 - 0.0285*sb + 0.0675*sccb*sq2 + 0.0905*sccb*sq2q3 - 0.1215*sccb + 0.0675*sq1cbcc*cq2 + 0.0905*sq1cbcc*cq2q3 + 0.0225*sq1cbcc + 0.03997*cb*cc*cq1 + 0.054*cb*cc
        y = p2 + 0.0675*sasbsc*sq2 + 0.0905*sasbsc*sq2q3 - 0.1215*sasbsc + 0.0675*sacc*sbsq1*cq2 + 0.0905*sacc*sbsq1*cq2q3 + 0.0225*sacc*sbsq1 + 0.03997*sacc*sb*cq1 + 0.054*sacc*sb - 0.03997*sa*sq1*cb + 0.0675*sacb*cq1*cq2 + 0.0905*sacb*cq1*cq2q3 + 0.0225*sacb*cq1 + 0.0285*sacb + 0.0675*scsq1ca*cq2 + 0.0905*scsq1ca*cq2q3 + 0.0225*scsq1ca + 0.03997*sc*ca*cq1 + 0.054*sc*ca - 0.0675*sq2*cacc - 0.0905*sq2q3*cacc + 0.1215*cacc
        z = p3 + 0.0675*sa*scsq1*cq2 + 0.0905*sa*scsq1*cq2q3 + 0.0225*sasc*sq1 + 0.03997*sasc*cq1 + 0.054*sa*sc - 0.0675*sacc*sq2 - 0.0905*sacc*sq2q3 + 0.1215*sacc - 0.0675*sbscca*sq2 - 0.0905*sbscca*sq2q3 + 0.1215*sbscca - 0.0675*sbsq1*cacc*cq2 - 0.0905*sbsq1*cacc*cq2q3 - 0.0225*sbsq1*cacc - 0.03997*sbcacc*cq1 - 0.054*sbcacc + 0.03997*sq1*cacb - 0.0675*cacb*cq1*cq2 - 0.0905*cacb*cq1*cq2q3 - 0.0225*cacb*cq1 - 0.0285*cacb

    return x,y,z



def symbolic_forward_kinematics(pata):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    # Orientación del CoM
    [a, b, c] = sp.symbols('a,b,c')

    # Ángulos a girar
    [q1, q2, q3] = sp.symbols('q1,q2,q3')


    # transformación debido a valores de la IMU con rotaciones consecutivas
    A00 = Rx(a) * Ry(b) * Rz(c)

    # Matriz de desplazamiento pata2 del CoM hacia la articulación de la pierna
    if (pata == 1):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(-sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 2):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 3):
        T0 = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(sp.pi / 2) * Ry(sp.pi / 2)
    elif (pata == 4):
        T0 = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        # Transformacion desde la orientación del CoM a la primera articulación
        A01 = Rz(-sp.pi / 2) * Ry(sp.pi / 2)

    # Rotación sobre la cadera, traslación sobre X (l2) y rotación sobre eje X de π / 2
    A12 = sym_transform_dh(q1, 0, l2, sp.pi / 2)

    # Traslación sobre X de -l1 hacia la articulación 2
    A23 = sym_transform_dh(0, -l1, 0, 0)

    # Rotación de q2 sobre la articulación 2 y desplazamiento en X de l3
    A34 = sym_transform_dh(q2, 0, l3, 0)

    # Rotación de q3 sobre la articulación 3 y desplazamiento en X de l4
    A45 = sym_transform_dh(q3, 0, l4, 0)

    # Punto inicial
    [p1,p2,p3] = sp.symbols('p1,p2,p3')

    Pi = sp.Matrix([[1, 0, 0, p1],
                   [0, 1, 0, p2],
                   [0, 0, 1, p3],
                   [0, 0, 0, 1]])

    # Transformación total desde el punto O
    T05 = sp.simplify(Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45)

    #T05 = Pi * A00 * T0 * A01 * A12 * A23 * A34 * A45

    x_tip = T05[0, 3]
    y_tip = T05[1, 3]
    z_tip = T05[2, 3]

    [x_tip_opt, y_tip_opt, z_tip_opt] = [sp.simplify(x_tip), sp.simplify(y_tip), sp.simplify(z_tip)]

    return [x_tip_opt, y_tip_opt, z_tip_opt]


def numeric_inverse_kinematics(x, y, z, eq_x, eq_y, eq_z):
    [a, b, c] = sp.symbols('a,b,c')
    [p1, p2, p3] = sp.symbols('p1,p2,p3')
    [q1, q2, q3] = sp.symbols('q1,q2,q3')

    if (eq_x == None):
        [eq_x, eq_y, eq_z] = sp.symbols('eq_x, eq_y, eq_z')
        eq_z = 1.0*p3 + 0.0675*sp.sin(a)*sp.sin(c)*sp.sin(q1)*sp.cos(q2) + 0.0905*sp.sin(a)*sp.sin(c)*sp.sin(q1)*sp.cos(q2 + q3) + 0.0225*sp.sin(a)*sp.sin(c)*sp.sin(q1) + 0.03997*sp.sin(a)*sp.sin(c)*sp.cos(q1) + 0.054*sp.sin(a)*sp.sin(c) - 0.0675*sp.sin(a)*sp.sin(q2)*sp.cos(c) - 0.0905*sp.sin(a)*sp.sin(q2 + q3)*sp.cos(c) - 0.1215*sp.sin(a)*sp.cos(c) - 0.0675*sp.sin(b)*sp.sin(c)*sp.sin(q2)*sp.cos(a) - 0.0905*sp.sin(b)*sp.sin(c)*sp.sin(q2 + q3)*sp.cos(a) - 0.1215*sp.sin(b)*sp.sin(c)*sp.cos(a) - 0.0675*sp.sin(b)*sp.sin(q1)*sp.cos(a)*sp.cos(c)*sp.cos(q2) - 0.0905*sp.sin(b)*sp.sin(q1)*sp.cos(a)*sp.cos(c)*sp.cos(q2 + q3) - 0.0225*sp.sin(b)*sp.sin(q1)*sp.cos(a)*sp.cos(c) - 0.03997*sp.sin(b)*sp.cos(a)*sp.cos(c)*sp.cos(q1) - 0.054*sp.sin(b)*sp.cos(a)*sp.cos(c) + 0.03997*sp.sin(q1)*sp.cos(a)*sp.cos(b) - 0.0675*sp.cos(a)*sp.cos(b)*sp.cos(q1)*sp.cos(q2) - 0.0905*sp.cos(a)*sp.cos(b)*sp.cos(q1)*sp.cos(q2 + q3) - 0.0225*sp.cos(a)*sp.cos(b)*sp.cos(q1) - 0.0285*sp.cos(a)*sp.cos(b)
        eq_y = 1.0*p2 + 0.0675*sp.sin(a)*sp.sin(b)*sp.sin(c)*sp.sin(q2) + 0.0905*sp.sin(a)*sp.sin(b)*sp.sin(c)*sp.sin(q2 + q3) + 0.1215*sp.sin(a)*sp.sin(b)*sp.sin(c) + 0.0675*sp.sin(a)*sp.sin(b)*sp.sin(q1)*sp.cos(c)*sp.cos(q2) + 0.0905*sp.sin(a)*sp.sin(b)*sp.sin(q1)*sp.cos(c)*sp.cos(q2 + q3) + 0.0225*sp.sin(a)*sp.sin(b)*sp.sin(q1)*sp.cos(c) + 0.03997*sp.sin(a)*sp.sin(b)*sp.cos(c)*sp.cos(q1) + 0.054*sp.sin(a)*sp.sin(b)*sp.cos(c) - 0.03997*sp.sin(a)*sp.sin(q1)*sp.cos(b) + 0.0675*sp.sin(a)*sp.cos(b)*sp.cos(q1)*sp.cos(q2) + 0.0905*sp.sin(a)*sp.cos(b)*sp.cos(q1)*sp.cos(q2 + q3) + 0.0225*sp.sin(a)*sp.cos(b)*sp.cos(q1) + 0.0285*sp.sin(a)*sp.cos(b) + 0.0675*sp.sin(c)*sp.sin(q1)*sp.cos(a)*sp.cos(q2) + 0.0905*sp.sin(c)*sp.sin(q1)*sp.cos(a)*sp.cos(q2 + q3) + 0.0225*sp.sin(c)*sp.sin(q1)*sp.cos(a) + 0.03997*sp.sin(c)*sp.cos(a)*sp.cos(q1) + 0.054*sp.sin(c)*sp.cos(a) - 0.0675*sp.sin(q2)*sp.cos(a)*sp.cos(c) - 0.0905*sp.sin(q2 + q3)*sp.cos(a)*sp.cos(c) - 0.1215*sp.cos(a)*sp.cos(c)
        eq_x = 1.0*p1 + 0.03997*sp.sin(b)*sp.sin(q1) - 0.0675*sp.sin(b)*sp.cos(q1)*sp.cos(q2) - 0.0905*sp.sin(b)*sp.cos(q1)*sp.cos(q2 + q3) - 0.0225*sp.sin(b)*sp.cos(q1) - 0.0285*sp.sin(b) + 0.0675*sp.sin(c)*sp.sin(q2)*sp.cos(b) + 0.0905*sp.sin(c)*sp.sin(q2 + q3)*sp.cos(b) + 0.1215*sp.sin(c)*sp.cos(b) + 0.0675*sp.sin(q1)*sp.cos(b)*sp.cos(c)*sp.cos(q2) + 0.0905*sp.sin(q1)*sp.cos(b)*sp.cos(c)*sp.cos(q2 + q3) + 0.0225*sp.sin(q1)*sp.cos(b)*sp.cos(c) + 0.03997*sp.cos(b)*sp.cos(c)*sp.cos(q1) + 0.054*sp.cos(b)*sp.cos(c)
    
    [av,bv,cv] = [0, 0, 0]       # Orientación de la IMU
    [p1v, p2v, p3v] = [0, 0, 0.4]  # ubicación del centro de referencia inicial

    eq1 = eq_x.evalf(subs={a:av, b:bv, c:cv, p1:p1v, p2:p2v, p3:p3v}) - x
    eq2 = eq_y.evalf(subs={a:av, b:bv, c:cv, p1:p1v, p2:p2v, p3:p3v}) - y
    eq3 = eq_z.evalf(subs={a:av, b:bv, c:cv, p1:p1v, p2:p2v, p3:p3v}) - z

    q = sp.nsolve((eq1, eq2, eq3), (q1, q2, q3), (1, 1, 1))
    print('Degrees: ', degrees(q[0]), degrees(q[1]), degrees(q[2]))
    return [q[0], q[1], q[2]]



def inverse_kinematics(pata, o_CoM, pos):
    # Se prueba IK Basic simulation by user Florian Wilk
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/blob/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics/Kinematic.ipynb

    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    Pt = transform_p(pos[0], pos[1], pos[2])

    # Giro de la posición original (*Florian_OK) hacia la nueva posición (*Florian_rotado)
    # respecto del frame del mundo: -90º sobre X, para poder especificar el punto destino
    # referenciado al frame del mundo

    if pata == 1:
        # PDI
        [a, b, c] = np.radians([90, 0, -180])
        T_pata = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = -1
    elif pata == 2:
        # PDD
        [a, b, c] = np.radians([-90, 0, 0])
        T_pata = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = 1
    elif pata == 3:
        # PTD
        [a, b, c] = np.radians([-90, 0, 0])
        T_pata = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = 1
    else:
        # PTI
        [a, b, c] = np.radians([90, 0, -180])
        T_pata = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = -1

    Rx = sp.Matrix([[1, 0, 0, 0], [0, cos(a), -sin(a), 0], [0, sin(a), cos(a), 0], [0, 0, 0, 1]])
    Ry = sp.Matrix([[cos(b), 0, sin(b), 0], [0, 1, 0, 0], [-sin(b), 0, cos(b), 0], [0, 0, 0, 1]])
    Rz = sp.Matrix([[cos(c), -sin(c), 0, 0], [sin(c), cos(c), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Orientación del CoM
    qx, qy, qz = o_CoM
    RoX = sp.Matrix([[1, 0, 0, 0], [0, cos(qx), -sin(qx), 0], [0, sin(qx), cos(qx), 0], [0, 0, 0, 1]])
    RoY = sp.Matrix([[cos(qy), 0, sin(qy), 0], [0, 1, 0, 0], [-sin(qy), 0, cos(qy), 0], [0, 0, 0, 1]])
    RoZ = sp.Matrix([[cos(qz), -sin(qz), 0, 0], [sin(qz), cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Se rota la traslación según los ángulos de inclinación
    # Traslación compuesta: Orientación X,Y,Z
    T = RoX * RoY * RoZ * T_pata

    # Orientación de la figura y orientación según IMU
    Pi = Rz.inv() * Rx.inv() * RoZ.inv() * RoY.inv() * RoX.inv() * (Pt - T)

    [x, y, z] = np.float64([Pi[0, 3], Pi[1, 3], Pi[2, 3]])

    F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
    G = F - l2
    H = sqrt(G ** 2 + z ** 2)

    theta1 = atan2(y, x) - atan2(F, -l1)

    D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
    theta3 = acos(D) * s

    theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))

    return ([theta1, theta2, theta3])



def IK4(o_CoM, pos):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    [p11, p12, p13, p21, p22, p23, p31, p32, p33, p41, p42, p43] = pos

    a, b, c = o_CoM

    [sa, sb, sc, ca, cb, cc] = [sin(a), sin(b), sin(c), cos(a), cos(b), cos(c)]

    [p11cbcc, p12sasbcc, p12scca, p13sasc, p13sbcacc, p12sacb, p13cacb ] = \
        [p11 * cb * cc, p12 * sa * sb * cc, p12 * sc * ca, p13 * sa * sc, p13 * sb * ca * cc,
         p12 * sa * cb, p13 * ca * cb]

    [p21cbcc, p22sasbcc, p22scca, p23sasc, p23sbcacc, p22sacb, p23cacb] = \
        [p21 * cb * cc, p22 * sa * sb * cc, p22 * sc * ca, p23 * sa * sc, p23 * sb * ca * cc,
         p22 * sa * cb, p23 * ca * cb]

    [p31cbcc, p32sasbcc, p32scca, p33sasc, p33sbcacc, p32sacb, p33cacb] = \
        [p31 * cb * cc, p32 * sa * sb * cc, p32 * sc * ca, p33 * sa * sc, p33 * sb * ca * cc,
         p32 * sa * cb, p33 * ca * cb]

    [p41cbcc, p42sasbcc, p42scca, p43sasc, p43sbcacc, p42sacb, p43cacb] = \
        [p41 * cb * cc, p42 * sa * sb * cc, p42 * sc * ca, p43 * sa * sc, p43 * sb * ca * cc,
         p42 * sa * cb, p43 * ca * cb]

    [tx, ty, tz] = [0.0540542602539063, 0.0277786254882813, 0.1212158203125]

    [x1, y1, z1, s1] = [-p11cbcc - p12sasbcc - p12scca - p13sasc + p13sbcacc + tx,
                        -p11 * sb + p12sacb - p13cacb - ty,
                        p11 * sc * cb + p12 * sa * sb * sc - p12 * ca * cc - p13 * sa * cc - p13 * sb * sc * ca - tz,
                        -1]

    [x2, y2, z2, s2] = [p21cbcc + p22sasbcc + p22scca + p23sasc - p23sbcacc + tx,
                        -p21 * sb + p22sacb - p23cacb - ty,
                        -p21 * sc * cb - p22 * sa * sb * sc + p22 * ca * cc + p23 * sa * cc + p23 * sb * sc * ca + tz,
                        1]

    [x3, y3, z3, s3] = [p31cbcc + p32sasbcc + p32scca + p33sasc - p33sbcacc + tx,
                        -p31 * sb + p32sacb - p33cacb - ty,
                        -p31 * sc * cb - p32 * sa * sb * sc + p32 * ca * cc + p33 * sa * cc + p33 * sb * sc * ca - tz,
                        -1]

    [x4, y4, z4, s4] = [-p41cbcc - p42sasbcc - p42scca - p43sasc + p43sbcacc + tx,
                        -p41 * sb + p42sacb - p43cacb - ty,
                        p41 * sc * cb + p42 * sa * sb * sc - p42 * ca * cc - p43 * sa * cc - p43 * sb * sc * ca + tz,
                        1]

    [F1, F2, F3, F4] = [sqrt(x1 ** 2 + y1 ** 2 - l1 ** 2),
                        sqrt(x2 ** 2 + y2 ** 2 - l1 ** 2),
                        sqrt(x3 ** 2 + y3 ** 2 - l1 ** 2),
                        sqrt(x4 ** 2 + y4 ** 2 - l1 ** 2)]

    [G1, G2, G3, G4] = [F1 - l2, F2 - l2, F3 - l2, F4 - l2]

    [H1, H2, H3, H4] = [sqrt(G1 ** 2 + z1 ** 2),
                        sqrt(G2 ** 2 + z2 ** 2),
                        sqrt(G3 ** 2 + z3 ** 2),
                        sqrt(G4 ** 2 + z4 ** 2)]

    [D1, D2, D3, D4] = [(H1 ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4),
                        (H2 ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4),
                        (H3 ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4),
                        (H4 ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)]

    [q11, q13, q21, q23, q31, q33, q41, q43] = [atan2(y1, x1) - atan2(F1, -l1), acos(D1) * s1,
                                                atan2(y2, x2) - atan2(F2, -l1), acos(D2) * s2,
                                                atan2(y3, x3) - atan2(F3, -l1), acos(D3) * s3,
                                                atan2(y4, x4) - atan2(F4, -l1), acos(D4) * s4]

    [q12, q22, q32, q42] = [atan2(z1, G1) - atan2(l4 * sin(q13), l3 + l4 * cos(q13)),
                            atan2(z2, G2) - atan2(l4 * sin(q23), l3 + l4 * cos(q23)),
                            atan2(z3, G3) - atan2(l4 * sin(q33), l3 + l4 * cos(q33)),
                            atan2(z4, G4) - atan2(l4 * sin(q43), l3 + l4 * cos(q43))]


    return ([q11,q12,q13, q21,q22,q23, q31,q32,q33, q41,q42,q43])



def IK(pata, o_CoM, pos):
    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    p1,p2,p3 = pos
    a,b,c = o_CoM
    [sa,sb,sc,ca,cb,cc] = [sin(a), sin(b), sin(c), cos(a), cos(b), cos(c)]
    [p1cbcc, p2sasbcc, p2scca, p3sasc, p3sbcacc] = [p1*cb*cc, p2*sa*sb*cc, p2*sc*ca, p3*sa*sc, p3*sb*ca*cc]
    [p2sacb, p3cacb] = [p2*sa*cb, p3*ca*cb]
    [tx, ty, tz] = [0.0540542602539063, 0.0277786254882813, 0.1212158203125]


    if (pata == 1):
        x = -p1cbcc - p2sasbcc - p2scca - p3sasc + p3sbcacc + tx
        y = -p1*sb + p2sacb - p3cacb - ty
        z = p1*sc*cb + p2*sa*sb*sc - p2*ca*cc - p3*sa*cc - p3*sb*sc*ca - tz
        s = -1
    if (pata == 2):
        x = p1cbcc + p2sasbcc + p2scca + p3sasc - p3sbcacc + tx
        y = -p1*sb + p2sacb - p3cacb - ty
        z = -p1*sc*cb - p2*sa*sb*sc + p2*ca*cc + p3*sa*cc + p3*sb*sc*ca + tz
        s = 1
    if (pata == 3):
        x = p1cbcc + p2sasbcc + p2scca + p3sasc - p3sbcacc + tx
        y = -p1*sb + p2sacb - p3cacb - ty
        z = -p1*sc*cb - p2*sa*sb*sc + p2*ca*cc + p3*sa*cc + p3*sb*sc*ca - tz
        s = -1
    if (pata == 4):
        x = -p1cbcc - p2sasbcc - p2scca - p3sasc + p3sbcacc + tx
        y = -p1*sb + p2sacb - p3cacb - ty
        z = p1*sc*cb + p2*sa*sb*sc - p2*ca*cc - p3*sa*cc - p3*sb*sc*ca + tz
        s = 1

    try:
        F = sqrt(x ** 2 + y ** 2 - l1 ** 2)
        G = F - l2
        H = sqrt(G ** 2 + z ** 2)

        theta1 = atan2(y, x) - atan2(F, -l1)
        D = (H ** 2 - l3 ** 2 - l4 ** 2) / (2 * l3 * l4)
        theta3 = acos(D) * s
        theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))
    except:
        [theta1, theta2, theta3] = [None, None, None]
        print ("IK exception:", sys.exc_info()[0])
        __raise_exception()

    return ([theta1, theta2, theta3])


def symbolic_inverse_kinematics (pata):
    # Se prueba IK Basic simulation by user Florian Wilk
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/tree/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics
    # https://gitlab.com/custom_robots/spotmicroai/simulation/-/blob/master/Basic%20simulation%20by%20user%20Florian%20Wilk/Kinematics/Kinematic.ipynb

    l1 = 39.97e-3
    l2 = 22.5e-3
    l3 = 67.5e-3
    l4 = 90.5e-3

    [p1,p2,p3] = sp.symbols('p1,p2,p3')
    Pt = sp.Matrix([[1, 0, 0, p1],
                   [0, 1, 0, p2],
                   [0, 0, 1, p3],
                   [0, 0, 0, 1]])


    # Giro de la posición original (*Florian_OK) hacia la nueva posición (*Florian_rotado)
    # respecto del frame del mundo: -90º sobre X, para poder especificar el punto destino
    # referenciado al frame del mundo

    if pata == 1:
        # PDI
        [a, b, c] = np.radians([90, 0, -180])
        T_pata = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = -1
    elif pata == 2:
        # PDD
        [a, b, c] = np.radians([-90, 0, 0])
        T_pata = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, -121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = 1
    elif pata == 3:
        # PTD
        [a, b, c] = np.radians([-90, 0, 0])
        T_pata = sp.Matrix([[1, 0, 0, -54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = 1
    else:
        # PTI
        [a, b, c] = np.radians([90, 0, -180])
        T_pata = sp.Matrix([[1, 0, 0, +54e-3], [0, 1, 0, +121.5e-3], [0, 0, 1, -28.5e-3], [0, 0, 0, 1]])
        s = -1

    Rx = sp.Matrix([[1, 0, 0, 0], [0, cos(a), -sin(a), 0], [0, sin(a), cos(a), 0], [0, 0, 0, 1]])
    Ry = sp.Matrix([[cos(b), 0, sin(b), 0], [0, 1, 0, 0], [-sin(b), 0, cos(b), 0], [0, 0, 0, 1]])
    Rz = sp.Matrix([[cos(c), -sin(c), 0, 0], [sin(c), cos(c), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Orientación del CoM
    [qx, qy, qz] = sp.symbols('qx,qy,qz')
    RoX = sp.Matrix([[1, 0, 0, 0], [0, sp.cos(qx), -sp.sin(qx), 0], [0, sp.sin(qx), sp.cos(qx), 0], [0, 0, 0, 1]])
    RoY = sp.Matrix([[sp.cos(qy), 0, sp.sin(qy), 0], [0, 1, 0, 0], [-sp.sin(qy), 0, sp.cos(qy), 0], [0, 0, 0, 1]])
    RoZ = sp.Matrix([[sp.cos(qz), -sp.sin(qz), 0, 0], [sp.sin(qz), sp.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # Se rota la traslación según los ángulos de inclinación
    # Traslación compuesta: Orientación X,Y,Z
    T = RoX * RoY * RoZ * T_pata

    # Orientación de la figura y orientación según IMU
    Pi = Rz.inv() * Rx.inv() * RoZ.inv() * RoY.inv() * RoX.inv() * (Pt - T)

    P = sp.Matrix([[Pi[0, 3], Pi[1, 3], Pi[2, 3]]])
    [p0,p1,p2] = [round_expr(P[0],3),round_expr(P[1],3),round_expr(P[2],3)]
    [p0s,p1s,p2s] = [sp.simplify(p0),sp.simplify(p1),sp.simplify(p2)]
    print ("x=",p0s,"\ny=",p1s,"\nz=",p2s)

    return

def round_expr(expr, num_digits):
    return expr.xreplace({n : round(n, num_digits) for n in expr.atoms(sp.Number)})
