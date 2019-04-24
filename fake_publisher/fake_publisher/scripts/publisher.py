#!/usr/bin/env python

import numpy as np
import math
import random
from numpy.linalg import inv
from math import sqrt
import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import *
from std_msgs.msg import String
from pozyx_drivers.msg import AnchorInfo 
from dynamic_reconfigure.server import Server
from fake_publisher.cfg import ErrorConfig


#variables para simulacion del pozyxs

r_error = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
position_error_a = np.array([0.0, 0.0, 0.0, 1.0])
position_error_b = np.array([0.0, 0.0, 0.0, 1.0])
position_error_c = np.array([0.0, 0.0, 0.0, 1.0])
position_error_d = np.array([0.0, 0.0, 0.0, 1.0])
position_error_e = np.array([0.0, 0.0, 0.0, 1.0])
position_error_f = np.array([0.0, 0.0, 0.0, 1.0])
position_error_g = np.array([0.0, 0.0, 0.0, 1.0])
position_error_h = np.array([0.0, 0.0, 0.0, 1.0])
desviacion_tipica_range = 0
desviacion_tipica_posicion = 0


def loop():

    global a,b,c,d,e,f,g,h

    #Topic: Anchors Info
    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(1)
    dr.position.x = (float)(a[0])
    dr.position.y = (float)(a[1])
    dr.position.z = (float)(a[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[0]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[0]
    dr.child_frame_id = "anchor_0"
    pub_anchor1_info.publish(dr)
   

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(2)
    dr.position.x = (float)(b[0])
    dr.position.y = (float)(b[1])
    dr.position.z = (float)(b[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[1]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[1]
    dr.child_frame_id = "anchor_1"
    pub_anchor2_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(3)
    dr.position.x = (float)(c[0])
    dr.position.y = (float)(c[1])
    dr.position.z = (float)(c[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[2]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[2]
    dr.child_frame_id = "anchor_2"
    pub_anchor3_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(4)
    dr.position.x = (float)(d[0])
    dr.position.y = (float)(d[1])
    dr.position.z = (float)(d[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[3]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[3]
    dr.child_frame_id = "anchor_3"
    pub_anchor4_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(5)
    dr.position.x = (float)(e[0])
    dr.position.y = (float)(e[1])
    dr.position.z = (float)(e[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[4]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[4]
    dr.child_frame_id = "anchor_4"
    pub_anchor5_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(6)
    dr.position.x = (float)(f[0])
    dr.position.y = (float)(f[1])
    dr.position.z = (float)(f[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[5]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[5]
    dr.child_frame_id = "anchor_5"
    pub_anchor6_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(7)
    dr.position.x = (float)(g[0])
    dr.position.y = (float)(g[1])
    dr.position.z = (float)(g[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[6]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[6]
    dr.child_frame_id = "anchor_6"
    pub_anchor7_info.publish(dr)

    dr = AnchorInfo()
    dr.header.stamp = rospy.get_rostime()
    dr.header.frame_id = 'world'
    dr.id = hex(8)
    dr.position.x = (float)(h[0])
    dr.position.y = (float)(h[1])
    dr.position.z = (float)(h[2])
    dr.position_cov = [desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion, 0.0, 0.0, 0.0, desviacion_tipica_posicion]
    dr.status = True
    dr.distance = r[7]
    dr.distance_cov = desviacion_tipica_range
    dr.RSS = RSS[7]
    dr.child_frame_id = "anchor_7"
    pub_anchor8_info.publish(dr)

def callback(config, level):
    #rospy.loginfo("""Reconfigure Request: {double_param}""".format(**config))
    global a,b,c,d,e,f,g,h,random,uniforme,RSS,desviacion_tipica_posicion,desviacion_tipica_range
    random = config['random']
    uniforme= config['uniforme']
    if random == False:
        print("Numeros elegidos por el usuario")
        r_error[0] = config['r1_error']
        r_error[1] = config['r2_error']
        r_error[2] = config['r3_error']
        r_error[3] = config['r4_error']
        r_error[4] = config['r5_error']
        r_error[5] = config['r6_error']
        r_error[6] = config['r7_error']
        r_error[7] = config['r8_error']
        position_error_a = config['a_error']
        position_error_b = config['b_error']
        position_error_c = config['c_error']
        position_error_d = config['d_error']
        position_error_e = config['e_error']
        position_error_f = config['f_error']
        position_error_g = config['g_error']
        position_error_h = config['h_error']
        desviacion_tipica_range = 0
        desviacion_tipica_posicion = 0
    else:
        if uniforme == True:
            print("Numeros aleatorios uniformes")
            min_distancia = config['min_distancia']#siempre negativo
            max_distancia = config['max_distancia']#siempre positivo
            suma_distancia = -min_distancia + max_distancia
            min_pos = config['min_pos']#siempre negativo
            max_pos = config['max_pos']#siempre positivo
            suma_pos = -min_pos + max_pos
            r_error[0] = min_distancia + np.random.rand()*suma_distancia
            r_error[1] = min_distancia + np.random.rand()*suma_distancia
            r_error[2] = min_distancia + np.random.rand()*suma_distancia
            r_error[3] = min_distancia + np.random.rand()*suma_distancia
            r_error[4] = min_distancia + np.random.rand()*suma_distancia
            r_error[5] = min_distancia + np.random.rand()*suma_distancia
            r_error[6] = min_distancia + np.random.rand()*suma_distancia
            r_error[7] = min_distancia + np.random.rand()*suma_distancia
            position_error_a = min_pos + np.random.rand(4)*suma_pos
            position_error_b = min_pos + np.random.rand(4)*suma_pos
            position_error_c = min_pos + np.random.rand(4)*suma_pos
            position_error_d = min_pos + np.random.rand(4)*suma_pos
            position_error_e = min_pos + np.random.rand(4)*suma_pos
            position_error_f = min_pos + np.random.rand(4)*suma_pos
            position_error_g = min_pos + np.random.rand(4)*suma_pos
            position_error_h = min_pos + np.random.rand(4)*suma_pos
            desviacion_tipica_range = 0
            desviacion_tipica_posicion = 0
        else:
            print("Numeros aleatorios gaussianos")
            media_range = config['media_range']
            desviacion_tipica_range = config['desviacion_tipica_range']
            media_posicion = config['media_posicion']
            desviacion_tipica_posicion = config['desviacion_tipica_posicion']
            r_error[0] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[1] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[2] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[3] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[4] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[5] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[6] = np.random.normal(media_range, desviacion_tipica_range)
            r_error[7] = np.random.normal(media_range, desviacion_tipica_range)
            position_error_a = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_b = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_c = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_d = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_e = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_f = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_g = np.random.normal(media_posicion, desviacion_tipica_posicion)
            position_error_h = np.random.normal(media_posicion, desviacion_tipica_posicion)
    r[0] = r_sinerror[0] + r_error[0]
    r[1] = r_sinerror[1] + r_error[1]
    r[2] = r_sinerror[2] + r_error[2]
    r[3] = r_sinerror[3] + r_error[3]
    r[4] = r_sinerror[4] + r_error[4]
    r[5] = r_sinerror[5] + r_error[5]
    r[6] = r_sinerror[6] + r_error[6]
    r[7] = r_sinerror[7] + r_error[7]
    a = a_sinerror + position_error_a
    a[3] = 1 
    b = b_sinerror + position_error_b
    b[3] = 1
    c = c_sinerror + position_error_c
    c[3] = 1
    d = d_sinerror + position_error_d
    d[3] = 1
    e = e_sinerror + position_error_e
    e[3] = 1
    f = f_sinerror + position_error_f
    f[3] = 1
    g = g_sinerror + position_error_g
    g[3] = 1
    h = h_sinerror + position_error_h
    h[3] = 1
    RSS[0] = config['RSS0']
    RSS[1] = config['RSS1']
    RSS[2] = config['RSS2']
    RSS[3] = config['RSS3']
    RSS[4] = config['RSS4']
    RSS[5] = config['RSS5']
    RSS[6] = config['RSS6']
    RSS[7] = config['RSS7']

    return config


if __name__ == "__main__":

    rospy.init_node('publisher')

    # Inicializar parametros

    a = np.array(eval(rospy.get_param('~a')))
    b = np.array(eval(rospy.get_param('~b')))
    c = np.array(eval(rospy.get_param('~c')))
    d = np.array(eval(rospy.get_param('~d')))
    e = np.array(eval(rospy.get_param('~e')))
    f = np.array(eval(rospy.get_param('~f')))
    g = np.array(eval(rospy.get_param('~g')))
    h = np.array(eval(rospy.get_param('~h')))

    r = np.array(eval(rospy.get_param('~r')))

    random = rospy.get_param('~random')
    normal = rospy.get_param('~normal')

    RSS = np.array(eval(rospy.get_param('~RSS')))

    # Variables para el programa
    r_sinerror = np.array(r, copy = True)
    
    a_sinerror = np.array(a, copy = True)
    b_sinerror = np.array(b, copy = True)
    c_sinerror = np.array(c, copy = True)
    d_sinerror = np.array(d, copy = True)
    e_sinerror = np.array(e, copy = True)
    f_sinerror = np.array(f, copy = True)
    g_sinerror = np.array(g, copy = True)
    h_sinerror = np.array(h, copy = True)


    # Creando publicadores
    pub_anchor1_info = rospy.Publisher('~anchor_info_0', AnchorInfo, queue_size=1)
    pub_anchor2_info = rospy.Publisher('~anchor_info_1', AnchorInfo, queue_size=1)
    pub_anchor3_info = rospy.Publisher('~anchor_info_2', AnchorInfo, queue_size=1)
    pub_anchor4_info = rospy.Publisher('~anchor_info_3', AnchorInfo, queue_size=1)
    pub_anchor5_info = rospy.Publisher('~anchor_info_4', AnchorInfo, queue_size=1)
    pub_anchor6_info = rospy.Publisher('~anchor_info_5', AnchorInfo, queue_size=1)
    pub_anchor7_info = rospy.Publisher('~anchor_info_6', AnchorInfo, queue_size=1)
    pub_anchor8_info = rospy.Publisher('~anchor_info_7', AnchorInfo, queue_size=1)
    rate = rospy.Rate(5)
    srv = Server(ErrorConfig, callback)  

    while not rospy.is_shutdown():
        loop()
        rate.sleep()
