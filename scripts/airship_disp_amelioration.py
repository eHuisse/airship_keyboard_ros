#!/usr/bin/env python

import sys
sys.path.insert(0,"/home/edouard/.local/lib/python2.7/site-packages")
import math
import numpy as np
import navpy
import copy


import matplotlib.pyplot as plt
import rospy
from airship_keyboard.msg import MotorCommand
from geometry_msgs.msg import Pose


X = []
Y = []
Z = []
PSI = []
THETA = []
PHI = []
ACC_X = []
VITESSE = []

# initial values
u = 0.0001  # axial velocity following the x axis (m/s)
v = 0.0001 # axial velocity following the y axis (m/s)
w = 0.0001 # axial velocity following the z axis (m/s)
p = 0.0001  # angular velocity following the psi angle (rad/s)
q = 0.0001  # angular velocity following the theta angle (rad/s)
r = 0.0001  # angular velocity following the phi angle (rad/s)
V0 = np.array([0, 0, 0, 0, 0, 0])

Vtotal = math.sqrt(u**2+v**2+w**2)

x = -6  # initial position following the x axis (m)
y = 0  # initial position following the y axis (m)
z = 2  # initial position following the z axis (m)
phi = math.radians(0)  # initial psi angle value (rad)
theta = math.radians(0)  # initial theta angle value (rad)
psi = math.radians(-90)  # initial phi theta value (rad)

C = 0

N = 2000

m = 0.950

rudder = math.radians(-0)
elevator = math.radians(0)
t_d = 0
t_g = 0

X.append(x)  # vector of the x position
Y.append(y)  # vector of the y position
Z.append(z)  # vector of the z position
PSI.append(psi)
THETA.append(theta)
PHI.append(phi)
VITESSE.append(Vtotal)

def DCM(phi, theta, psi):

    a1 = np.array([[math.cos(theta)*math.cos(psi), -math.cos(phi)*math.sin(psi) + math.sin(phi)*math.sin(theta)*math.cos(psi), math.sin(phi)*math.sin(psi)+math.cos(phi)*math.sin(theta)*math.cos(psi)],
                   [math.cos(theta)*math.sin(psi), math.cos(phi)*math.cos(psi) + math.sin(phi)*math.sin(theta)*math.sin(psi), -math.sin(phi)*math.cos(psi)+math.cos(phi)*math.sin(theta)*math.sin(psi)],
                   [-math.sin(theta), math.sin(phi)*math.cos(theta), math.cos(phi)*math.cos(theta)]])

    return a1

def angularDCM(phi, theta, psi):
    a1 = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                   [0, np.cos(phi), -np.sin(phi)],
                   [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
    return a1

############################################################
rospy.init_node("simulation")
motor_command = MotorCommand()
simulated_pose = Pose()

def call_motor_command(msg):
    global t_d
    global t_g
    global elevator
    global rudder

    t_g = msg.left_motor
    t_d = msg.right_motor

    elevator = msg.tail_yaw
    rudder = msg.tail_pitch
    rospy.loginfo("TESSSSSSSSSSSSSSSSSSSSSSSt" + str(t_d) +str(t_g)+str(elevator)+str(rudder))

pose_pub = rospy.Publisher("sim_pose", Pose, queue_size=10)
rospy.Subscriber("/hardcom/motorcommand", MotorCommand, call_motor_command)

############################################################

tmp_prev_time = rospy.Time.now()
prev_time = tmp_prev_time.secs + tmp_prev_time.nsecs * 1e-9

while not rospy.is_shutdown():
    # Definition temporelle
    tmp_time_now = rospy.Time.now()
    time_now = tmp_time_now.secs + tmp_time_now.nsecs * 1e-9
    delta_t = time_now - prev_time
    prev_time = time_now

    # calculate the propulsion vector
    mu = math.radians(0)
    Xprop = (t_d + t_g)*math.cos(mu)
    Yprop = 0
    Zprop = (t_g + t_g)*math.sin(mu)
    Lprop = 0
    Mprop = 0
    Nprop = 0
    P = np.array([Xprop, Yprop, Zprop, Lprop, Mprop, Nprop]).reshape((6, 1))

    a1 = - u * abs(u) * 0.5
    a2 = - v * abs(v) * 1
    a3 = - w * abs(w) * 1
    a4 = 0
    a5 = elevator * 0.3 - 0.5 * q - 1 * np.sin(theta/2)
    a6 = rudder * 0.1 - 0.5 * r
    A = np.array([a1, a2, a3, a4, a5, a6]).reshape((6, 1))
    Acc = 1 / m * (P + A)

    # calculate the  velocity vector

    # update velocity vector
    u = float(u + Acc[0]*delta_t)
    v = float(v + Acc[1]*delta_t)
    w = float(w + Acc[2]*delta_t)
    p = float(p + Acc[3]*delta_t)
    q = float(q + Acc[4]*delta_t)
    r = float(r + Acc[5]*delta_t)

    V = np.array([u, v, w, p, q, r]).reshape((6, 1)) 

    dcm = DCM(phi, theta, psi)
    R = angularDCM(phi, theta, psi)

    ConvC1 = np.concatenate((dcm, np.zeros((3,3))), axis=0)
    ConvC2 = np.concatenate((np.zeros((3,3)), R), axis=0)
    confu = np.concatenate((ConvC1, ConvC2), axis=1)

    V = np.dot(confu, V)

#    V[0] = V[0] * np.cos(theta) * np.cos(psi) - V[1] * np.sin(psi)#np.dot(confu, V)
#    V[1] = V[0] * np.sin(psi) * np.cos(theta) + V[1] * np.sin(psi)#
#    V[2] = V[0] * np.sin(theta) + V[2] * np.cos(theta)
#    V[3] = V[3] * np.cos(theta) * np.cos(psi) - V[4] * np.sin(psi)#np.dot(confu, V)
#    V[4] = V[3] * np.sin(psi) * np.cos(theta) + V[4] * np.sin(psi)
#    V[5] = V[3] * np.sin(theta) + V[5] * np.cos(theta)


   #if elevator==0:
   #     V[4] = 0

    #if rudder==0:
    #    V[5] = 0
    

    # cartesian and angular coordonates
    x = x + V[0]*delta_t
    y = y + V[1]*delta_t
    z = z + V[2]*delta_t
    phi = 0
    theta = (theta + V[4]*delta_t)
    psi = (psi + V[5]*delta_t)

    Vtotal = math.sqrt(u**2+v**2+w**2)

    C = C + 1

    #print('C = ', C)

    X.append(x)
    Y.append(y)
    Z.append(z)
    PSI.append(psi)
    THETA.append(theta)
    PHI.append(phi)
    VITESSE.append(Vtotal)


    simulated_pose.position.x = x
    simulated_pose.position.y = y
    simulated_pose.position.z = z

    q0, qa = navpy.angle2quat(psi, theta, phi, rotation_sequence='ZYX')
    simulated_pose.orientation.x = qa[0]
    simulated_pose.orientation.y = qa[1]
    simulated_pose.orientation.z = qa[2]
    simulated_pose.orientation.w = q0

    pose_pub.publish(simulated_pose)





# plot figure

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

ax.scatter(X, Y, Z, c='r', marker='o')
plt.title('deplacement')
plt.show()

fig2 = plt.figure()
plt.plot(PSI, c='r', marker='o')
plt.title('PSI')
plt.show()

fig3 = plt.figure()
plt.plot(THETA, c='r', marker='o')
plt.title('THETA')
plt.show()

fig4 = plt.figure()
plt.plot(PHI, c='r', marker='o')
plt.title('PHI')
plt.show()

fig5 = plt.figure()
plt.plot(ACC_X, c='r', marker='o')
plt.title('Acceleration')
plt.show()

fig6 = plt.figure()
plt.plot(VITESSE, c='r', marker='o')
plt.title('Vitesse')
plt.show()
