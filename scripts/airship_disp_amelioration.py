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
u = 0.1  # axial velocity following the x axis (m/s)
v = 0  # axial velocity following the y axis (m/s)
w = 0  # axial velocity following the z axis (m/s)
p = 0  # angular velocity following the psi angle (rad/s)
q = 0  # angular velocity following the theta angle (rad/s)
r = 0  # angular velocity following the phi angle (rad/s)

x = 0  # initial position following the x axis (m)
y = 0  # initial position following the y axis (m)
z = 2  # initial position following the z axis (m)
psi = math.radians(0)  # initial psi angle value (rad)
theta = math.radians(0)  # initial theta angle value (rad)
phi = math.radians(0)  # initial phi theta value (rad)
eul = (psi, theta, phi)  # euler vector

X.append(x)  # vector of the x position
Y.append(y)  # vector of the y position
Z.append(z)  # vector of the z position

# Airship geometry and coefficient values
L = 2.75  # length (m)
d = 0.5  # diameter (m)
Sh = 4  # Hull reference area
Sf = 0.172   # Fin reference area
Sg = 0.0025  # Gondola reference area
nf = 0.4  # Fin efficiency factor
nk = 1.0  # Hull efficiency factor
Cdho = 0.024  # Hull zero_incidence drag coefficient
Cdfo = 0.003  # Fin zero-incidence drag coefficient
Cdgo = 0.01  # Gondola zero-incidence drag coefficient
Cdch = 0.32  # Hull cross-flow drag coefficient
Cdcf = 2  # Fin cross-flow drag coefficient
Cdcg = 0.25  # Gondola cross-flow drag coefficient
dCl_dalpha_f = 5.73  # Derivative of fin lift coefficient with respect to angle of attack
dCl_ddelta_f = 1.24  # Derivative of fin lift coefficient with respect to fin angle
k1 = 0.10  # Lamb's inertia ratio about X
k2 = 0.82  # Lamb's inertia ratio about Y or Z
k_p = 0.51  # Lamb's inertia ratio about Y or Z
dgy = 0.01  # CV to gondola CG distance along y
dgz = 0.27  # CV to gondola CG distance along z
dfx = 0.8  # CV to fin center distance along x
dfz = 0.27  # CV to fin center distance along z

# another index
# a1 = L / 2
# a2 = L / 2
# xcv = a1 - (4*math.pi/3)*(a1-a2)
b = d/2
V = 1
rho = 1.292
g = 9.81  # constant of gravitation
H = 15
s = 0.85

ax = 0.2
ay = 0
az = 0.2
Ix = 0.045
Iy = 0.229
Iz = 0.229

I1 = 0
I3 = -1.0839
J1 = 1.7897
J2 = 0.6809

# Aerodynamic coefficient

Cx1 = -(Cdho*Sh + Cdfo*Sf + Cdgo*Sg)
Cx2 = (k2 - k1)*nk*I1*Sh
# Cx3 = Ctf*Sf
Cy1 = -Cx2
Cy2 = -0.5*(dCl_dalpha_f*Sf*nf)
Cy3 = -(Cdch*J1*Sh + Cdcf*Sf + Cdcg*Sg)
Cy4 = -0.5*dCl_ddelta_f*Sf*nf
Cz1 = -Cx2
Cz2 = Cy2
Cz3 = -(Cdch*J1*Sh + Cdcf*Sf)
Cz4 = Cy4
C_psi_1 = Cdcg*Sg*dgz
C_psi_2 = -2*Cdcf*Sf*dfz**3
C_psi_3 = -Cdcg*Sg*dgz*d**2
C_theta_1 = (k2-k1)*nk*I3*Sh*L
C_theta_2 = -0.5*dCl_ddelta_f*Sf*dfx
C_theta_3 = -(Cdch*J2*Sh*L+Cdcf*Sf*dfx)
C_theta_4 = -0.5*1.24*Sf*dfx
C_theta_5 = -Cdcf*Sf*dfx
C_phi_1 = -C_theta_1
C_phi_2 = -C_theta_2
C_phi_3 = Cdch*J2*Sh*L+Cdcf*Sf*dfx+Cdcg*Sg*s
C_phi_4 = -C_theta_4
C_phi_5 = -(Cdcf*Sf*dfx**3-Cdcg*Sg*s**3)

D_e = math.radians(0)  # elevator fin angle
D_r = math.radians(0)  # rudder fin angle

# detail of the airship mass
m_e = 0.400  # envelope mass
m_g = 0.121  # gondola mass
m_r = 0.019  # rail mass
m = m_e + m_g + m_r  # total mass of the airship
mx = (1+k1)*m
my = (1+k2)*m
mz = my

Ifxz = 0
Ifx = 0

#  gondola inertia
Igx = dgz**2
Igy = math.sqrt(s**2+dgz**2)
Igz = s**2
Igxz = -s*dgz

# envelop and rail inertia
If_y = 0
If_z = 0

# inertia

Jx = Ifx+Igx
Jy = (1+k_p)*If_y + Igy
Jz = (1+k_p)*If_z + Igz
Jxz = Ifxz + Igxz

dm_x = s*(m_g/m)
dm_z = (m_r+m_g)/m

#  calculate mass matrix (supposed to be constant, we can include virtual mass)
rcg = np.transpose([dm_x, 0, dm_z])
dcg = np.array([[0, -dm_z, 0], [dm_z, 0, -dm_x], [0, dm_x, 0]])
M = np.array([[mx, 0, 0, 0, m*dm_z, 0], [0, my, 0, -m*dm_z, 0, m*dm_x], [0, 0, mz, 0, -m*dm_x, 0], [0, -m*dm_z, 0, Jx, 0, -Jxz], [m*dm_z, 0, -m*dm_z, 0, Jy, 0], [0, m*dm_x, 0, -Jxz, 0, Jz]])
M_inv = np.linalg.inv(M)

#  calculate the dynamic vector
f1 = -mz*w*q + my*r*v + m*(ax*(q**2+r**2) - az*r*p)
f2 = -m*u*r + m*p*w + m*(-ax*p*q - az*r*q)
f3 = -m*v*p + m*q*u + m*(-ax*r*p + az*(q**2+p**2))
f4 = -(Jz - Jy)*r*q + Jxz*p*q + m*az*(u*r-p*w)
f5 = -(Jx-Jz)*p*r + Jxz*(r**2-p**2) + m*(ax*(v*p - q*u)-az*(w*q - r*v))
f6 = -(Jy - Jx)*q*p - Jxz*q*r + m*(-ax*(u*r - p*w))
Fd = np.array([[f1], [f2], [f3], [f4], [f5], [f6]])

# calculate the propulsion vector
mu = math.radians(0)  # motor angle
T_r = 0.00001
T_l = 0.00001
Xprop = (T_r + T_l)*math.cos(mu)
Yprop = 0
Zprop = -(T_r + T_l)*math.sin(mu)
Lprop = 0
Mprop = (T_r + T_l)*dgz*math.cos(mu)
Nprop = (T_l - T_r)*dgy*math.cos(mu)
P = np.array([[Xprop], [Yprop], [Zprop], [Lprop], [Mprop], [Nprop]])

alpha = np.arctan(w/u)
Vtotal = math.sqrt(u**2+v**2+w**2)
beta = np.arcsinh(v/Vtotal)

# Aerodynamic vector
q_dyn = 0.5*rho*Vtotal**2*Sh
Ax = q_dyn*(Cx1*math.cos(alpha)**2*math.cos(beta)**2+Cx2*(math.sin(2*alpha)*math.sin(alpha/2)+math.sin(2*beta)*math.sin(beta/2)))
Ay = q_dyn*(Cy1*math.cos(beta/2)*math.sin(2*beta)+Cy2*math.sin(2*beta)+Cy3*math.sin(beta)*math.sin(abs(beta))+Cy4*(2*D_r))
Az = q_dyn*(Cz1*math.cos(alpha/2)*math.sin(2*alpha)+Cz2*math.sin(2*alpha)+Cz3*math.sin(alpha)*math.sin(abs(alpha))+Cz4*(2*D_e))
Al = q_dyn*(C_psi_1*math.sin(beta)*math.sin(abs(beta))+0.5*C_psi_3*rho*r*abs(r)+0.5*C_psi_3*rho*p*abs(p))
Am = q_dyn*(C_theta_1*math.cos(alpha/2)*math.sin(2*alpha)+C_theta_2*math.sin(2*alpha)+C_theta_3*math.sin(alpha)*math.sin(abs(alpha))+C_theta_4*(2*D_e))+0.5*rho*C_theta_5*r*abs(r)
An = q_dyn*(C_phi_1*math.cos(beta/2)*math.sin(2*beta)+C_phi_2*math.sin(2*beta)+C_phi_3*math.sin(beta)*math.sin(abs(beta))+C_phi_4*(2*D_r)) + 0.5*rho*C_phi_5*p*abs(p)
A = np.array([[Ax], [Ay], [Az], [Al], [Am], [An]])

# calculate gravity and buoyancy vector G
dcm = navpy.angle2dcm(rotAngle1=float(eul[2]), rotAngle2=float(eul[1]), rotAngle3=float(eul[2]), input_unit='rad', rotation_sequence='ZYX', output_type='ndarray')
g_mat = np.dot(np.transpose(dcm), np.array([[0], [0], [g]]))
U = m/rho
G = np.insert(m*g_mat, 3, m*np.dot(dcg, g_mat), axis=0) - np.insert(rho*U*g_mat, 3, rho*U*np.dot(dcg, g_mat), axis=0)

# time interval for the simulation
delta_t = 0.1

# calculate the Acceleration vector
Acc = np.dot(M_inv, Fd + P + A + G)

# calculate the  velocity vector
u1 = float(np.trapz([0, float(Acc[0])], dx=delta_t))
v1 = float(np.trapz([0, float(Acc[1])], dx=delta_t))
w1 = float(np.trapz([0, float(Acc[2])], dx=delta_t))
p1 = float(np.trapz([0, float(Acc[3])], dx=delta_t))
q1 = float(np.trapz([0, float(Acc[4])], dx=delta_t))
r1 = float(np.trapz([0, float(Acc[5])], dx=delta_t))

# calculate position and angular added vector
x_1 = float(np.trapz([u, u1], dx=delta_t))
y_1 = float(np.trapz([v, v1], dx=delta_t))
z_1 = float(np.trapz([w, w1], dx=delta_t))
psi_1 = float(np.trapz([p, p1], dx=delta_t))
theta_1 = float(np.trapz([q, q1], dx=delta_t))
phi_1 = float(np.trapz([r, r1], dx=delta_t))

# update acceleration vector
Acc_0 = Acc

# update velocity vector
u = u1
v = v1
w = w1
p = p1
q = q1
r = r1

# update position and angular vector
x = x + x_1
y = y + y_1
z = z + z_1
psi = psi + psi_1
theta = theta + theta_1
phi = phi + phi_1

# add position 
X.append(x)
Y.append(y)
Z.append(z)
PSI.append(psi)
THETA.append(theta)
PHI.append(phi)
ACC_X.append(float(Acc[0]))
VITESSE.append(Vtotal)


C = 1
N = 4000  # number of iteration

############################################################
rospy.init_node("simulation")
motor_command = MotorCommand()
simulated_pose = Pose()

def call_motor_command(msg):
    global T_l
    global T_r
    global D_e
    global D_r

    T_l = msg.left_motor
    T_r = msg.right_motor

    D_e = msg.tail_yaw
    D_r = msg.tail_pitch
    rospy.loginfo("_______________________TESSSSSSSSSSSSSSSSSSSSSSSt___________________________" + str(T_l) +str(T_r)+str(D_e)+str(D_r))

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
    Xprop = (T_r + T_l)*math.cos(mu)
    Yprop = 0
    Zprop = -(T_r + T_l)*math.sin(mu)
    Lprop = 0
    Mprop = (T_r + T_l)*dgz*math.cos(mu)
    Nprop = (T_l - T_r)*dgy*math.cos(mu)
    PV = np.array([[Xprop], [Yprop], [Zprop], [Lprop], [Mprop], [Nprop]])

    # calculate the dynamic vector
    f1 = - mz*w*q + my*r*v + m*(ax*(q**2+r**2) - az*r*p)
    f2 = - m*u*r + m*p*w + m*(-ax*p*q - az*r*q)
    f3 = - m*v*p + m*q*u + m*(-ax*r*p + az*(q**2+p**2))
    f4 = -(Jz - Jy)*r*q + Jxz*p*q + m*az*(u*r-p*w)
    f5 = -(Jx-Jz)*p*r + Jxz*(r**2-p**2) + m*(ax*(v*p - q*u)-az*(w*q - r*v))
    f6 = -(Jy - Jx)*q*p - Jxz*q*r + m*(-ax*(u*r - p*w))
    FdV = np.array([[f1], [f2], [f3], [f4], [f5], [f6]])

    alpha = np.arctan(w/u)
    Vtotal = math.sqrt(u**2+v**2+w**2)
    beta = np.arcsinh(v/Vtotal)

    # Aerodynamic vector
    q_dyn = 0.5*rho*Vtotal**2*Sh
    Ax = q_dyn*(Cx1*math.cos(alpha)**2*math.cos(beta)**2+Cx2*(math.sin(2*alpha)*math.sin(alpha/2)+math.sin(2*beta)*math.sin(beta/2)))
    Ay = q_dyn*(Cy1*math.cos(beta/2)*math.sin(2*beta) + Cy2*math.sin(2*beta)+Cy3*math.sin(beta)*math.sin(abs(beta))+Cy4*(2*D_r))
    Az = q_dyn*(Cz1*math.cos(alpha/2)*math.sin(2*alpha)+Cz2*math.sin(2*alpha)+Cz3*math.sin(alpha)*math.sin(abs(alpha))+Cz4*(2*D_e))
    Al = q_dyn*(C_psi_1*math.sin(beta)*math.sin(abs(beta))+0.5*C_psi_3*rho*r*abs(r)+0.5*C_psi_3*rho*p*abs(p))
    Am = q_dyn*(C_theta_1*math.cos(alpha/2)*math.sin(2*alpha)+C_theta_2*math.sin(2*alpha)+C_theta_3*math.sin(alpha)*math.sin(abs(alpha))+C_theta_4*(2*D_e))+0.5*rho*C_theta_5*r*abs(r)
    An = q_dyn*(C_phi_1*math.cos(beta/2)*math.sin(2*beta)+C_phi_2*math.sin(2*beta)+C_phi_3*math.sin(beta*math.sin(abs(beta))+C_phi_4*(2*D_r)))+0.5*rho*C_phi_5*p*abs(p)
    AV = np.array([[Ax], [Ay], [Az], [Al], [Am], [An]])

    eul = (psi, theta, phi)
    dcm = navpy.angle2dcm(rotAngle1=float(eul[2]), rotAngle2=float(eul[1]), rotAngle3=float(eul[2]), input_unit='rad', rotation_sequence='ZYX', output_type='ndarray')
    g_mat = np.dot(np.transpose(dcm), np.array([[0], [0], [g]]))
    U = m/rho
    #G = np.insert(m*g_mat, 3, m*np.dot(dcg, g_mat), axis=0) - np.insert(rho*U*g_mat, 3, rho*U*np.dot(dcg, g_mat), axis=0)
    Acc = np.dot(M_inv, FdV + PV + AV + G)

    rospy.loginfo("_______________________TIME___________________________" + str(T_l) +str(T_r)+str(D_e)+str(D_r))
    # calculate the  velocity vector
    u1 = float(np.trapz([float(Acc_0[0]), Acc[0]], dx=delta_t))
    v1 = float(np.trapz([float(Acc_0[1]), Acc[1]], dx=delta_t))
    w1 = float(np.trapz([float(Acc_0[2]), Acc[2]], dx=delta_t))
    p1 = float(np.trapz([float(Acc_0[3]), Acc[3]], dx=delta_t))
    q1 = float(np.trapz([float(Acc_0[2]), Acc[4]], dx=delta_t))
    r1 = float(np.trapz([float(Acc_0[3]), Acc[5]], dx=delta_t))

    # calculate position and angular added vector

    x_1 = float(np.trapz([u, u1], dx=delta_t))
    y_1 = float(np.trapz([v, v1], dx=delta_t))
    z_1 = float(np.trapz([w, w1], dx=delta_t))
    psi_1 = float(np.trapz([w, w1], dx=delta_t))
    theta_1 = float(np.trapz([q, q1], dx=delta_t))
    phi_1 = float(np.trapz([r, r1], dx=delta_t))


    # update acceleration vector
    Acc_0 = Acc

    # update velocity vector
    u = u1 + u
    v = v1 + v
    w = w1 + w
    p = p1 + p
    q = q1 + q
    r = r1 + r

    # update position and angular vector
    x = x + x_1
    y = y + y_1
    z = z + z_1
    psi = (psi + psi_1)%(2*np.pi)
    theta = (theta + theta_1)%(2*np.pi)
    phi = (phi + phi_1)%(2*np.pi)

    X.append(x)
    Y.append(y)
    Z.append(z)
    PSI.append(psi)
    THETA.append(theta)
    PHI.append(phi)
    ACC_X.append(float(Acc[0]))
    VITESSE.append(Vtotal)



    simulated_pose.position.x = x
    simulated_pose.position.y = y
    simulated_pose.position.z = z

    q0, qa = navpy.angle2quat(phi, theta, psi, rotation_sequence='ZYX')
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
