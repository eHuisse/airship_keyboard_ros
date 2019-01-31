import math
import numpy as np
import matplotlib.pyplot as plt
import quintic_polynomials_planner as qpp


class PursuitControl:

    def trajectory_hauteur(self):
        #Z = []
        #delta_z = (self.gz-self.sz)/len(self.cy)
        #zi = self.sz
        #Z.append(zi)
        #if self.sz < self.gz:
        #    while zi < self.gz - 0.05:
        #        zi = zi + delta_z
        #       Z.append(zi)
        #elif self.sz > self.gz:
        #    while zi > self.gz+ 0.05:
        #        zi = zi + delta_z
        #        Z.append(zi)
        Z = np.linspace(self.sz, self.gz, len(self.cx))
        return Z


    def __init__(self, lookforward, sx=0.0, sy=0.0, sz= 0.0, syaw=0, gx=0.02, gy=0.0, gz=0, gyaw=0.): ###

        # Pose
        self.x = sx
        self.y = sy
        self.z = sz

        # other
        self.target_speed = 1.0  # [m/s]
        self.lf = lookforward
        self.k = 0.5

        self.finish = False
        self.sx = sx
        self.sy = sy
        self.sz = sz
        self.syaw = syaw
        self.gx = gx  # goal x position [m]
        self.gy = gy  # goal y position [m]
        self.gz = gz
        self.gyaw = gyaw
        self.trackline_dist = np.sqrt((sx - gx)**2 + (sy - gy)**2 + (sz - gz)**2)

        self.results = qpp.quinic_polynomials_planner(self.sx, self.sy, self.syaw, self.gx, self.gy, self.gyaw, 1/(self.trackline_dist*50))
        self.cx = self.results[1]
        self.cy = self.results[2]
        self.cz = self.trajectory_hauteur()
       

        #plt.figure(2)
        #plt.plot(self.cy, self.cx, 'b-')
        #plt.figure(3)
        #plt.plot(self.cz)
        #plt.show()

        self.cv = self.results[4]
        self.gam = 0
        self.di = 0
        self.target_ind = 0
        self.v = 0
        


    def calc_target_index(self):

        # search nearest point index
        dx = [self.x - icx for icx in self.cx]
        dy = [self.y - icy for icy in self.cy]
        dz = [self.z - icz for icz in self.cz]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2 + idz **2)) for (idx, idy, idz) in zip(dx, dy, dz)]
        ind = d.index(min(d))
        near_ind = ind
        l = 0

        lookForwardDist = self.k * self.v + self.lf
        while lookForwardDist > l and (ind+1)<len(self.cx):
            dx = self.cx[ind+1] - self.cx[ind]
            dy = self.cy[ind+1] - self.cy[ind]
            dz = self.cz[ind+1] - self.cz[ind]
            l += np.sqrt(dx**2+dy**2+dz**2)
            ind += 1

        dx = self.x - self.cx[-1]
        dy = self.y - self.cy[-1]
        dz = self.z - self.cz[-1]

        dist2goal = np.sqrt(dx**2+dy**2+dz**2)

        if dist2goal < 1.:
            self.finish = True

        return ind, near_ind

    def PIDControl(self, target, current):
        a = self.kp * (target - current)

        return a

    def update(self, x, y, z, v):
        # update position
        self.x = x
        self.y = y
        self.z = z
        self.v = v

        # compute the index in the trajectory
        #self.target_ind = self.calc_target_index(self.cx, self.cy, self.cz)
        # compute ai gain
        #self.ai = self.PIDControl(self.target_speed, self.v)
        # compute rudder and elevator angle
        self.gam, self.di, self.vi, _ = self.pure_pursuit_control() ###
        print("vitesse envoyee par update : ", self.vi)

        return self.gam, self.di, self.vi, self.finish


    def pure_pursuit_control(self):

        ind, near_ind = self.calc_target_index()

        #if pind >= ind:
        #    ind = pind

        if ind < len(self.cx):
            tx = self.cx[ind]
            ty = self.cy[ind]
            tz = self.cz[ind]

        else:
            tx = self.cx[-1]
            ty = self.cy[-1]
            tz = self.cz[-1]
            ind = len(self.cx) - 1

        alpha = math.atan2(ty - self.y, tx - self.x)
        gamma = math.atan2(tz - self.z, math.sqrt(self.x ** 2 + self.y ** 2)) ### angle entre OXY et le point zr
        vitesse = self.cv[near_ind]

        return gamma, alpha, vitesse, ind ###


#                if self.is_trajectory_tracking:
#                    total_speed = np.sqrt(self.estimated_pose.twist.twist.linear.x **2 + self.estimated_pose.twist.twist.linear.z**2 + self.estimated_pose.twist.twist.linear.y**2)
                    #self.wish_command.y, self.wish_command.z, self.wish_command.x, tmp = self.trajectory_tracker.update(self.estimated_pose.pose.pose.position.x,
                    #   self.estimated_pose.pose.pose.position.y, self.estimated_pose.pose.pose.position.z,total_speed)
                    #if tmp:
                    #   self.is_line_tracking = not(tmp)
                    #   self.wish_command.x = 0.
                    #   self.wish_command.y = 0.
                    #   self.wish_command.z = 0.

                    #self.wish_command_pub.publish(self.wish_command)

                #this is the rate of the publishment.