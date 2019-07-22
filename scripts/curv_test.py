#!/usr/bin/env python

import numpy as np

# radius = 5.5
# curv = -1/radius
# nori = np.sign(curv) # orientation of the normal
# speed = 0.5
# yaw = 1.0 + np.pi/2.0
# angle = yaw - nori*np.pi/2.0
# dt = 1.0
# dang = dt*curv*speed

# vel_tang = np.array([speed*np.cos(yaw), speed*np.sin(yaw)])
# acc_norm = np.array([curv*speed*speed*np.cos(yaw + nori*np.pi/2.0), curv*speed*speed*np.sin(yaw + nori*np.pi/2.0)])
# dpos1 = vel_tang*dt + 0.5*acc_norm*dt*dt;

# posA = radius*np.array([np.cos(angle), np.sin(angle)])
# posB = radius*np.array([np.cos(angle + dang), np.sin(angle + dang)])
# dpos2 = posB-posA


# print(vel_tang)
# print(acc_norm)
# print(dpos1)
# print(dpos2)

curv = -1e-9
radius = np.abs(1/curv)
nori = np.sign(curv) # orientation of the normal
speed = 0.5
yaw = 1.0
angle = yaw + np.pi/2.0
dt = 1.0
dang = dt*curv*speed

vel_tang = np.array([speed*np.cos(yaw), speed*np.sin(yaw)])
normal = np.array([speed*np.cos(yaw + np.pi/2), speed*np.sin(yaw + np.pi/2)])
acc_norm = curv*speed*speed*normal
dpos1 = vel_tang*dt + 0.5*acc_norm*dt*dt;

posA = radius*np.array([np.cos(angle), np.sin(angle)])
posB = radius*np.array([np.cos(angle + dang), np.sin(angle + dang)])
dpos2 = posB-posA

# a = np.array([1/curv*np.cos(angle), 1/curv*np.cos(angle)])
# b = np.array([1/curv*np.cos(angle + dang), 1/curv*np.cos(angle + dang)])
# dpos3 = b - a

print(dpos1)
print(dpos2)
# print(dpos3)
