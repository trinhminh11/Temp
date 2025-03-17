from random import *
from math import *
import numpy as np
import matplotlib.pyplot as plt
import timeit

import sys
import os
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from CONSTANT import *

import numpy
from numpy import sqrt, dot, cross
from numpy.linalg import norm


Rs = 40
t = uniform(0, 2*pi)
a = random()
x, y = cos(t)*a*Rs, sin(t)*a*Rs


class Intersection_Point(object):
	def __init__(self, v, parent):
		self.v = v
		self.parent = parent
		self.cover = []

	def is_cover(self, Disk):
		if Disk not in self.cover:
			if dist(self.v, Disk.v) <= Disk.R or Disk in self.parent:
				return True

		return False

	def is_remove(self, rD):
		if self.parent[0] in rD or self.parent[1] in rD:
			return True

		return False

	def remove_cover(self, rD):
		for r in rD:
			if r in self.cover:
				self.cover.remove(r)

class Disk(object):
	def __init__(self, T, R, index):
		self.T = T
		self.v = T.v
		self.q = T.q
		self.R = R
		self.index = index
	

#Finding sensors
def Q_Coverage(T, Rs):
	n = len(T)
	D = [Disk(T[i], Rs, i) for i in range(n)] #set of Disk
	D.sort(key = lambda x: x.q)
	S = [] #set of sensor

	intersection_points = []

	#calc intersection points
	for i in range(len(D)-1):
		for j in range(i+1, len(D)):
			xi, yi = D[i].v
			xj, yj = D[j].v
			d = dist((xi, yi), (xj, yj)) #Dance from Ti to Tj
			if int(d) != 0:
				if d <= 2*Rs: #if Di and Dj intersect
					h = (Rs*Rs-d*d/4)**.5
					x1 = (xj+xi)/2 + h*(yj-yi)/d
					x2 = (xj+xi)/2 - h*(yj-yi)/d
					y1 = (yj+yi)/2 - h*(xj-xi)/d
					y2 = (yj+yi)/2 + h*(xj-xi)/d
					parent = (D[i], D[j])
					intersection_points.append(Intersection_Point((x1,y1), parent))
					intersection_points.append(Intersection_Point((x2,y2), parent))
	# O((n*(n-1)/2)

	#calc point cover by intersection_points
	for point in intersection_points:
		for Di in D:
			if point.is_cover(Di):
				point.cover.append(Di)
	#O(n^2*(n-1)/2)

	#calc number of sensor
	while len(D)!= 0:

		#add Q sensors to Disk that don't intersect with any other Disk
		if len(intersection_points) == 0:
			for Di in D:
				for j in range(Di.q):
					xi, yi = Di.v
					t = uniform(0, 2*pi)
					a = random()
					x, y = cos(t)*a*Rs + xi, sin(t)*a*Rs + yi
					sensor = (x,y)

					tempS = Sensor(sensor, Rs, [Di.T])
					S.append(tempS)
					Di.T.Sensors.append(S[-1])
			D = []

		#calc number of Dick covered and index of that Disk
		else:
			#sort set of intersection points in descending order of number of Target covered
			intersection_points.sort(reverse = True, key = lambda x: len(x.cover))

			#point A
			x1, y1 = intersection_points[0].v

			for i in range(1, len(intersection_points)):
				if intersection_points[i].cover == intersection_points[0].cover:
					#point B
					x2, y2 = intersection_points[i].v
					if x2 == 1587.59521402561 and y2 ==  1782.2906572181284:
						print(*intersection_points[0].cover)
						print(*intersection_points[i].cover)
						exit()
					break

			#add Q sensors to S
			intersection_points[0].cover.sort(key = lambda x: x.q)
			minq = intersection_points[0].cover[0].q
			for i in range(minq):
				#place random
				# a = random()
				# x = x1 + a*(x2-x1)
				# y = y1 + a*(y2-y1)

				#place evenly

				x = x1 + (i+1)*(x2-x1)/(minq+1)
				y = y1 + (i+1)*(y2-y1)/(minq+1)

				sensor = (x, y)
				tempS = Sensor(sensor,Rs, [])
				S.append(tempS)
				for j in range(len(intersection_points[0].cover)):
					intersection_points[0].cover[j].T.Sensors.append(S[-1])
					S[-1].Targets.append(intersection_points[0].cover[j].T)

			rD = []

			for i in range(len(intersection_points[0].cover)):
				intersection_points[0].cover[i].q -= minq
				if intersection_points[0].cover[i].q == 0:
					rD.append(intersection_points[0].cover[i])


			#remove sastified Disk
			for r in rD:
				D.remove(r)


			i = 0
			while i < len(intersection_points):
				if intersection_points[i].is_remove(rD):
					intersection_points.pop(i)
					i -= 1
				else:
					intersection_points[i].remove_cover(rD)
				i += 1
	#O(n^3)
	return S
#O(n^3*qmax)