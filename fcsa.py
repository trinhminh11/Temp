from CONSTANT import *
import random, timeit
import matplotlib.pyplot as plt
from math import dist
from munkres import Munkres
import numpy as np
from copy import deepcopy



class Group:
	def __init__(self, T : Target, index):
		self.parent: Group = None
		self.childs: list[Group] = []
		self.index: int = index
		self.T = T
		self.depth = 0

		if type(T) == Target:
			n = len(T.Sensors)
			self.mid = [0] * len(self.T.v)

			
			count = 0
			for i in range(n):
				if self.T.Sensors[i] != 0:
					for j in range(len(self.mid)):
						self.mid[j] += self.T.Sensors[i].v[j]
					count  += 1

			for j in range(len(self.mid)):
				self.mid[j] /= count


		if type(T) == Base:
			self.mid = T.v

	def find_child(self, path, GVs):
		for i in range(len(path)):

			if path[i][0] == self.index and self.parent != GVs[path[i][1]]:
				self.childs.append(GVs[path[i][1]])
				GVs[path[i][1]].parent = self
				GVs[path[i][1]].depth = self.depth + 1


			if path[i][1] == self.index and self.parent != GVs[path[i][0]]:
				self.childs.append(GVs[path[i][0]])
				GVs[path[i][0]].parent = self
				GVs[path[i][0]].depth = self.depth + 1

		for i in range(len(self.childs)):
			self.childs[i].find_child(path, GVs)
		
	def draw_path(self):
		for child in self.childs:
			plt.plot([self.mid[0], child.mid[0]], [self.mid[1], child.mid[1]])

		plt.annotate(str(self.depth), self.mid)	


def addRelay(A, B, r):
	res = []
	c = dist(A, B)

	add = int((c-0.0001)//(r))

	for j in range(add):
		sensor = []

		for k in range(len(A)):
			sensor.append(A[k] + (j+1)*(B[k]-A[k])/(add+1))

		res.append(sensor)
	
	return res


def Put_Relay(S1: Sensor, S2:Sensor):
	if type(S2) == int or type(S1) == int or S1.v == S2.v:
		return []

	temp_Rn = []

	all_sec = False
	all_commu = False

	M1 = []
	M2 = []

	for T1 in S1.Targets:
		for T2 in S2.Targets:
			P1 = T1.v
			P2 = T2.v

			RP1 = T1.Rsz
			RP2 = T2.Rsz

			M1 = circle_line_intersection(P1, RP1, S1.v, S2.v)
			M2 = circle_line_intersection(P2, RP2, S1.v, S2.v)

			# M1.extend(line_sphere_intersection(P1, RP1, S1.v, S2.v))
			# M2.extend(line_sphere_intersection(P2, RP2, S1.v, S2.v))

	
	if M1:
		if M2:
			farA = max(M1, key= lambda x: x[-1])
			nearB = min(M2, key= lambda x: x[-1])


			if farA[-1] <= nearB[-1]:
				temp_Rn += addRelay(farA, nearB, Rc)

				temp_Rn += addRelay(S1.v, farA, Rsc)

				temp_Rn += addRelay(nearB, S2.v, Rsc)


			else:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)

		else:
			farA = max(M1, key= lambda x: x[-1])
			
			if dist(S1.v, S2.v) <= farA[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, farA, Rsc)
				temp_Rn += addRelay(farA, S2.v, Rc)

	else:
		if M2:
			nearB = min(M2, key= lambda x: x[-1])
			if dist(S1.v, S2.v) < nearB[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, nearB, Rc)
				temp_Rn += addRelay(nearB, S2.v, Rsc)
		else:
			temp_Rn += addRelay(S1.v, S2.v, Rsc)
			print("?")
			print(S1.v)
			print(S2.v)
			print(*S1.Targets)
			print(*S2.Targets)
			print(Rs)
			print(S1.Targets[0].Rsz, S1.Targets[1].Rsz)
			print(S2.Targets[0].Rsz)
			exit()
			# print("?")
			# print(S1, S2)
			# print(*S1.Targets)
			# print(*S2.Targets)
			# print(M1, M2, RP1, RP2)
			# print("WTF3")
			# exit()
			# return []
	
	return temp_Rn

# def Put_Relay(S1: Sensor, S2:Sensor):
# 	try:
# 		return addRelay(S1.v, S2.v, Rc)
# 	except AttributeError as e:
# 		return []

def Put_Relays(S1: list[Sensor], S2: list[Sensor]):
	temp_Rn = []
	
	for A, B in zip(S1, S2):
		temp_Rn += Put_Relay(A, B)
	return temp_Rn


def Put_Relay(S: Sensor, B: Base):

	Ms = []

	for T in S.Targets:
		Ms += circle_line_intersection(T.v, T.Rsz, S.v, B.v)
	
	try:
		M = min(Ms, key= lambda x: dist(x, B.v))
		# i = Ms.index(M)
		i = 0
		Rn = []
		for i in range(S.Targets[i].q):
			Rn += addRelay(M, B.v, Rc)
			Rn += addRelay(S.v, M, Rsc)
	except:
		Rn = addRelay(S.v, B.v, Rc)
	

	return Rn

def Put_Relay(S: Sensor, B: Base):

	Ms = []

	for T in S.Targets:
		Ms += line_sphere_intersection(T.v, T.Rsz, S.v, B.v)
	
	try:
		M = min(Ms, key= lambda x: dist(x[:-1], B.v))[:-1]
		# i = Ms.index(M)
		i = 0
		Rn = []
		for i in range(S.Targets[i].q):
			Rn += addRelay(M, B.v, Rc)
			Rn += addRelay(S.v, M, Rsc)
	except:
		Rn = addRelay(S.v, B.v, Rc)
	

	return Rn


#GHS Constraint
def FCSA(base, Ts: list[Target]):
	S = set()
	for T in Ts:
		S |= set(T.Sensors)

	
	S: list[Sensor] = list(S)

	Rn = []

	for s in S:
		Rn += Put_Relay(s, base)

	return Rn
	

#O(n*Qmax!*Qmax + n^2 * Qmax)

def Plotdata(H, T, Rs):
	plt.xlabel('width')
	plt.ylabel("height")
	plt.xlim(0, H) 
	plt.ylim(0, H)

	theta = np.linspace( 0 , 2 * np.pi , 150 )

	for i in range(len(T)):
		plt.scatter(T[i].v[0], T[i].v[1])
		a = Rs * np.cos( theta ) + T[i].v[0]
		b = Rs * np.sin( theta ) + T[i].v[1]

		x = T[i].Rsz * np.cos( theta ) + T[i].v[0]
		y = T[i].Rsz * np.sin( theta ) + T[i].v[1]

		plt.plot(a,b, c = 'r')
		plt.plot(x,y, c = 'g')

		plt.scatter(T[i].v[0], T[i].v[1], c = 'r')

		for S in T[i].Sensors:
			plt.scatter(S.v[0], S.v[1], c = 'orange')



def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	change = "Q"

	file = "sonla"

	is_plot = False

	base = Base([0, 0])

	n = 400
	Rs = 40
	Qmax = 5

	if change == "N":
		n = 100
	if change == "R":
		Rs = 0
	if change == "Q":
		Qmax = 0

	res = []
	res_time = []


	for _ in range(dataset_num):
		if change == "N":
			n += n_step

		if change == "R":
			Rs += Rs_step
		
		if change == "Q":
			Qmax += Qmax_step



		totalRn = 0
		totaltime = 0
		Rsc = Rs/10
		Rc = Rs*2

		for run in range(data_num):
			print(f'{change}//{n}_{Rs}_{Qmax}_{run+1}', end = "\r")
			with open(f'{change}//{n}_{Rs}_{Qmax}_{run+1}.pickle', 'rb') as f:
				T: list[Target] = pickle.load(f)
			
			for t in T:
				t.v = t.v[:2]

			if is_plot:
				Plotdata(H, T, Rs)


			starttime = timeit.default_timer()

			Rn = FCSA(base, T)

			endtime = timeit.default_timer()
			totalRn += len(Rn)
			totaltime += endtime - starttime

			if is_plot:
				plt.show()
				print(len(Rn))
				exit()
		print()

		totalRn = round(totalRn / data_num)
		totaltime = round(totaltime / data_num, 5)

		# print(n, Rs, Rc, Rsc, Qmax, f'{Rs} - {Rs+Rs}', end = "\t")
		res.append(totalRn)
		res_time.append(totaltime)
	
	print(*res, sep = "\n")
	print(*res_time, sep = "\n")


def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	n = 400
	Rs = 40
	Rsc = 4
	Rc = 80
	Qmax = 5
	is_plot = False
	
	base = Base([0, 0, 0])

	file = "sonla"
	with open(f"3D//{file}.pickle", "rb") as f:
		T: list[Target] = pickle.load(f)

	starttime = timeit.default_timer()

	Rn = FCSA(base, T)

	endtime = timeit.default_timer()

	print(len(Rn))
	print(endtime-starttime)


if __name__ == "__main__":
	# custom_test()
	main()

