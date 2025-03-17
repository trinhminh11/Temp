from CONSTANT import *
import random, timeit
import matplotlib.pyplot as plt
from math import dist
from SPARTA_CC import SPARTA_CC
from copy import deepcopy
from munkres import Munkres
import numpy as np



class Group:
	def __init__(self, T : Target, index):
		self.parent: Group = None
		self.childs: list[Group] = []
		self.index: int = index
		self.T = T
		self.depth = 0
		if type(T) == Target:
			n = len(T.Sensors)
			self.mid = [0,0]

			count = 0
			for i in range(n):
				if self.T.Sensors[i] != 0:
					self.mid[0] += self.T.Sensors[i].v[0]
					self.mid[1] += self.T.Sensors[i].v[1]
					count  += 1

			self.mid[0] /= count
			self.mid[1] /= count


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

	def nearest_parent(self, index):
		return self.parent.T.Sensors[index] if type(self.T.Sensors[index]) != Sensor else self.parent.nearest_parent(index)

	def draw_path(self):
		for child in self.childs:
			plt.plot([self.mid[0], child.mid[0]], [self.mid[1], child.mid[1]])

		plt.annotate(str(self.depth), self.mid)	

def find_path(index, paths: list[list], GVs: list[Group]):

	for G in GVs:
		# if G.T.Sensors[index] not in paths[index] and type(G.T.Sensors[index]) == Sensor:
		# 	paths[index].append(G.T.Sensors[index])
		
		if type(G.T.Sensors[index]) == Sensor:
			paths[index].append(G.T.Sensors[index])
	
		
def Put_Relays(S1: list[Sensor], S2: list[Sensor]):
	temp_Rn = []
	
	for index in range(len(S1)):
		A = S1[index]
		B = S2[index]

		if B == 0 or A == B:
			S2[index] = S1[index]
			B = A

		else:
			all_sec = False
			all_commu = False
			MA = []
			MB = []

			for T1 in A.Targets:
				for T2 in B.Targets:
					P1 = T1.v
					P2 = T2.v

					RP1 = T1.Rsz
					RP2 = T2.Rsz

					M1 = circle_line_intersection(P1, RP1, A.v, B.v)
					M2 = circle_line_intersection(P2, RP2, A.v, B.v)

					for M in M1:
						MA.append(M)

					for M in M2:
						MB.append(M)

			if A.v[0] < B.v[0]:
				MA.sort(reverse = True)
				MB.sort()

				if len(MA) > 0 and len(MB) > 0:
					if MA[0] >= MB[0]:
						all_sec = True

			elif A.v[0] > B.v[0]:
				MA.sort()
				MB.sort(reverse = True)
				if len(MA) > 0 and len(MB) > 0:
					if MA[0] <= MB[0]:
						all_sec = True

			else:
				if A.v[1] < B.v[1]:
					MA.sort(reverse = True, key = lambda x: x[1])
					MB.sort(key = lambda x: x[1])

					if len(MA) > 0 and len(MB) > 0:
						if MA[1] >= MB[1]:
							all_sec = True

				elif A.v[1] > B.v[1]:
					MA.sort(key = lambda x: x[1])
					MB.sort(reverse = True, key = lambda x: x[1])

					if len(MA) > 0 and len(MB) > 0:
						if MA[1] <= MB[1]:
							all_sec = True

			if len(MA) > 0:
				MA = MA[0]
			else:
				MA = False

			if len(MB) > 0:
				MB = MB[0]
			else:
				MB = False

			if not MA and not MB:
				Ta = A.Targets
				Tb = B.Targets

				for T in Ta:
					if dist(T.v, B.v) <= T.Rsz:
						all_sec = True
						break

				for T in Tb:
					if dist(A.v, T.v) <= T.Rsz:
						all_sec = True
						break

				if not all_sec:
					all_commu = True

			if all_sec:
				c = dist(A.v, B.v)
				add = int((c-0.0001)//(Rsc))
				for j in range(add):
					x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
					y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)

					sensor = (x, y)
					temp_Rn.append(sensor)

			elif all_commu:
				c = dist(A.v, B.v)
				add = int((c-0.0001)//(Rc))
				for j in range(add):
					x = A.v[0] + (j+1)*(B.v[0]-A.v[0])/(add+1)
					y = A.v[1] + (j+1)*(B.v[1]-A.v[1])/(add+1)

					sensor = (x, y)

					temp_Rn.append(sensor)

			else:
				put_A = False
				put_B = False
				if MA and MB:
					put_A = True
					put_B = True
					c = dist(MA, MB)
					add = int((c-0.0001)//(Rc))

					for j in range(add):
						x = MA[0] + (j+1)*(MB[0]-MA[0])/(add+1)
						y = MA[1] + (j+1)*(MB[1]-MA[1])/(add+1)

						sensor = (x, y)

						temp_Rn.append(sensor)

				elif MA and not MB:
					temp_Rn.append(MA)
					put_A = True

					c = dist(MA, B.v)
					add = int((c-0.0001)//(Rc))

					for j in range(add):
						x = MA[0] + (j+1)*(B.v[0]-MA[0])/(add+1)
						y = MA[1] + (j+1)*(B.v[1]-MA[1])/(add+1)

						sensor = (x, y)
						temp_Rn.append(sensor)

				elif MB and not MA:
					temp_Rn.append(MB)
					put_B = True

					c = dist(A.v, MB)
					add = int((c-0.0001)//(Rc))

					for j in range(add):
						x = A.v[0] + (j+1)*(MB[0]-A.v[0])/(add+1)
						y = A.v[1] + (j+1)*(MB[1]-A.v[1])/(add+1)

						sensor = (x, y)
						temp_Rn.append(sensor)

				if put_A:
					c = dist(A.v, MA)
					add = int((c-0.0001)//(Rsc))
					for j in range(add):
						x = A.v[0] + (j+1)*(MA[0]-A.v[0])/(add+1)
						y = A.v[1] + (j+1)*(MA[1]-A.v[1])/(add+1)

						sensor = (x, y)
						temp_Rn.append(sensor)

				if put_B:
					c = dist(B.v, MB)
					add = int((c-0.0001)//(Rsc))
					for j in range(add):
						x = B.v[0] + (j+1)*(MB[0]-B.v[0])/(add+1)
						y = B.v[1] + (j+1)*(MB[1]-B.v[1])/(add+1)

						sensor = (x, y)
						temp_Rn.append(sensor)
	return temp_Rn

	
# kruskal algorithm
def Kruskal(S, key = lambda x: x):
	def find_set(v, parent):

		if v == parent[v]: 
			return v
		p = find_set(parent[v], parent)
		parent[v] = p
		return p

	def union_sets(a, b, parent):
		a = find_set(a, parent)
		b = find_set(b, parent)
		if (a != b):
			parent[b] = a

	S = [key(S[i]) for i in range(len(S))]
	ans = []
	E = []
	parent = [i for i in range(len(S))]

	for i in range(len(S)-1):
		for j in range(i+1, len(S)):
			E.append([dist(S[i], S[j]),i,j])

	E.sort()
	count = 0

	for i in range(len(E)):
		u = E[i][1]
		v = E[i][2]
		if find_set(u, parent) == find_set(v, parent):
			continue
		union_sets(u, v, parent)
		ans.append([u,v])
		count += 1
		if count == len(S)-1:
			break

	return ans
# O(mlog(m))

def calc_lock(Ts: list[Target]):
	Qmax = Ts[0].q

	rT = []

	for i in range(1,len(Ts)):
		for j in range(i):
			if set(Ts[i].Sensors).issubset(set(Ts[j].Sensors)):
				rT.append(Ts[i])
				break
	

	for i in range(len(rT)):
		Ts.remove(rT[i])

	for i in range(1, len(Ts)):
		for j in range(Qmax-len(Ts[i].Sensors)):
			Ts[i].Sensors.append(0)
	

	locked = []
	for i in range(1, len(Ts)):
		j = 0
		is_swap = []
		while j < Qmax:
			if j not in is_swap:
				if Ts[i].Sensors[j] != 0:
					for k in range(i):
						if Ts[i].Sensors[j] in Ts[k].Sensors:
							if not Ts[i].Sensors[j].locked:
								locked.append(Ts[i].Sensors[j])
								Ts[i].Sensors[j].locked = True
								l = Ts[k].Sensors.index(Ts[i].Sensors[j])
								Ts[i].Sensors[j], Ts[i].Sensors[l] = Ts[i].Sensors[l], Ts[i].Sensors[j]
								is_swap.append(l)
								j-=1
								break
						
						for T in Ts:
							if {Ts[i].Sensors[j], Ts[k].Sensors[j]}.issubset(T.Sensors):
								for l in range(Qmax):
									if Ts[i].Sensors[l] != 0:
										for T in Ts:
											if {Ts[i].Sensors[l], Ts[k].Sensors[j]}.issubset(T.Sensors) or {Ts[i].Sensors[j], Ts[k].Sensors[l]}.issubset(T.Sensors):
												break
										else:
											Ts[i].Sensors[j], Ts[i].Sensors[l] = Ts[i].Sensors[l], Ts[i].Sensors[j]
											is_swap.append(l)
											j -= 1
											break
									else:
										for T in Ts:
											if {Ts[i].Sensors[l], Ts[k].Sensors[j]}.issubset(T.Sensors):
												break
										else:
											Ts[i].Sensors[j], Ts[i].Sensors[l] = Ts[i].Sensors[l], Ts[i].Sensors[j]
											is_swap.append(l)
											j -= 1
											break
									
								break

						
			j += 1

#GEMSTONE Constraint
def GEMSTONE(base, Ts: list[Target]):
	# Ts.sort(reverse = True, key = lambda x: [len(x.Sensors), len(max(x.Sensors, key= lambda y: len(y.Targets)).Targets), sum([len(S.Targets) for S in x.Sensors])])
	Ts.sort(reverse = True, key = lambda x: len(x.Sensors))

	Qmax = Ts[0].q

	base.Sensors = [Sensor(base.v, Targets=[Target(base.v)]) for _ in range(Qmax)]

	calc_lock(Ts)
	#O(n^2*Qmax)


	GVs = [Group(Ts[i],  i+1) for i in range(len(Ts))]


	GVs = [Group(base, 0)] + GVs



	paths: list[list] = [[] for _ in range(Qmax)]

	for index in range(Qmax):
		find_path(index, paths, GVs)
	
	
	for i in range(len(paths)-1):
		for j in range(i+1, len(paths)):
			temp = set(paths[i]).intersection(set(paths[j]))
			if temp:
				print(*temp, i, j)
			for S in temp:
				print(paths[i].index(S), paths[i].count(S))
				print(paths[j].index(S), paths[j].count(S))

	
	
		
	exit()

	path = Kruskal(GVs, lambda x: x.mid)

	GVs[0].find_child(path, GVs)

	GVs.sort(key= lambda x: x.depth)

	Rn = []

	for G in GVs:
		for child in G.childs:
			Rn += Put_Relays(G.T.Sensors, child.T.Sensors)
	
	if is_plot:
		for G in GVs:
			for child in G.childs:
				for q in range(Qmax):
					plt.plot([G.T.Sensors[q].v[0], child.T.Sensors[q].v[0]], [G.T.Sensors[q].v[1], child.T.Sensors[q].v[1]])

		for R in Rn:
			plt.scatter(R[0], R[1], color = 'blue')
	
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
		plt.plot(x,y, c = 'r')

		plt.scatter(T[i].v[0], T[i].v[1], c = 'r')

		for S in T[i].Sensors:
			plt.scatter(S.v[0], S.v[1], c = 'orange')


def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	change = "N"

	is_plot = False

	base = Base([0, 0])
	H = 2000

	n = 400
	Rs = 40
	Qmax = 5

	if change == "N":
		n = 100
	if change == "R":
		Rs = 10
	if change == "Q":
		Qmax = 0

	for _ in range(dataset_num):
		if change == "N":
			n += n_step
		if change == "R":
			Rs += Rs_step
		
		if change == "Q":
			Qmax += Qmax_step


		totalRn = 0
		totaltime = 0
		Rsc = Rs/5
		Rc = Rs*2

		for run in range(data_num):
			with open(f'{change}//{n}_{Rs}_{Qmax}_{run+1}.pickle', 'rb') as f:
				T = pickle.load(f)

			if is_plot:
				Plotdata(H, T, Rs)

			starttime = timeit.default_timer()

			Rn = GEMSTONE(base, T)

			endtime = timeit.default_timer()
			totalRn += len(Rn)
			totaltime += endtime - starttime

			if is_plot:
				plt.show()
				print(len(Rn))
				exit()

		totalRn = round(totalRn / data_num)
		totaltime = round(totaltime / data_num, 5)

		# print(n, Rs, Rc, Rsc, Qmax, f'{Rs} - {Rs+Rs}', end = "\t")
		print(totalRn,"\n", totaltime)



if __name__ == "__main__":
	main()
