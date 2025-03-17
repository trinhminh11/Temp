from CONSTANT import *
import matplotlib.pyplot as plt
from SPARTA_CC import SPARTA_CC
from copy import deepcopy
from math import dist
import random
import numpy as np
import timeit



class Group:
	def __init__(self, T : Target, index):
		self.parent: Group = None
		self.childs: list[Group] = []
		self.index: int = index
		self.T = T
		self.depth = 0
		if type(T) == Target:
			n = len(T.Sensors)
			self.mid = [0,0, 0]

			count = 0
			for i in range(n):
				if self.T.Sensors[i] != 0:
					self.mid[0] += self.T.Sensors[i].v[0]
					self.mid[1] += self.T.Sensors[i].v[1]
					self.mid[2] += self.T.Sensors[i].v[2]
					count  += 1

			self.mid[0] /= count
			self.mid[1] /= count
			self.mid[2] /= count


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
		x = A[0] + (j+1)*(B[0]-A[0])/(add+1)
		y = A[1] + (j+1)*(B[1]-A[1])/(add+1)
		z = A[2] + (j+1)*(B[2]-A[2])/(add+1)

		sensor = (x, y, z)

		res.append(sensor)
	
	return res

def Put_Relays(S1: list[Sensor], S2: list[Sensor]):
	temp_Rn = []
	
	for A, B in zip(S1, S2):
		temp_Rn += Put_Relay(A, B)
	return temp_Rn




def Put_Relay(S1: Sensor, S2:Sensor):
	if type(S2) == int or S1.v == S2.v:
		return []

	temp_Rn = []

	all_sec = False
	all_commu = False

	for T1 in S1.Targets:
		for T2 in S2.Targets:
			P1 = T1.v
			P2 = T2.v

			RP1 = T1.Rsz
			RP2 = T2.Rsz

			# M1 = circle_line_intersection(P1, RP1, S1.v, S2.v)
			# M2 = circle_line_intersection(P2, RP2, S1.v, S2.v)

			M1 = line_sphere_intersection(P1, RP1, S1.v, S2.v)
			M2 = line_sphere_intersection(P2, RP2, S1.v, S2.v)

	
	if M1:
		if M2:
			farA = max(M1, key= lambda x: x[3])
			nearB = min(M2, key= lambda x: x[3])


			if farA[3] <= nearB[3]:
				temp_Rn += addRelay(farA[:-1], nearB[:-1], Rc)

				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)

				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)


			else:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)

		else:
			farA = max(M1, key= lambda x: x[3])
			
			if dist(S1.v, S2.v) <= farA[3]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)
				temp_Rn += addRelay(farA[:-1], S2.v, Rc)

	else:
		if M2:
			nearB = min(M2, key= lambda x: x[3])
			if dist(S1.v, S2.v) < nearB[3]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, nearB[:-1], Rc)
				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)
		else:
			print(S1, S2)
			print(*S1.Targets)
			print(*S2.Targets)
			print("WTF")
			exit()
			return []
	
	return temp_Rn



def brute_force(G1: Group, G2: Group):
	S1 = G1.T.Sensors
	S2 = G2.T.Sensors.copy()

	n = len(S1)

	used = [False for i in range(n)]
	permu = [0 for i in range(n)]

	S2_locked = []
	for index in range(n):
		if S2[index] != 0:
			if S2[index].locked:
				S2_locked.append(index)

	for i in range(n):
		if S2[i] == S1[i] and i not in S2_locked:
			S2_locked.append(i)

	for index in S2_locked:
		used[index] = True
	
	ans = [i for i in range(n)]


	def bt(pos, minRn: list, ans: list):
		if pos == n:
			for lock in S2_locked:
				if permu[lock] != lock:
					return

			temp_Rn = Put_Relays(S1, [S2[permu[i]] for i in range(n)])

			if len(temp_Rn) <= len(minRn):
				minRn.clear()
				minRn += temp_Rn
				ans.clear()
				ans += permu

			return

		if pos in S2_locked:
			permu[pos] = pos
			bt(pos+1, minRn, ans)

		else:
			for i in range(n):
				if not used[i]:
					permu[pos] = i
					used[i] = True
					bt(pos+1, minRn, ans)
					used[i] = False

	#O(Qmax! * Qmax)

	minRn = Put_Relays(S1, S2)
	bt(0, minRn, ans)

	for lock in S2_locked:
		if ans[lock] != lock:
			print("WTF")
			exit()

	S2 = [S2[ans[i]] for i in range(n)]


	G2.T.Sensors = S2

	for index in range(n):
		if G2.T.Sensors[index] == 0:
			G2.T.Sensors[index] = G1.T.Sensors[index]
	
	return minRn
	

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

#Brute_Force Constraint
def Brute_Force(base, Ts: list[Target]):
	Ts.sort(reverse = True, key = lambda x: len(x.Sensors))

	Qmax = Ts[0].q

	base.Sensors = [Sensor(base.v, Targets=[Target(base.v)]) for _ in range(Qmax)]

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
		while j < len(Ts[i].Sensors):
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
			j += 1
	#O(n^2*Qmax)


	GVs = [Group(Ts[i],  i+1) for i in range(len(Ts))]

	GVs = [Group(base, 0)] + GVs


	path = Kruskal(GVs, lambda x: x.mid)


	GVs[0].find_child(path, GVs)

	GVs.sort(key= lambda x: x.depth)

	G_depth: list[list[Group]] = []

	for G in GVs:
		if G.depth == len(G_depth):
			G_depth.append([])

		G_depth[G.depth].append(G)
	
	Rn = []
	for G in GVs:
		for child in G.childs:
			Rn += brute_force(G, child)
	
	

	if is_plot:
		for G in GVs:
			for child in G.childs:
				for q in range(Qmax):
					plt.plot([G.T.Sensors[q].v[0], child.T.Sensors[q].v[0]], [G.T.Sensors[q].v[1], child.T.Sensors[q].v[1]], c = 'green')
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
		# plt.plot(x,y, c = 'r')

		plt.scatter(T[i].v[0], T[i].v[1], c = 'r')

		for S in T[i].Sensors:
			plt.scatter(S.v[0], S.v[1], c = 'orange')

def custom_test():
	import random
	from SPARTA_CC import SPARTA_CC

	global n, Rs, Rsc, Rc, Qmax, is_plot

	change = "N"

	is_plot = True

	base = Base([0, 0, 0])
	H = 200

	n = 20
	Rs = 10
	Qmax = 3


	Rsc = Rs/5
	Rc = Rs*2

	T = []
	Rsz = [random.random()*Rs+Rs for i in range(n)]

	for i in range(n):
		x,y = random.random()*(H-5)+5, random.random()*(H-5)+5
		T.append([x,y])

	Q = [random.randint(1, Qmax) for i in range(n)]

	T = [Target(T[i], Q[i], Rsz[i], []) for i in range(len(T))]
	

	S = SPARTA_CC(T, Rs)

	if is_plot:
		Plotdata(H, T, Rs)


	Rn = Brute_Force(base, T)


	if is_plot:
		plt.show()
		print(len(Rn))
		exit()


def main():
	import pickle
	global n, Rs, Rsc, Rc, Qmax, is_plot

	change = "N"

	file = 'hanoi'

	is_plot = False

	base = Base([0, 0, 0])

	n = 400
	Rs = 40
	Qmax = 10

	if change == "N":
		n = -50
	if change == "R":
		Rs = -5
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
			print(_, run)
			with open(f'{file}//{change}//{n}_{Rs}_{Qmax}_{run+1}.pickle', 'rb') as f:
				T = pickle.load(f)

			if is_plot:
				Plotdata(H, T, Rs)

			starttime = timeit.default_timer()

			Rn = Brute_Force(base, T)

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
	# custom_test()
	main()
