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
			sensor.append(A[k] + (j+1)*(B[k]-A[k])/(add+1) + np.random.choice([1e-10, 1e-9, 1e-8, -1e-10, -1e-9, -1e-8, 2e-10, 2e-9, 2e-8, -2e-10, -2e-9, -2e-8]))

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

			if Dim == "3D":
				M1.extend(line_sphere_intersection(P1, RP1, S1.v, S2.v))
				M2.extend(line_sphere_intersection(P2, RP2, S1.v, S2.v))
			elif Dim == "2D":
				M1.extend(circle_line_intersection(P1, RP1, S1.v, S2.v))
				M2.extend(circle_line_intersection(P2, RP2, S1.v, S2.v))

	if M1:
		if M2:
			farA = max(M1, key= lambda x: x[-1])
			nearB = min(M2, key= lambda x: x[-1])


			if farA[-1] <= nearB[-1]:
				temp_Rn += addRelay(farA[:-1], nearB[:-1], Rc)

				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)

				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)


			else:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)

		else:
			farA = max(M1, key= lambda x: x[-1])
			
			if dist(S1.v, S2.v) <= farA[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, farA[:-1], Rsc)
				temp_Rn += addRelay(farA[:-1], S2.v, Rc)

	else:
		if M2:
			nearB = min(M2, key= lambda x: x[-1])
			if dist(S1.v, S2.v) < nearB[-1]:
				temp_Rn += addRelay(S1.v, S2.v, Rsc)
			else:
				temp_Rn += addRelay(S1.v, nearB[:-1], Rc)
				temp_Rn += addRelay(nearB[:-1], S2.v, Rsc)
		else:
			# inside sec  range
			temp_Rn += addRelay(S1.v, S2.v, Rsc)
			# print("?")
			# print(S1.v)
			# print(S2.v)
			# print(*S1.Targets)
			# print(*S2.Targets)
			# print(Rs)
			# print(S1.Targets[0].Rsz, S1.Targets[1].Rsz)
			# print(S2.Targets[0].Rsz)
			# exit()
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
		temp_Rn.append(Put_Relay(A, B))
		# temp_Rn += Put_Relay(A, B)
	return temp_Rn


def Hungarian(G1: Group, G2: Group):
	S1 = G1.T.Sensors
	S2 = deepcopy(G2.T.Sensors)


	q = len(S1)

	cost_matrix: list[list] = []

	rows: list[Sensor] = []
	cols: list[Sensor] = []

	S2_locked = []

	for index in range(q):
		if S2[index] != 0:
			if S2[index].locked:
				S2_locked.append(index)

	current_index = -1
	for i in range(q):
		if i not in S2_locked:
			cost_matrix.append([])
			rows.append(i)
			current_index += 1
			for j in range(q):
				if S2[j] == 0 or S2[j].locked:
					continue
				cols.append(j)
				cost_matrix[current_index].append( len(Put_Relay(S1[i], S2[j])) )
	
	
	ans = {i: None for i in range(q)}

	if cost_matrix:
		assignment = Munkres().compute(cost_matrix)

		for row, col in assignment:
			ans[rows[row]] = cols[col]
	

	for lock in S2_locked:
		if ans[lock] != None:
			print("WTF1")
			breakpoint()
		else:
			ans[lock] = lock


	for row in range(q):
		if ans[row] == None:
			for col in range(q):
				if col not in ans.values():
					ans[row] = col
	
	
	for i in range(q):
		if type(S2[i]) != int:
			if S2[i].locked:
				if ans[i] != i:
					print("WTF2")

	temp_S2 = [0 for _ in range(q)]

	for row, col in ans.items():
		temp_S2[row] = S2[col]

	# print("here")

	# print(*S1)
	# print(*S2)
	# print(*temp_S2)

	old = Put_Relays(S1, S2)
	len_old = sum([len(t) for t in old])

	new = Put_Relays(S1, temp_S2)
	len_new = sum([len(t) for t in new])


	# if len_new != sum([len(Put_Relay(S1[row], S2[col])) for row, col in ans.items()]):
	# 	print("wtf")

	if len_old > len_new:
		G2.T.Sensors = [temp_S2[i] for i in range(q)]

	for i in range(q):
		if G2.T.Sensors[i] == 0:
			G2.T.Sensors[i] = deepcopy(G1.T.Sensors[i])
	
	# return new

	return old if len_old < len_new else new
	
# kruskal algorithm
def Kruskal(Gvs: list[Group], key = lambda x: x):
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

	S = [key(Gvs[i]) for i in range(len(Gvs))]
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



#GHS Constraint
def GHS(base, Ts: list[Target]):
	Ts.sort(reverse = True, key = lambda x: len(x.Sensors))

	Qmax = Ts[0].q

	base.Sensors = [Sensor(base.v, Targets=[Target(base.v)]) for _ in range(Qmax)]

	calc_lock(Ts)
	#O(n^2*Qmax)

	for T in Ts:
		for i in range(len(T.Sensors)):
			if type(T.Sensors[i]) != int:
				if T.Sensors[i].locked:
					T.Sensors[i] = deepcopy(T.Sensors[i])
	

	GVs = [Group(Ts[i],  i+1) for i in range(len(Ts))]

	GVs = [Group(base, 0)] + GVs



	path = Kruskal(GVs, lambda x: x.mid)


	GVs[0].find_child(path, GVs)

	GVs.sort(key= lambda x: x.depth)

	# for G in GVs:
	# 	G.draw_path()

	
	Rn = []

	for G in GVs:
		for i in range(len(G.T.Sensors)):
			G.T.Sensors[i].next_connection = [None for _ in range(len(G.childs))] if len(G.childs) != 0 else None

		# if G_idx == 44:
		# 	for i in range(len(GVs[44].T.Sensors)):
		# 		print(GVs[44].T.Sensors[i].next_connection)
		
		for child_idx in range(len(G.childs)):
			Rns = Hungarian(G, G.childs[child_idx])

			for i in range(len(G.T.Sensors)):
				G.T.Sensors[i].next_connection[child_idx] = Rns[i]
				Rn += Rns[i]

		

			# Rn += Hungarian(G, child)
	if is_plot:
		for G in GVs:
			plt.scatter(G.T.v[0], G.T.v[1], c = 'blue')
			for q in range(Qmax):

				c = np.random.rand(3,)
				try:
					for Relay in G.T.Sensors[q].next_connection:
						plt.scatter([G.T.Sensors[q].v[0], Relay[0]], [G.T.Sensors[q].v[1], Relay[1]], c = c)
				except:
					pass

				for child in G.childs:
					plt.plot([G.T.Sensors[q].v[0], child.T.Sensors[q].v[0]], [G.T.Sensors[q].v[1], child.T.Sensors[q].v[1]], c = c)
					

		# for R in Rn:
			# plt.scatter(R[0], R[1], color = 'blue')
		
		# plt.savefig("temp.png")
		plt.show()

	return GVs, Rn

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
	from collections import Counter
	global n, Rs, Rsc, Rc, Qmax, is_plot, Dim

	Dim = "3D"
	file = "hanoi"

	n = 400
	Rs = 40
	Rc = Rs*2
	Rsc = Rs//10
	Qmax = 5
	is_plot = False
	
	
	if Dim == "3D":
		base = Base([0, 0, 0])
		with open(f"3D//{file}.pickle", "rb") as f:
			T: list[Target] = pickle.load(f)
		
		starttime = timeit.default_timer()
		GVs, Rn = GHS(base, T)
		endtime = timeit.default_timer()

		with open(f'3D//{file}GHS.pickle', 'wb') as f:
			pickle.dump(GVs, f)

		
		# Check for duplicates in Rn and print all duplicates with their counts
		# relay_counter = Counter(tuple(relay) for relay in Rn)
		# duplicates = {relay: count for relay, count in relay_counter.items() if count > 1}

		# if duplicates:
		# 	print("Duplicates found in Rn:")
		# 	for duplicate, count in sorted(duplicates.items()):
		# 		print(f"{list(duplicate)} appears {count} times")
		# else:
		# 	print("No duplicates in Rn")

		total = len(Rn)

	elif Dim == "2D":
		base = Base([0, 0])
		total = 0
		print(n, Rs, Rc, Rsc, Qmax)
		for i in range(1, 21):
			with open(f'{file}//{n}_{Rs}_{Qmax}_{i}.pickle', 'rb') as f:
				T: list[Target] = pickle.load(f)
			
			starttime = timeit.default_timer()
			GVs, Rn = GHS(base, T)
			endtime = timeit.default_timer()

			total += len(Rn)

			with open(f'GHS//{file}//{n}_{Rs}_{Qmax}_{i}.pickle', 'wb') as f:
				pickle.dump(GVs, f)
		
		total/=20

	print(total)
	print(endtime-starttime)

if __name__ == "__main__":
	# custom_test()
	main()

